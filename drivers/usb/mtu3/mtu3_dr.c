// SPDX-License-Identifier: GPL-2.0
/*
 * mtu3_dr.c - dual role switch and host glue layer
 *
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/of_platform.h>
#include <linux/string_choices.h>
#include "mtu3.h"
#include "mtu3_dr.h"
#include "mtu3_debug.h"

#define USB2_PORT 2
#define USB3_PORT 3

static inline struct ssusb_mtk *otg_sx_to_ssusb(struct otg_switch_mtk *otg_sx)
{
	return container_of(otg_sx, struct ssusb_mtk, otg_switch);
}

/*
 * On MT6878 and later the host clocks and power are gated until requested
 * through the IPPC hardware resource-control registers; without the request
 * the IPPC port-control writes (e.g. the U2 host-select) do not take effect
 * and the root hub never sees a device. Request the resources before routing
 * the port to the host, and release them when going back to device mode.
 * No-op on SoCs that do not advertise "mediatek,hwrscs-vers".
 */
static void ssusb_hwrscs_req(struct ssusb_mtk *ssusb, bool on)
{
	void __iomem *ibase = ssusb->ippc_base;
	u32 con, sts;
	int ret;

	if (!ssusb->hwrscs_vers)
		return;

	con = mtu3_readl(ibase, U3D_SSUSB_HWRSCS_CON) & ~SSUSB_HWRSCS_CON_MSK;
	if (on)
		con |= SSUSB_HWRSCS_REQ;
	else
		con &= ~SSUSB_HWRSCS_REQ;
	mtu3_writel(ibase, U3D_SSUSB_HWRSCS_CON, con);

	/* wait for the SPM to (de)assert the resources; non-fatal on timeout */
	ret = readl_poll_timeout(ibase + U3D_SSUSB_HWRSCS_STS, sts,
				 (sts & SSUSB_HWRSCS_REQ) == (con & SSUSB_HWRSCS_REQ),
				 100, 20000);
	if (ret)
		dev_info(ssusb->dev, "hwrscs %s timeout, sts=0x%x\n",
			 str_on_off(on), sts);
}

static void toggle_opstate(struct ssusb_mtk *ssusb)
{
	mtu3_setbits(ssusb->mac_base, U3D_DEVICE_CONTROL, DC_SESSION);
	mtu3_setbits(ssusb->mac_base, U3D_POWER_MANAGEMENT, SOFT_CONN);
}

/* only port0 supports dual-role mode */
static int ssusb_port0_switch(struct ssusb_mtk *ssusb,
	int version, bool tohost)
{
	void __iomem *ibase = ssusb->ippc_base;
	u32 value;

	dev_dbg(ssusb->dev, "%s (switch u%d port0 to %s)\n", __func__,
		version, tohost ? "host" : "device");

	if (version == USB2_PORT) {
		/* 1. power off and disable u2 port0 */
		value = mtu3_readl(ibase, SSUSB_U2_CTRL(0));
		value |= SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS;
		mtu3_writel(ibase, SSUSB_U2_CTRL(0), value);

		/* 2. power on, enable u2 port0 and select its mode */
		value = mtu3_readl(ibase, SSUSB_U2_CTRL(0));
		value &= ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS);
		value = tohost ? (value | SSUSB_U2_PORT_HOST_SEL) :
			(value & (~SSUSB_U2_PORT_HOST_SEL));
		mtu3_writel(ibase, SSUSB_U2_CTRL(0), value);
	} else {
		/* 1. power off and disable u3 port0 */
		value = mtu3_readl(ibase, SSUSB_U3_CTRL(0));
		value |= SSUSB_U3_PORT_PDN | SSUSB_U3_PORT_DIS;
		mtu3_writel(ibase, SSUSB_U3_CTRL(0), value);

		/* 2. power on, enable u3 port0 and select its mode */
		value = mtu3_readl(ibase, SSUSB_U3_CTRL(0));
		value &= ~(SSUSB_U3_PORT_PDN | SSUSB_U3_PORT_DIS);
		value = tohost ? (value | SSUSB_U3_PORT_HOST_SEL) :
			(value & (~SSUSB_U3_PORT_HOST_SEL));
		mtu3_writel(ibase, SSUSB_U3_CTRL(0), value);
	}

	return 0;
}

static void switch_port_to_host(struct ssusb_mtk *ssusb)
{
	void __iomem *ibase = ssusb->ippc_base;
	u32 sts;
	int i;

	dev_dbg(ssusb->dev, "%s\n", __func__);

	/*
	 * MT6878: the IPPC U2 port-select bits only latch after a fresh IP
	 * software reset, which also resets the xHCI MAC underneath the running
	 * host controller. Tear the host controller down so a fresh one can be
	 * brought up against the newly-routed port.
	 */
	of_platform_depopulate(ssusb->dev);

	/*
	 * Enable VBUS up front. When this switch is taken from a sink (the port
	 * had been charging from a host/charger), the MT6375 has to turn its
	 * charge path into an OTG 5V boost, which ramps far slower than from
	 * idle. Starting it here gives the boost the whole power-down/bring-up
	 * below to stabilise, so the device is powered and pulling up D+ well
	 * before the root hub first scans the port; otherwise the scan races the
	 * boost ramp and PORTSC.CCS is read as 0.
	 */
	ssusb_set_vbus(&ssusb->otg_switch, 1);

	/*
	 * Power the SSUSB IP fully down to its sleep state before bringing the
	 * host up. A device->host switch taken after the port had been a USB
	 * *sink* (real VBUS / B-session from a host or charger) otherwise comes
	 * up with PORTSC.CCS stuck at 0 until a reboot: a plain ip_sw_reset and
	 * re-route do not clear the session state the sink leaves latched in the
	 * SSUSB analog. The stock driver passes through switch_port_to_off
	 * between roles, which power-gates the IP; mirror its core here - power
	 * down both ports and the host/device IP, wait for the IP to sleep, and
	 * drop the PHY - so the host is re-initialised from a clean gated state
	 * exactly as at boot.
	 */
	ssusb_host_disable(ssusb);	/* U2/U3 ports PDN+DIS, HOST_PDN */
	mtu3_setbits(ibase, U3D_SSUSB_IP_PW_CTRL2, SSUSB_IP_DEV_PDN);
	if (readl_poll_timeout(ibase + U3D_SSUSB_IP_PW_STS1, sts,
			       sts & SSUSB_IP_SLEEP_STS, 100, 100000))
		dev_info(ssusb->dev, "IP sleep timeout before host switch\n");
	for (i = 0; i < ssusb->num_phys; i++)
		phy_power_off(ssusb->phys[i]);

	/* bring the PHY(s) back up and select host mode from the gated state */
	for (i = 0; i < ssusb->num_phys; i++) {
		phy_power_on(ssusb->phys[i]);
		phy_set_mode(ssusb->phys[i], PHY_MODE_USB_HOST);
	}

	ssusb_ip_sw_reset(ssusb);

	/* request the host hardware resources (MT6878+) */
	ssusb_hwrscs_req(ssusb, true);

	/*
	 * Enable the host IP and all ports exactly as the probe-time host init
	 * does (ssusb_host_init -> ssusb_host_setup -> ssusb_host_enable), not
	 * just a single-port toggle: the probe path is the one that reliably
	 * enumerates a device attached at boot. ssusb_host_enable also waits
	 * for the MAC/xHCI reset-done, so no separate ssusb_check_clocks here.
	 */
	ssusb_host_enable(ssusb);

	/* after all clocks are stable */
	toggle_opstate(ssusb);

	/*
	 * Make sure VBUS (enabled at the top) has fully settled before the root
	 * hub scans the port, so a device attached across a sink->source switch
	 * is powered and detected on the first scan rather than racing the OTG
	 * boost ramp.
	 */
	msleep(50);

	/* bring a fresh xHCI up against the freshly-routed, powered port */
	if (of_platform_populate(ssusb->dev->of_node, NULL, NULL, ssusb->dev))
		dev_err(ssusb->dev, "failed to repopulate host\n");

	for (i = 0; i < ssusb->num_phys; i++)
		phy_set_mode(ssusb->phys[i], PHY_MODE_USB_HOST);
}

static void switch_port_to_device(struct ssusb_mtk *ssusb)
{
	u32 check_clk = 0;
	int i;

	dev_dbg(ssusb->dev, "%s\n", __func__);

	for (i = 0; i < ssusb->num_phys; i++)
		phy_set_mode(ssusb->phys[i], PHY_MODE_USB_DEVICE);

	ssusb_port0_switch(ssusb, USB2_PORT, false);

	if (ssusb->otg_switch.is_u3_drd) {
		ssusb_port0_switch(ssusb, USB3_PORT, false);
		check_clk = SSUSB_U3_MAC_RST_B_STS;
	}

	ssusb_check_clocks(ssusb, check_clk);

	/* release the host clocks/power now the port is back in device mode */
	ssusb_hwrscs_req(ssusb, false);
}

int ssusb_set_vbus(struct otg_switch_mtk *otg_sx, int is_on)
{
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct regulator *vbus = otg_sx->vbus;
	int ret;

	/* vbus is optional */
	if (!vbus)
		return 0;

	dev_dbg(ssusb->dev, "%s: turn %s\n", __func__, str_on_off(is_on));

	if (is_on) {
		ret = regulator_enable(vbus);
		if (ret) {
			dev_err(ssusb->dev, "vbus regulator enable failed\n");
			return ret;
		}
	} else {
		regulator_disable(vbus);
	}

	return 0;
}

static void ssusb_mode_sw_work(struct work_struct *work)
{
	struct otg_switch_mtk *otg_sx =
		container_of(work, struct otg_switch_mtk, dr_work);
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct mtu3 *mtu = ssusb->u3d;
	enum usb_role desired_role = otg_sx->desired_role;
	enum usb_role current_role;
	unsigned long flags;

	current_role = ssusb->is_host ? USB_ROLE_HOST : USB_ROLE_DEVICE;

	/*
	 * The partner went away while in device mode. When the PHY forces a
	 * valid B-session, the controller never sees VBUS drop and keeps the
	 * gadget "connected", which also blocks system suspend. Stop the
	 * device controller and report the disconnect instead; it is
	 * restarted when a device role is requested again.
	 */
	if (desired_role == USB_ROLE_NONE && current_role == USB_ROLE_DEVICE &&
	    otg_sx->default_role == USB_ROLE_DEVICE) {
		if (!mtu->is_active)
			return;

		pm_runtime_get_sync(ssusb->dev);
		mtu3_stop(mtu);
		spin_lock_irqsave(&mtu->lock, flags);
		if (mtu->g.speed != USB_SPEED_UNKNOWN)
			mtu3_gadget_disconnect(mtu);
		mtu->g.speed = USB_SPEED_UNKNOWN;
		mtu->connected = 0;
		spin_unlock_irqrestore(&mtu->lock, flags);
		pm_runtime_put(ssusb->dev);
		return;
	}

	if (desired_role == USB_ROLE_NONE) {
		/* the default mode is host as probe does */
		desired_role = USB_ROLE_HOST;
		if (otg_sx->default_role == USB_ROLE_DEVICE)
			desired_role = USB_ROLE_DEVICE;
	}

	if (current_role == desired_role) {
		/* a partner is back after a disconnect in device mode */
		if (desired_role == USB_ROLE_DEVICE && !mtu->is_active) {
			pm_runtime_get_sync(ssusb->dev);
			mtu3_start(mtu);
			pm_runtime_put(ssusb->dev);
		}
		return;
	}

	dev_dbg(ssusb->dev, "set role : %s\n", usb_role_string(desired_role));
	mtu3_dbg_trace(ssusb->dev, "set role : %s", usb_role_string(desired_role));
	pm_runtime_get_sync(ssusb->dev);

	switch (desired_role) {
	case USB_ROLE_HOST:
		ssusb_set_force_mode(ssusb, MTU3_DR_FORCE_HOST);
		mtu3_stop(mtu);
		/* switch_port_to_host() enables VBUS before populating xHCI */
		switch_port_to_host(ssusb);
		ssusb->is_host = true;
		break;
	case USB_ROLE_DEVICE:
		ssusb_set_force_mode(ssusb, MTU3_DR_FORCE_DEVICE);
		ssusb->is_host = false;
		ssusb_set_vbus(otg_sx, 0);
		switch_port_to_device(ssusb);
		mtu3_start(mtu);
		break;
	case USB_ROLE_NONE:
	default:
		dev_err(ssusb->dev, "invalid role\n");
	}
	pm_runtime_put(ssusb->dev);
}

static void ssusb_set_mode(struct otg_switch_mtk *otg_sx, enum usb_role role)
{
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);

	if (ssusb->dr_mode != USB_DR_MODE_OTG)
		return;

	otg_sx->desired_role = role;
	queue_work(system_freezable_wq, &otg_sx->dr_work);
}

static int ssusb_id_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct otg_switch_mtk *otg_sx =
		container_of(nb, struct otg_switch_mtk, id_nb);

	ssusb_set_mode(otg_sx, event ? USB_ROLE_HOST : USB_ROLE_DEVICE);

	return NOTIFY_DONE;
}

static int ssusb_extcon_register(struct otg_switch_mtk *otg_sx)
{
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct extcon_dev *edev = otg_sx->edev;
	int ret;

	/* extcon is optional */
	if (!edev)
		return 0;

	otg_sx->id_nb.notifier_call = ssusb_id_notifier;
	ret = devm_extcon_register_notifier(ssusb->dev, edev, EXTCON_USB_HOST,
					&otg_sx->id_nb);
	if (ret < 0) {
		dev_err(ssusb->dev, "failed to register notifier for USB-HOST\n");
		return ret;
	}

	ret = extcon_get_state(edev, EXTCON_USB_HOST);
	dev_dbg(ssusb->dev, "EXTCON_USB_HOST: %d\n", ret);

	/* default as host, switch to device mode if needed */
	if (!ret)
		ssusb_set_mode(otg_sx, USB_ROLE_DEVICE);

	return 0;
}

/*
 * We provide an interface via debugfs to switch between host and device modes
 * depending on user input.
 * This is useful in special cases, such as uses TYPE-A receptacle but also
 * wants to support dual-role mode.
 */
void ssusb_mode_switch(struct ssusb_mtk *ssusb, int to_host)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;

	ssusb_set_mode(otg_sx, to_host ? USB_ROLE_HOST : USB_ROLE_DEVICE);
}

void ssusb_set_force_mode(struct ssusb_mtk *ssusb,
			  enum mtu3_dr_force_mode mode)
{
	u32 value;

	value = mtu3_readl(ssusb->ippc_base, SSUSB_U2_CTRL(0));
	switch (mode) {
	case MTU3_DR_FORCE_DEVICE:
		value |= SSUSB_U2_PORT_FORCE_IDDIG | SSUSB_U2_PORT_RG_IDDIG;
		break;
	case MTU3_DR_FORCE_HOST:
		value |= SSUSB_U2_PORT_FORCE_IDDIG;
		value &= ~SSUSB_U2_PORT_RG_IDDIG;
		break;
	case MTU3_DR_FORCE_NONE:
		value &= ~(SSUSB_U2_PORT_FORCE_IDDIG | SSUSB_U2_PORT_RG_IDDIG);
		break;
	default:
		return;
	}
	mtu3_writel(ssusb->ippc_base, SSUSB_U2_CTRL(0), value);
}

static int ssusb_role_sw_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct ssusb_mtk *ssusb = usb_role_switch_get_drvdata(sw);
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;

	ssusb_set_mode(otg_sx, role);

	return 0;
}

static enum usb_role ssusb_role_sw_get(struct usb_role_switch *sw)
{
	struct ssusb_mtk *ssusb = usb_role_switch_get_drvdata(sw);

	return ssusb->is_host ? USB_ROLE_HOST : USB_ROLE_DEVICE;
}

static int ssusb_role_sw_register(struct otg_switch_mtk *otg_sx)
{
	struct usb_role_switch_desc role_sx_desc = { 0 };
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct device *dev = ssusb->dev;
	enum usb_dr_mode mode;

	if (!otg_sx->role_sw_used)
		return 0;

	mode = usb_get_role_switch_default_mode(dev);
	if (mode == USB_DR_MODE_PERIPHERAL)
		otg_sx->default_role = USB_ROLE_DEVICE;
	else
		otg_sx->default_role = USB_ROLE_HOST;

	role_sx_desc.set = ssusb_role_sw_set;
	role_sx_desc.get = ssusb_role_sw_get;
	role_sx_desc.fwnode = dev_fwnode(dev);
	role_sx_desc.driver_data = ssusb;
	role_sx_desc.allow_userspace_control = true;
	otg_sx->role_sw = usb_role_switch_register(dev, &role_sx_desc);
	if (IS_ERR(otg_sx->role_sw))
		return PTR_ERR(otg_sx->role_sw);

	ssusb_set_mode(otg_sx, otg_sx->default_role);

	return 0;
}

int ssusb_otg_switch_init(struct ssusb_mtk *ssusb)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;
	int ret = 0;

	INIT_WORK(&otg_sx->dr_work, ssusb_mode_sw_work);

	if (otg_sx->manual_drd_enabled)
		ssusb_dr_debugfs_init(ssusb);
	else if (otg_sx->role_sw_used)
		ret = ssusb_role_sw_register(otg_sx);
	else
		ret = ssusb_extcon_register(otg_sx);

	return ret;
}

void ssusb_otg_switch_exit(struct ssusb_mtk *ssusb)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;

	cancel_work_sync(&otg_sx->dr_work);
	usb_role_switch_unregister(otg_sx->role_sw);
}
