// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Legacy firmware interface of the MT7921 compatible WiFi integrated in
 * MediaTek SoCs.
 *
 * The firmware of these SoCs predates the unified commands used by the
 * mt7921 driver to manage interfaces, BSSes and stations. Translate them to
 * the legacy commands the firmware understands, and the legacy events back.
 * Unified commands get a synthesized successful answer, since the legacy ones
 * are not acknowledged.
 */

#include <linux/etherdevice.h>

#include "mt7921.h"
#include "mcu.h"
#include "soc.h"
#include "soc_mcu.h"

/* The interface's own WTBL entry, see mt7921_add_interface() */
static u8 mt7921_soc_bmc_wlan_idx(u8 bss_idx)
{
	return MT792x_WTBL_RESERVED - bss_idx;
}

struct mt7921_soc_vif_lookup {
	u8 bss_idx;
	struct ieee80211_vif *vif;
};

static void mt7921_soc_vif_iter(void *priv, u8 *mac, struct ieee80211_vif *vif)
{
	struct mt792x_vif *mvif = (struct mt792x_vif *)vif->drv_priv;
	struct mt7921_soc_vif_lookup *lookup = priv;

	if (mvif->bss_conf.mt76.idx == lookup->bss_idx)
		lookup->vif = vif;
}

static struct ieee80211_vif *mt7921_soc_find_vif(struct mt792x_dev *dev, u8 bss_idx)
{
	struct mt7921_soc_vif_lookup lookup = { .bss_idx = bss_idx };

	ieee80211_iterate_active_interfaces_atomic(mt76_hw(dev),
						   IEEE80211_IFACE_ITER_RESUME_ALL,
						   mt7921_soc_vif_iter, &lookup);

	return lookup.vif;
}

/* Queue a legacy command, called with the MCU mutex held */
int mt7921_soc_legacy_send(struct mt792x_dev *dev, int id, const void *data, int len)
{
	struct mt76_dev *mdev = &dev->mt76;
	struct sk_buff *skb;
	int ret;

	skb = mt76_mcu_msg_alloc(mdev, data, len);
	if (!skb)
		return -ENOMEM;

	ret = mt76_connac2_mcu_fill_message(mdev, skb,
					    FIELD_PREP(__MCU_CMD_FIELD_ID, id) |
					    __MCU_CMD_FIELD_CE, NULL);
	if (ret) {
		dev_kfree_skb(skb);
		return ret;
	}

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[MT_MCUQ_WM], skb, 0);
}

static u16 mt7921_soc_rate_set(struct ieee80211_supported_band *sband, u32 bitmap)
{
	static const struct {
		u16 bitrate;
		u16 bit;
	} map[] = {
		{ 10, BIT(0) }, { 20, BIT(1) }, { 55, BIT(2) }, { 110, BIT(3) },
		{ 60, BIT(6) }, { 90, BIT(7) }, { 120, BIT(8) }, { 180, BIT(9) },
		{ 240, BIT(10) }, { 360, BIT(11) }, { 480, BIT(12) }, { 540, BIT(13) },
	};
	u16 set = 0;
	int i, j;

	for (i = 0; sband && i < sband->n_bitrates; i++) {
		if (!(bitmap & BIT(i)))
			continue;

		for (j = 0; j < ARRAY_SIZE(map); j++)
			if (map[j].bitrate == sband->bitrates[i].bitrate)
				set |= map[j].bit;
	}

	return set;
}

/* Lowest rate of a rate set as hardware rate code */
static u16 mt7921_soc_lowest_rate(u16 rate_set)
{
	if (rate_set & GENMASK(3, 0))
		return 0;			/* CCK 1M long preamble */

	return (MT_PHY_TYPE_OFDM << 6) | 0xb;	/* OFDM 6M */
}

static u8 mt7921_soc_phy_type_set(struct ieee80211_sta *sta, enum nl80211_band band,
				  u16 rate_set)
{
	u8 set = 0;

	if (band == NL80211_BAND_2GHZ) {
		if (rate_set & GENMASK(3, 0))
			set |= LEGACY_PHY_TYPE_HR_DSSS;
		if (rate_set & GENMASK(13, 6))
			set |= LEGACY_PHY_TYPE_ERP;
	} else {
		set |= LEGACY_PHY_TYPE_OFDM;
	}

	if (sta && sta->deflink.ht_cap.ht_supported)
		set |= LEGACY_PHY_TYPE_HT;
	if (sta && sta->deflink.vht_cap.vht_supported)
		set |= LEGACY_PHY_TYPE_VHT;
	if (sta && sta->deflink.he_cap.has_he)
		set |= LEGACY_PHY_TYPE_HE;

	return set;
}

static u8 mt7921_soc_band(enum nl80211_band band)
{
	switch (band) {
	case NL80211_BAND_6GHZ:
		return LEGACY_BAND_6G;
	case NL80211_BAND_5GHZ:
		return LEGACY_BAND_5G;
	default:
		return LEGACY_BAND_2G4;
	}
}

static void mt7921_soc_fill_rlm(struct mt7921_legacy_bss_rlm *rlm,
				struct ieee80211_vif *vif, u8 bss_idx)
{
	struct cfg80211_chan_def *chandef = &vif->bss_conf.chanreq.oper;
	struct ieee80211_channel *chan = chandef->chan;
	int ctrl, center;

	rlm->bss_idx = bss_idx;
	rlm->tx_nss = 2;
	rlm->rx_nss = 2;
	rlm->use_short_preamble = vif->bss_conf.use_short_preamble;
	rlm->use_short_slot_time = vif->bss_conf.use_short_slot;
	rlm->ht_protect_mode = vif->bss_conf.ht_operation_mode &
			       IEEE80211_HT_OP_MODE_PROTECTION;
	rlm->ht_op_info2 = cpu_to_le16(vif->bss_conf.ht_operation_mode);
	if (!chan)
		return;

	ctrl = chan->hw_value;
	center = ieee80211_frequency_to_channel(chandef->center_freq1);
	rlm->band = mt7921_soc_band(chan->band);
	rlm->primary_channel = ctrl;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_40:
		rlm->sco = ctrl < center ? 1 : 3;
		break;
	case NL80211_CHAN_WIDTH_80:
		rlm->sco = ctrl < center ? 1 : 3;
		rlm->vht_channel_width = 1;
		rlm->vht_channel_freq_s1 = center;
		break;
	case NL80211_CHAN_WIDTH_160:
		rlm->sco = ctrl < center ? 1 : 3;
		rlm->vht_channel_width = 2;
		rlm->vht_channel_freq_s1 = center;
		break;
	case NL80211_CHAN_WIDTH_80P80:
		rlm->sco = ctrl < center ? 1 : 3;
		rlm->vht_channel_width = 3;
		rlm->vht_channel_freq_s1 = center;
		rlm->vht_channel_freq_s2 =
			ieee80211_frequency_to_channel(chandef->center_freq2);
		break;
	default:
		break;
	}
}

static struct ieee80211_sta *mt7921_soc_find_ap(struct ieee80211_vif *vif)
{
	if (is_zero_ether_addr(vif->bss_conf.bssid))
		return NULL;

	return ieee80211_find_sta(vif, vif->bss_conf.bssid);
}

static int mt7921_soc_send_bss_info(struct mt792x_dev *dev, struct ieee80211_vif *vif,
				    struct mt7921_soc_bss *sbss)
{
	struct mt792x_vif *mvif = (struct mt792x_vif *)vif->drv_priv;
	u8 idx = mvif->bss_conf.mt76.idx;
	struct mt7921_legacy_bss_info req = {
		.bss_idx = idx,
		.connection_state = sbss->connected ? LEGACY_MEDIA_STATE_CONNECTED :
						      LEGACY_MEDIA_STATE_DISCONNECTED,
		.is_qbss = vif->bss_conf.qos,
		.sta_rec_idx_of_ap = 0xfe,
		.bmc_wlan_idx = sbss->active ? mt7921_soc_bmc_wlan_idx(idx) : 0xff,
		.dbdc_band = LEGACY_BAND_AUTO,
		.wmm_set = mvif->bss_conf.mt76.wmm_idx,
		.nss = 2,
		.he_op_params = { 0xf0, 0x3f, 0x00 },
	};
	struct cfg80211_chan_def *chandef = &vif->bss_conf.chanreq.oper;
	struct ieee80211_supported_band *sband = NULL;
	struct ieee80211_sta *sta = NULL;
	u16 rates = 0, basic = 0;

	if (chandef->chan)
		sband = mt76_hw(dev)->wiphy->bands[chandef->chan->band];

	if (sbss->ap) {
		req.current_op_mode = LEGACY_OP_MODE_AP;
		req.is_ap_mode = 1;
		req.sta_rec_idx_of_ap = 0;
	}

	req.ssid_len = min_t(int, vif->cfg.ssid_len, IEEE80211_MAX_SSID_LEN);
	memcpy(req.ssid, vif->cfg.ssid, req.ssid_len);
	ether_addr_copy(req.bssid, vif->bss_conf.bssid);

	rcu_read_lock();
	if (!sbss->ap)
		sta = mt7921_soc_find_ap(vif);
	if (sta && sband)
		rates = mt7921_soc_rate_set(sband, sta->deflink.supp_rates[sband->band]);
	else if (sbss->ap && sband)
		/* An access point offers all rates */
		rates = mt7921_soc_rate_set(sband, ~0);
	if (sta && sbss->connected)
		req.sta_rec_idx_of_ap = ((struct mt792x_sta *)sta->drv_priv)->deflink.wcid.idx;
	if (sband) {
		basic = mt7921_soc_rate_set(sband, vif->bss_conf.basic_rates);
		req.phy_type_set = mt7921_soc_phy_type_set(sta, sband->band, rates);
		req.non_ht_basic_phy_type = sband->band != NL80211_BAND_2GHZ ? 3 :
					    (rates & GENMASK(13, 6)) ? 1 : 0;
	}
	rcu_read_unlock();

	if (sbss->ap && sband) {
		if (chandef->width != NL80211_CHAN_WIDTH_20_NOHT)
			req.phy_type_set |= LEGACY_PHY_TYPE_HT;
		if (chandef->width != NL80211_CHAN_WIDTH_20_NOHT &&
		    sband->band != NL80211_BAND_2GHZ && sband->vht_cap.vht_supported)
			req.phy_type_set |= LEGACY_PHY_TYPE_VHT;
		if (vif->bss_conf.he_support)
			req.phy_type_set |= LEGACY_PHY_TYPE_HE;
	}

	req.operational_rate_set = cpu_to_le16(rates);
	req.bss_basic_rate_set = cpu_to_le16(basic);
	req.hw_default_fixed_rate_code = cpu_to_le16(mt7921_soc_lowest_rate(basic));

	/* The firmware takes the key management from the keys installed later */
	if (sbss->ap) {
		req.auth_mode = sbss->privacy ? 7 : 0;
		req.enc_status = sbss->privacy ? 6 : 1;
	} else if (vif->cfg.assoc || sbss->connected) {
		req.auth_mode = vif->bss_conf.bss &&
				(vif->bss_conf.bss->capability & WLAN_CAPABILITY_PRIVACY) ?
				7 : 0;
		req.enc_status = req.auth_mode ? 6 : 1;
	} else {
		req.enc_status = 1;
	}

	mt7921_soc_fill_rlm(&req.rlm, vif, idx);
	req.rlm.vht_basic_mcs_set = cpu_to_le16(basic);

	return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_SET_BSS_INFO, &req, sizeof(req));
}

static int mt7921_soc_send_activate(struct mt792x_dev *dev, u8 bss_idx, u8 omac_idx,
				    const u8 *addr, bool ap, bool active)
{
	struct mt7921_legacy_bss_activate req = {
		.bss_idx = bss_idx,
		.active = active,
		.network_type = ap ? LEGACY_NETWORK_TYPE_P2P : LEGACY_NETWORK_TYPE_AIS,
		.omac_idx = omac_idx,
		.bmc_wlan_idx = mt7921_soc_bmc_wlan_idx(bss_idx),
	};

	if (active)
		ether_addr_copy(req.mac_addr, addr);

	return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_BSS_ACTIVATE_CTRL,
				      &req, sizeof(req));
}

/* Abort a granted channel request the BSS kept, see mt7921_soc_roc() */
static void mt7921_soc_abort_channel(struct mt792x_dev *dev, u8 bss_idx)
{
	struct mt7921_soc_bss *sbss = &mt7921_soc_dev(dev)->bss[bss_idx];
	struct mt7921_legacy_ch_privilege req = {
		.bss_idx = bss_idx,
		.token_id = sbss->abort_token,
		.action = LEGACY_CH_ACTION_ABORT,
		.dbdc_band = LEGACY_BAND_AUTO,
	};

	if (!sbss->abort_pending)
		return;

	sbss->abort_pending = false;
	mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_CH_PRIVILEGE, &req, sizeof(req));
}

static int mt7921_soc_remove_sta(struct mt792x_dev *dev, u8 action, u8 sta_idx,
				 u8 bss_idx)
{
	struct mt7921_legacy_remove_sta req = {
		.action = action,
		.sta_idx = sta_idx,
		.bss_idx = bss_idx,
	};

	return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_REMOVE_STA_RECORD,
				      &req, sizeof(req));
}

static int mt7921_soc_pm_bss(struct mt792x_dev *dev, struct ieee80211_vif *vif,
			     bool connected)
{
	struct mt792x_vif *mvif = (struct mt792x_vif *)vif->drv_priv;
	u8 idx = mvif->bss_conf.mt76.idx;

	if (connected) {
		struct mt7921_legacy_pm_bss_connected req = {
			.bss_idx = idx,
			.dtim_period = vif->bss_conf.dtim_period ?: 1,
			.aid = cpu_to_le16(vif->cfg.aid),
			.beacon_interval = cpu_to_le16(vif->bss_conf.beacon_int),
		};

		return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_PM_BSS_CONNECTED,
					      &req, sizeof(req));
	} else {
		struct mt7921_legacy_pm_bss_abort req = {
			.bss_idx = idx,
		};

		return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_PM_BSS_ABORT,
					      &req, sizeof(req));
	}
}

static int mt7921_soc_ps_profile(struct mt792x_dev *dev, u8 bss_idx, bool ps)
{
	struct {
		u8 bss_idx;
		u8 ps_profile;
		u8 rsv[2];
	} __packed req = {
		.bss_idx = bss_idx,
		/* CAM or fast PS-Poll */
		.ps_profile = ps ? 2 : 0,
	};

	return mt7921_soc_legacy_send(dev, MCU_CE_CMD_SET_PS_PROFILE, &req, sizeof(req));
}

/* UNI DEV_INFO_UPDATE: own address of an interface, BSS_ACTIVATE_CTRL */
static int mt7921_soc_dev_info(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct {
		u8 omac_idx;
		u8 band_idx;
		__le16 pad;
		__le16 tag;
		__le16 len;
		u8 active;
		u8 link_idx;
		u8 omac_addr[ETH_ALEN];
	} __packed *req = (void *)skb->data;
	struct mt7921_soc_bss *sbss;
	struct ieee80211_vif *vif;
	u8 idx;

	if (skb->len < sizeof(*req))
		return -EINVAL;

	/* The BSS index of an interface is also its own MAC index */
	idx = req->omac_idx;
	if (idx >= ARRAY_SIZE(mt7921_soc_dev(dev)->bss))
		return -EINVAL;

	sbss = &mt7921_soc_dev(dev)->bss[idx];
	if (!req->active)
		mt7921_soc_abort_channel(dev, idx);
	if (!req->active && sbss->connected) {
		sbss->connected = false;
		vif = mt7921_soc_find_vif(dev, idx);
		if (vif)
			mt7921_soc_pm_bss(dev, vif, false);
	}
	sbss->active = req->active;
	if (req->active) {
		sbss->ap = mt7921_soc_dev(dev)->adding_ap;
		sbss->privacy = false;
	}

	mt7921_soc_send_activate(dev, idx, req->omac_idx, req->omac_addr, sbss->ap,
				 req->active);
	if (req->active)
		/* The firmware defaults to power save */
		mt7921_soc_ps_profile(dev, idx, false);
	else
		mt7921_soc_remove_sta(dev, 1, 27, idx);

	return 0;
}

/*
 * UNI beacon content: TXD and beacon frame, the legacy template has the IEs
 * only. Returns true if the privacy of the access point changed.
 */
static bool mt7921_soc_beacon(struct mt792x_dev *dev, u8 bss_idx,
			      struct mt7921_soc_bss *sbss, const void *tlv)
{
	const struct {
		__le16 tag;
		__le16 len;
		__le16 tim_ie_pos;
		__le16 csa_ie_pos;
		__le16 bcc_ie_pos;
		u8 enable;
		u8 type;
		__le16 pkt_len;
		u8 pkt[];
	} __packed *bcn = tlv;
	int hdr = MT_TXD_SIZE + offsetof(struct ieee80211_mgmt, u.beacon.variable);
	int len = le16_to_cpu(bcn->pkt_len) - hdr;
	const struct ieee80211_mgmt *mgmt = (const void *)(bcn->pkt + MT_TXD_SIZE);
	struct mt7921_legacy_beacon *req;
	bool privacy;

	if (!bcn->enable || len < 0 || len > LEGACY_BEACON_MAX_IE_LEN ||
	    le16_to_cpu(bcn->pkt_len) > le16_to_cpu(bcn->len) - sizeof(*bcn))
		return false;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return false;

	req->update_method = LEGACY_BEACON_UPDATE_ALL;
	req->bss_idx = bss_idx;
	req->capability = mgmt->u.beacon.capab_info;
	req->ie_len = cpu_to_le16(len);
	memcpy(req->ie, mgmt->u.beacon.variable, len);
	mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_UPDATE_BEACON, req,
			       offsetof(struct mt7921_legacy_beacon, ie) + len);
	kfree(req);

	privacy = le16_to_cpu(mgmt->u.beacon.capab_info) & WLAN_CAPABILITY_PRIVACY;
	if (privacy == sbss->privacy)
		return false;

	sbss->privacy = privacy;
	return true;
}

/* UNI BSS_INFO_UPDATE: SET_BSS_INFO with the current state of the BSS */
static int mt7921_soc_bss_info(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt76_connac_bss_basic_tlv *basic = NULL;
	bool rlm = false, was_connected;
	void *ps = NULL, *bcn = NULL;
	struct mt7921_soc_bss *sbss;
	struct ieee80211_vif *vif;
	u8 bss_idx;
	int ofs;

	if (skb->len < 4)
		return -EINVAL;

	bss_idx = skb->data[0];
	if (bss_idx >= ARRAY_SIZE(mt7921_soc_dev(dev)->bss))
		return 0;

	for (ofs = 4; ofs + sizeof(struct tlv) <= skb->len;) {
		struct tlv *tlv = (struct tlv *)(skb->data + ofs);
		u16 len = le16_to_cpu(tlv->len);

		if (len < sizeof(*tlv) || ofs + len > skb->len)
			break;

		switch (le16_to_cpu(tlv->tag)) {
		case UNI_BSS_INFO_BASIC:
			basic = (void *)tlv;
			break;
		case UNI_BSS_INFO_RLM:
			rlm = true;
			break;
		case UNI_BSS_INFO_PS:
			ps = (void *)tlv;
			break;
		case UNI_BSS_INFO_BCN_CONTENT:
			bcn = (void *)tlv;
			break;
		default:
			break;
		}
		ofs += len;
	}

	if (ps) {
		/* Power save with the AP, mac80211 decides when to use it */
		struct {
			__le16 tag;
			__le16 len;
			u8 profile;
		} __packed *tlv = ps;

		mt7921_soc_ps_profile(dev, bss_idx, tlv->profile);
	}

	sbss = &mt7921_soc_dev(dev)->bss[bss_idx];
	/* The key management of an access point follows its beacon */
	if (bcn && mt7921_soc_beacon(dev, bss_idx, sbss, bcn))
		rlm = true;

	/* Beacon filter and similar settings are not handled */
	if (!basic && !rlm)
		return 0;

	vif = mt7921_soc_find_vif(dev, bss_idx);
	if (!vif)
		return 0;

	was_connected = sbss->connected;
	if (basic)
		sbss->connected = sbss->active && !basic->conn_state;

	if (was_connected && !sbss->connected) {
		mt7921_soc_abort_channel(dev, bss_idx);
		mt7921_soc_pm_bss(dev, vif, false);
	}

	mt7921_soc_send_bss_info(dev, vif, sbss);

	if (was_connected && !sbss->connected)
		mt7921_soc_remove_sta(dev, 1, 27, bss_idx);

	return 0;
}

static u8 mt7921_soc_cipher(u8 mcu_cipher)
{
	switch (mcu_cipher) {
	case MCU_CIPHER_WEP40:
		return 1;
	case MCU_CIPHER_WEP104:
		return 5;
	case MCU_CIPHER_TKIP:
		return 2;
	case MCU_CIPHER_AES_CCMP:
		return 4;
	case MCU_CIPHER_CCMP_256:
		return 10;
	case MCU_CIPHER_GCMP:
		return 11;
	case MCU_CIPHER_GCMP_256:
		return 12;
	case MCU_CIPHER_BIP_CMAC_128:
		return 6;
	default:
		return 0;
	}
}

static int mt7921_soc_key(struct mt792x_dev *dev, struct mt76_wcid *wcid,
			  struct ieee80211_vif *vif, struct sta_rec_sec *sec)
{
	struct mt792x_vif *mvif = (struct mt792x_vif *)vif->drv_priv;
	bool group = wcid == &mvif->sta.deflink.wcid;
	bool ap = vif->type == NL80211_IFTYPE_AP;
	struct ieee80211_sta *sta = wcid_to_sta(wcid);
	int i;

	if (sec->add != SET_KEY) {
		struct mt7921_legacy_key req = {
			.key_type = !group,
			.bss_idx = mvif->bss_conf.mt76.idx,
			.wlan_idx = wcid->idx,
		};

		/* The pairwise key goes away with the station */
		if (!group)
			return 0;

		return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_ADD_REMOVE_KEY,
					      &req, sizeof(req));
	}

	for (i = 0; i < sec->n_cipher && i < ARRAY_SIZE(sec->key); i++) {
		struct sec_key *key = &sec->key[i];
		struct mt7921_legacy_key req = {
			.add_remove = 1,
			/* An access point sends with the group key too */
			.tx_key = !group || ap,
			.key_type = !group,
			.is_authenticator = ap,
			.bss_idx = mvif->bss_conf.mt76.idx,
			.algorithm_id = mt7921_soc_cipher(key->cipher_id),
			.key_id = key->key_id,
			.key_len = key->key_len,
			.wlan_idx = wcid->idx,
			.mgmt_protection = sta && sta->mfp,
		};

		if (!req.algorithm_id || key->key_len > sizeof(req.key_material))
			continue;

		/* The group key is installed for the AP, that is us in AP mode */
		if (group)
			ether_addr_copy(req.peer_addr, vif->bss_conf.bssid);
		else if (sta)
			ether_addr_copy(req.peer_addr, sta->addr);

		memcpy(req.key_material, key->key, key->key_len);
		mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_ADD_REMOVE_KEY,
				       &req, sizeof(req));
	}

	/* The key handshake with the AP is done */
	if (!group && !ap)
		mt7921_soc_abort_channel(dev, mvif->bss_conf.mt76.idx);

	return 0;
}

static void mt7921_soc_fill_sta(struct mt792x_dev *dev, struct mt7921_legacy_sta_rec *req,
				struct ieee80211_vif *vif, struct ieee80211_sta *sta,
				u8 state)
{
	struct mt792x_sta *msta = (struct mt792x_sta *)sta->drv_priv;
	struct ieee80211_link_sta *link = &sta->deflink;
	struct ieee80211_supported_band *sband = NULL;
	bool ap = vif->type == NL80211_IFTYPE_AP;
	u16 rates = 0, basic = 0;

	if (vif->bss_conf.chanreq.oper.chan)
		sband = mt76_hw(dev)->wiphy->bands[vif->bss_conf.chanreq.oper.chan->band];
	if (sband) {
		rates = mt7921_soc_rate_set(sband, link->supp_rates[sband->band]);
		basic = mt7921_soc_rate_set(sband, vif->bss_conf.basic_rates);
	}

	req->sta_idx = msta->deflink.wcid.idx;
	/* Clients of an access point belong to a P2P network */
	req->sta_type = ap ? LEGACY_STA_TYPE_P2P | LEGACY_STA_TYPE_CLIENT :
			     LEGACY_STA_TYPE_LEGACY | LEGACY_STA_TYPE_AP;
	ether_addr_copy(req->mac_addr, sta->addr);
	req->bss_idx = msta->vif->bss_conf.mt76.idx;
	req->desired_phy_type_set = mt7921_soc_phy_type_set(sta, sband ? sband->band :
							    NL80211_BAND_2GHZ, rates);
	req->desired_non_ht_rate_set = cpu_to_le16(rates);
	req->bss_basic_rate_set = cpu_to_le16(basic);
	req->is_qos = sta->wme;
	req->version = 1;
	req->wlan_idx = msta->deflink.wcid.idx;
	req->bmc_wlan_idx = 0xff;
	req->rts_policy = 3;
	req->hw_default_fixed_rate_code = cpu_to_le16(mt7921_soc_lowest_rate(basic));
	/* Block ack sessions are negotiated by the firmware */
	req->tx_ampdu = 1;
	req->rx_ampdu = 1;
	req->ba_size = cpu_to_le32(link->he_cap.has_he ? 0x01000100 : 0x4040);
	req->tx_amsdu_in_ampdu = 1;
	req->rx_amsdu_in_ampdu = 1;

	if (state == MT76_STA_INFO_STATE_ASSOC) {
		req->sta_state = LEGACY_STA_STATE_3;
		req->need_resp = 1;
		if (ap) {
			/* mac80211 does not tell the client's listen interval */
			req->aid = cpu_to_le16(sta->aid);
			req->listen_interval = cpu_to_le16(10);
		} else {
			req->aid = cpu_to_le16(vif->cfg.aid);
			req->listen_interval =
				cpu_to_le16(mt76_hw(dev)->conf.listen_interval);
		}
	}

	if (link->ht_cap.ht_supported) {
		req->mcs_set = link->ht_cap.mcs.rx_mask[0];
		req->sup_mcs32 = link->ht_cap.mcs.rx_mask[4] & 1;
		memcpy(req->rx_mcs_bitmask, link->ht_cap.mcs.rx_mask,
		       sizeof(req->rx_mcs_bitmask));
		req->rx_highest_supported_rate = link->ht_cap.mcs.rx_highest;
		req->tx_rate_info = cpu_to_le32(link->ht_cap.mcs.tx_params);
		req->ht_cap_info = cpu_to_le16(link->ht_cap.cap);
		req->ampdu_param = link->ht_cap.ampdu_factor |
				   (link->ht_cap.ampdu_density << 2);
		req->tx_max_amsdu_in_ampdu_len = cpu_to_le32(11454);
	}

	if (link->vht_cap.vht_supported) {
		req->vht_cap_info = cpu_to_le32(link->vht_cap.cap);
		req->vht_rx_mcs_map = link->vht_cap.vht_mcs.rx_mcs_map;
		req->vht_rx_highest_supported_data_rate = link->vht_cap.vht_mcs.rx_highest;
		req->vht_tx_mcs_map = link->vht_cap.vht_mcs.tx_mcs_map;
		req->vht_tx_highest_supported_data_rate = link->vht_cap.vht_mcs.tx_highest;
	}

	if (link->he_cap.has_he) {
		struct ieee80211_sta_he_cap *he = &link->he_cap;

		memcpy(req->he_mac_cap, he->he_cap_elem.mac_cap_info, sizeof(req->he_mac_cap));
		memcpy(req->he_phy_cap, he->he_cap_elem.phy_cap_info, sizeof(req->he_phy_cap));
		req->he_rx_mcs_map_bw80 = he->he_mcs_nss_supp.rx_mcs_80;
		req->he_tx_mcs_map_bw80 = he->he_mcs_nss_supp.tx_mcs_80;
		req->he_rx_mcs_map_bw160 = he->he_mcs_nss_supp.rx_mcs_160;
		req->he_tx_mcs_map_bw160 = he->he_mcs_nss_supp.tx_mcs_160;
		req->he_rx_mcs_map_bw80p80 = he->he_mcs_nss_supp.rx_mcs_80p80;
		req->he_tx_mcs_map_bw80p80 = he->he_mcs_nss_supp.tx_mcs_80p80;
		req->he_6g_band_cap = link->he_6ghz_capa.capa;
	}

	/* RCPI of the AP, from its last beacons; a client starts at -60 dBm */
	if (ap) {
		req->rcpi = 100;
		return;
	}
	req->rcpi = clamp((ewma_rssi_read(&msta->vif->bss_conf.rssi) ?
			   -(int)ewma_rssi_read(&msta->vif->bss_conf.rssi) : -60) + 110,
			  0, 110) * 2;
}

/* UNI STA_REC_UPDATE: UPDATE/REMOVE_STA_RECORD or ADD_REMOVE_KEY */
static int mt7921_soc_sta_rec(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct sta_req_hdr *hdr = (void *)skb->data;
	struct mt7921_legacy_sta_rec req = {};
	struct sta_rec_basic *basic = NULL;
	struct sta_rec_state *state = NULL;
	struct sta_rec_sec *sec = NULL;
	struct ieee80211_vif *vif;
	struct ieee80211_sta *sta;
	struct mt792x_vif *mvif;
	struct mt76_wcid *wcid;
	u16 wlan_idx;
	int ofs, ret = 0;

	if (skb->len < sizeof(*hdr))
		return -EINVAL;

	for (ofs = sizeof(*hdr); ofs + sizeof(struct tlv) <= skb->len;) {
		struct tlv *tlv = (struct tlv *)(skb->data + ofs);
		u16 len = le16_to_cpu(tlv->len);

		if (len < sizeof(*tlv) || ofs + len > skb->len)
			break;

		switch (le16_to_cpu(tlv->tag)) {
		case STA_REC_BASIC:
			basic = (void *)tlv;
			break;
		case STA_REC_STATE:
			state = (void *)tlv;
			break;
		case STA_REC_KEY_V2:
			sec = (void *)tlv;
			break;
		default:
			break;
		}
		ofs += len;
	}

	wlan_idx = hdr->wlan_idx_lo | (hdr->wlan_idx_hi << 8);

	rcu_read_lock();
	wcid = mt76_wcid_ptr(dev, wlan_idx);
	if (!wcid)
		goto out;

	if (wcid->sta) {
		struct mt792x_sta *msta = container_of(wcid, struct mt792x_sta, deflink.wcid);

		mvif = msta->vif;
	} else {
		mvif = container_of(wcid, struct mt792x_vif, sta.deflink.wcid);
	}
	vif = container_of((void *)mvif, struct ieee80211_vif, drv_priv);

	if (sec) {
		ret = mt7921_soc_key(dev, wcid, vif, sec);
		goto out;
	}

	/* The interface's own entry is handled by BSS_ACTIVATE_CTRL */
	sta = wcid_to_sta(wcid);
	if (!sta || !basic)
		goto out;

	if (basic->conn_state == CONN_STATE_DISCONNECT) {
		ret = mt7921_soc_remove_sta(dev, 0, wlan_idx, mvif->bss_conf.mt76.idx);
		goto out;
	}

	mt7921_soc_fill_sta(dev, &req, vif, sta,
			    state ? state->state : MT76_STA_INFO_STATE_NONE);
	ret = mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_UPDATE_STA_RECORD,
				     &req, sizeof(req));

	/* The firmware tracks the AP for power save once it is associated */
	if (!ret && req.sta_state == LEGACY_STA_STATE_3 && vif->type == NL80211_IFTYPE_STATION)
		ret = mt7921_soc_pm_bss(dev, vif, true);
out:
	rcu_read_unlock();

	return ret;
}

/*
 * Channel for starting an access point, called with the MCU mutex held. The
 * firmware keeps the channel of the access point once it is started.
 */
int mt7921_soc_ap_channel(struct mt792x_dev *dev, struct ieee80211_vif *vif)
{
	struct mt792x_vif *mvif = (struct mt792x_vif *)vif->drv_priv;
	struct mt7921_soc_dev *sdev = mt7921_soc_dev(dev);
	u8 idx = mvif->bss_conf.mt76.idx;
	struct mt7921_legacy_bss_rlm rlm = {};
	struct mt7921_legacy_ch_privilege req = {
		.bss_idx = idx,
		.token_id = ++dev->phy.roc_token_id,
		.action = LEGACY_CH_ACTION_REQ,
		.req_type = LEGACY_CH_REQ_GO_START_BSS,
		.dbdc_band = LEGACY_BAND_AUTO,
		.max_interval = cpu_to_le32(200),
	};
	int ret;

	mt7921_soc_fill_rlm(&rlm, vif, idx);
	req.primary_channel = rlm.primary_channel;
	req.sco = rlm.sco;
	req.band = rlm.band;
	req.channel_width = rlm.vht_channel_width;
	req.center_freq_s1 = rlm.vht_channel_freq_s1;
	req.center_freq_s2 = rlm.vht_channel_freq_s2;

	reinit_completion(&sdev->ap_grant);
	ret = mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_CH_PRIVILEGE, &req, sizeof(req));
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&sdev->ap_grant, HZ)) {
		dev_warn(dev->mt76.dev, "No channel for the access point\n");
		return -ETIMEDOUT;
	}

	/* Released once the access point is created */
	sdev->bss[idx].abort_token = req.token_id;
	sdev->bss[idx].abort_pending = true;

	return 0;
}

/* UNI ROC: CH_PRIVILEGE, for joining a BSS or remaining on a channel */
static int mt7921_soc_roc(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct tlv *tlv = (struct tlv *)(skb->data + 4);
	struct mt7921_legacy_ch_privilege req = {
		.dbdc_band = LEGACY_BAND_AUTO,
	};

	if (skb->len < 4 + sizeof(*tlv))
		return -EINVAL;

	if (le16_to_cpu(tlv->tag) == UNI_ROC_ACQUIRE) {
		struct {
			__le16 tag;
			__le16 len;
			u8 bss_idx;
			u8 tokenid;
			u8 control_channel;
			u8 sco;
			u8 band;
			u8 bw;
			u8 center_chan;
			u8 center_chan2;
			u8 bw_from_ap;
			u8 center_chan_from_ap;
			u8 center_chan2_from_ap;
			u8 reqtype;
			__le32 maxinterval;
		} __packed *roc = (void *)tlv;

		req.bss_idx = roc->bss_idx;
		req.token_id = roc->tokenid;
		req.action = LEGACY_CH_ACTION_REQ;
		req.primary_channel = roc->control_channel;
		req.sco = roc->sco;
		req.band = roc->band;
		req.req_type = roc->reqtype;
		req.max_interval = roc->maxinterval;
		/* Long enough for the whole join, including the key handshake */
		if (roc->reqtype == MT7921_ROC_REQ_JOIN)
			req.max_interval = cpu_to_le32(max_t(u32, 4000,
							     le32_to_cpu(roc->maxinterval)));
	} else {
		struct {
			__le16 tag;
			__le16 len;
			u8 bss_idx;
			u8 tokenid;
		} __packed *abort = (void *)tlv;
		struct mt7921_soc_bss *sbss;
		struct ieee80211_vif *vif;

		req.bss_idx = abort->bss_idx;
		req.token_id = abort->tokenid;
		req.action = LEGACY_CH_ACTION_ABORT;

		/*
		 * The firmware holds data frames without the channel: once
		 * associated to a protected BSS keep it until the key
		 * handshake is done. The grant does not expire, every later
		 * request waits for the abort.
		 */
		if (abort->bss_idx >= ARRAY_SIZE(mt7921_soc_dev(dev)->bss))
			return -EINVAL;

		sbss = &mt7921_soc_dev(dev)->bss[abort->bss_idx];
		vif = mt7921_soc_find_vif(dev, abort->bss_idx);
		if (sbss->connected && vif && vif->bss_conf.bss &&
		    (vif->bss_conf.bss->capability & WLAN_CAPABILITY_PRIVACY)) {
			sbss->abort_token = abort->tokenid;
			sbss->abort_pending = true;
			return 0;
		}
	}

	return mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_CH_PRIVILEGE, &req, sizeof(req));
}

/* Answer to a unified command the firmware did not get */
static void mt7921_soc_fake_response(struct mt792x_dev *dev, int cmd, int seq)
{
	struct mt76_connac2_mcu_rxd *rxd;
	struct mt76_connac_mcu_uni_event *event;
	struct sk_buff *skb;

	skb = alloc_skb(sizeof(*rxd) + sizeof(*event), GFP_KERNEL);
	if (!skb)
		return;

	rxd = skb_put_zero(skb, sizeof(*rxd));
	rxd->seq = seq;
	rxd->eid = MCU_UNI_EVENT_RESULT;
	event = skb_put_zero(skb, sizeof(*event));
	event->cid = FIELD_GET(__MCU_CMD_FIELD_ID, cmd);

	mt76_mcu_rx_event(&dev->mt76, skb);
}

/*
 * Handle a command the firmware does not know. Returns true if the command
 * has been consumed, @skb is freed then.
 */
bool mt7921_soc_legacy_cmd(struct mt792x_dev *dev, struct sk_buff *skb, int cmd,
			   int *wait_seq, int *ret)
{
	struct mt76_dev *mdev = &dev->mt76;
	int seq;

	*ret = 0;

	if (cmd == MCU_CE_CMD(SET_RX_FILTER)) {
		/* Legacy filter: broadcast and all multicast frames */
		__le32 filter = cpu_to_le32(0x0c);

		skb_trim(skb, 0);
		skb_put_data(skb, &filter, sizeof(filter));
		skb_put_zero(skb, 64);
		return false;
	}

	if (cmd == MCU_CE_CMD(START_HW_SCAN)) {
		struct mt76_connac_hw_scan_req *req = (void *)skb->data;

		/* No split scan, the whole scan is done at once */
		req->scan_func &= ~SCAN_FUNC_SPLIT_SCAN;
		/* A broadcast BSSID filter makes the firmware ignore everything */
		if (is_broadcast_ether_addr(req->bssid))
			eth_zero_addr(req->bssid);
		/* Fields after the random address are laid out differently */
		memset(req->pad, 0, skb->len - offsetof(struct mt76_connac_hw_scan_req, pad));
		if (skb->len < 1192)
			skb_put_zero(skb, 1192 - skb->len);
		return false;
	}

	if (cmd == MCU_CE_CMD(SET_PS_PROFILE) && skb->len < 4) {
		skb_put_zero(skb, 4 - skb->len);
		return false;
	}

	/* An access point is created, not connected */
	if (cmd == MCU_CE_CMD(SET_BSS_CONNECTED) && skb->len >= 6 &&
	    skb->data[0] < ARRAY_SIZE(mt7921_soc_dev(dev)->bss) &&
	    mt7921_soc_dev(dev)->bss[skb->data[0]].ap) {
		struct mt7921_legacy_pm_bss_created req = {
			.bss_idx = skb->data[0],
			.dtim_period = skb->data[1] ?: 1,
			.beacon_interval = cpu_to_le16(get_unaligned_le16(skb->data + 4)),
		};

		mt7921_soc_legacy_send(dev, MCU_CE_CMD_LEGACY_PM_BSS_CREATED,
				       &req, sizeof(req));
		mt7921_soc_abort_channel(dev, req.bss_idx);
		goto consume;
	}

	/* Channel and RX path are managed by the firmware through CH_PRIVILEGE */
	if (cmd == MCU_EXT_CMD(CHANNEL_SWITCH) || cmd == MCU_EXT_CMD(SET_RX_PATH))
		goto consume;

	if (!(cmd & __MCU_CMD_FIELD_UNI))
		return false;

	if (cmd == MCU_UNI_CMD(DEV_INFO_UPDATE))
		*ret = mt7921_soc_dev_info(dev, skb);
	else if (cmd == MCU_UNI_CMD(BSS_INFO_UPDATE))
		*ret = mt7921_soc_bss_info(dev, skb);
	else if (cmd == MCU_UNI_CMD(STA_REC_UPDATE))
		*ret = mt7921_soc_sta_rec(dev, skb);
	else if (cmd == MCU_UNI_CMD(ROC))
		*ret = mt7921_soc_roc(dev, skb);
	else
		/* Offloads, suspend and sniffer settings */
		dev_dbg(mdev->dev, "Skipping unified command %08x\n", cmd);

consume:
	/* Consume a sequence number for a synthesized answer */
	mt76_connac2_mcu_fill_message(mdev, skb, cmd, &seq);
	dev_kfree_skb(skb);
	if (wait_seq) {
		*wait_seq = seq;
		mt7921_soc_fake_response(dev, cmd, seq);
	}

	return true;
}

/* Legacy CH_PRIVILEGE event, handed to mt7921 as unified ROC grant */
static void mt7921_soc_ch_privilege_event(struct mt76_dev *mdev, enum mt76_rxq_id q,
					  struct sk_buff *skb, u32 *info)
{
	struct mt76_connac2_mcu_rxd *rxd = (void *)skb->data;
	struct mt7921_roc_grant_tlv *grant;
	struct sk_buff *uni;
	u8 *data;

	if (skb->len < sizeof(*rxd) + 16)
		goto out;

	data = rxd->tlv;
	/* Only grants are of interest */
	if (data[2])
		goto out;

	/* Requested by mt7921_soc_ap_channel() */
	if (data[9] == LEGACY_CH_REQ_GO_START_BSS) {
		struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);

		complete(&mt7921_soc_dev(dev)->ap_grant);
		goto out;
	}

	uni = alloc_skb(sizeof(*rxd) + 4 + sizeof(*grant), GFP_ATOMIC);
	if (!uni)
		goto out;

	skb_put_data(uni, rxd, sizeof(*rxd));
	rxd = (void *)uni->data;
	rxd->eid = MCU_UNI_EVENT_ROC;
	rxd->option |= MCU_UNI_CMD_UNSOLICITED_EVENT;
	rxd->seq = 0;
	skb_put_zero(uni, 4);
	grant = skb_put_zero(uni, sizeof(*grant));
	grant->tag = cpu_to_le16(UNI_EVENT_ROC_GRANT);
	grant->len = cpu_to_le16(sizeof(*grant));
	memcpy(&grant->bss_idx, data, 12);
	memcpy(&grant->max_interval, data + 12, 4);

	mt7921_queue_rx_skb(mdev, q, uni, info);
out:
	dev_kfree_skb(skb);
}

/* Block ack sessions set up by the firmware */
static void mt7921_soc_ba_event(struct mt76_dev *mdev, struct sk_buff *skb, bool start)
{
	struct mt76_connac2_mcu_rxd *rxd = (void *)skb->data;
	struct mt76_wcid *wcid;
	u8 *data = rxd->tlv;

	if (skb->len < sizeof(*rxd) + 8)
		return;

	rcu_read_lock();
	wcid = __mt76_wcid_ptr(mdev, data[0]);
	if (wcid && wcid->sta) {
		if (start) {
			u16 param = get_unaligned_le16(data + 2);
			u16 ssc = get_unaligned_le16(data + 6);

			mt76_rx_aggr_start(mdev, wcid, (param >> 2) & 0xf, ssc >> 4,
					   param >> 6);
		} else {
			mt76_rx_aggr_stop(mdev, wcid, data[1]);
		}
	}
	rcu_read_unlock();
}

/* Returns true if the event has been consumed */
bool mt7921_soc_legacy_event(struct mt76_dev *mdev, enum mt76_rxq_id q,
			     struct sk_buff *skb, u32 *info)
{
	struct mt76_connac2_mcu_rxd *rxd = (void *)skb->data;

	if (rxd->option & MCU_UNI_CMD_UNSOLICITED_EVENT)
		return false;

	switch (rxd->eid) {
	case MCU_EVENT_LEGACY_CH_PRIVILEGE:
		mt7921_soc_ch_privilege_event(mdev, q, skb, info);
		return true;
	case MCU_EVENT_LEGACY_RX_ADDBA:
		mt7921_soc_ba_event(mdev, skb, true);
		break;
	case MCU_EVENT_LEGACY_RX_DELBA:
		mt7921_soc_ba_event(mdev, skb, false);
		break;
	/* Answers to legacy commands nobody waits for */
	case MCU_EVENT_LEGACY_ACTIVATE_STA_REC:
	case MCU_EVENT_LEGACY_ADD_PKEY_DONE:
	case MCU_EVENT_LEGACY_CMD_RESULT:
	case MCU_EVENT_LEGACY_CHIP_CONFIG:
		break;
	default:
		return false;
	}

	dev_kfree_skb(skb);
	return true;
}
