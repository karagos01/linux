/* SPDX-License-Identifier: BSD-3-Clause-Clear */

#ifndef __MT7921_SOC_H
#define __MT7921_SOC_H

#include <linux/firmware.h>
#include <linux/soc/mediatek/mtk-connsys.h>

#include "mt7921.h"

/* State of a BSS as told to the legacy firmware */
struct mt7921_soc_bss {
	bool active;
	bool connected;
	/* An access point, a P2P network for the firmware */
	bool ap;
	bool privacy;
	/* Channel request to abort once the BSS does not need it any more */
	bool abort_pending;
	u8 abort_token;
};

struct mt7921_soc_dev {
	struct mt792x_dev dev;
	struct mtk_connsys *cs;
	const struct firmware *nvram;
	u8 *nvram_rf;
	int nvram_rf_len;
	struct mt7921_soc_bss bss[MT792x_MAX_INTERFACES];
	/* The interface being added is an access point, DEV_INFO has no type */
	bool adding_ap;
	struct completion ap_grant;
};

static inline struct mt7921_soc_dev *mt7921_soc_dev(struct mt792x_dev *dev)
{
	return container_of(dev, struct mt7921_soc_dev, dev);
}

int mt7921_soc_legacy_send(struct mt792x_dev *dev, int id, const void *data, int len);
int mt7921_soc_ap_channel(struct mt792x_dev *dev, struct ieee80211_vif *vif);
bool mt7921_soc_legacy_cmd(struct mt792x_dev *dev, struct sk_buff *skb, int cmd,
			   int *wait_seq, int *ret);
bool mt7921_soc_legacy_event(struct mt76_dev *mdev, enum mt76_rxq_id q,
			     struct sk_buff *skb, u32 *info);

#endif /* __MT7921_SOC_H */
