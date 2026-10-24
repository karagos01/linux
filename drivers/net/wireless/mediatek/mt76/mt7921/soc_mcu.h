/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Legacy (pre unified command) firmware interface of the MT7921 compatible
 * WiFi integrated in MediaTek SoCs.
 */

#ifndef __MT7921_SOC_MCU_H
#define __MT7921_SOC_MCU_H

#include "../mt76_connac_mcu.h"

/* Legacy command IDs, sent as MCU_CE_CMD() */
enum {
	MCU_CE_CMD_LEGACY_ADD_REMOVE_KEY = 0x07,
	MCU_CE_CMD_LEGACY_INFRASTRUCTURE = 0x09,
	MCU_CE_CMD_LEGACY_BSS_ACTIVATE_CTRL = 0x11,
	MCU_CE_CMD_LEGACY_SET_BSS_INFO = 0x12,
	MCU_CE_CMD_LEGACY_UPDATE_STA_RECORD = 0x13,
	MCU_CE_CMD_LEGACY_REMOVE_STA_RECORD = 0x14,
	MCU_CE_CMD_LEGACY_PM_BSS_CREATED = 0x15,
	MCU_CE_CMD_LEGACY_PM_BSS_CONNECTED = 0x16,
	MCU_CE_CMD_LEGACY_PM_BSS_ABORT = 0x17,
	MCU_CE_CMD_LEGACY_UPDATE_BEACON = 0x18,
	MCU_CE_CMD_LEGACY_CH_PRIVILEGE = 0x1c,
};

/* Legacy event IDs */
enum {
	MCU_EVENT_LEGACY_ACTIVATE_STA_REC = 0x0c,
	MCU_EVENT_LEGACY_CH_PRIVILEGE = 0x10,
	MCU_EVENT_LEGACY_RX_ADDBA = 0x0a,
	MCU_EVENT_LEGACY_RX_DELBA = 0x0b,
	MCU_EVENT_LEGACY_ADD_PKEY_DONE = 0x24,
	MCU_EVENT_LEGACY_CHIP_CONFIG = 0x44,
	MCU_EVENT_LEGACY_CMD_RESULT = 0xfd,
};

#define LEGACY_NETWORK_TYPE_AIS		0
#define LEGACY_NETWORK_TYPE_P2P		1

#define LEGACY_OP_MODE_AP		2

#define LEGACY_STA_TYPE_LEGACY		BIT(0)
#define LEGACY_STA_TYPE_P2P		BIT(1)
#define LEGACY_STA_TYPE_CLIENT		BIT(5)
#define LEGACY_STA_TYPE_AP		BIT(6)

#define LEGACY_STA_STATE_1		0
#define LEGACY_STA_STATE_2		1
#define LEGACY_STA_STATE_3		2

#define LEGACY_MEDIA_STATE_CONNECTED	0
#define LEGACY_MEDIA_STATE_DISCONNECTED	1

#define LEGACY_PHY_TYPE_HR_DSSS		BIT(0)
#define LEGACY_PHY_TYPE_ERP		BIT(1)
#define LEGACY_PHY_TYPE_OFDM		BIT(2)
#define LEGACY_PHY_TYPE_HT		BIT(4)
#define LEGACY_PHY_TYPE_VHT		BIT(5)
#define LEGACY_PHY_TYPE_HE		BIT(6)

#define LEGACY_BAND_2G4			1
#define LEGACY_BAND_5G			2
#define LEGACY_BAND_6G			3
#define LEGACY_BAND_AUTO		4

#define LEGACY_CH_ACTION_REQ		0
#define LEGACY_CH_ACTION_ABORT		1

#define LEGACY_CH_REQ_GO_START_BSS	3

#define LEGACY_BEACON_UPDATE_ALL	1
#define LEGACY_BEACON_MAX_IE_LEN	600

struct mt7921_legacy_bss_activate {
	u8 bss_idx;
	u8 active;
	u8 network_type;
	u8 omac_idx;
	u8 mac_addr[ETH_ALEN];
	u8 bmc_wlan_idx;
	u8 mld_link_idx;
} __packed;

struct mt7921_legacy_bss_rlm {
	u8 bss_idx;
	u8 band;
	u8 primary_channel;
	u8 sco;
	u8 erp_protect_mode;
	u8 ht_protect_mode;
	u8 gf_operation_mode;
	u8 tx_rifs_mode;
	__le16 ht_op_info3;
	__le16 ht_op_info2;
	u8 ht_op_info1;
	u8 use_short_preamble;
	u8 use_short_slot_time;
	u8 vht_channel_width;
	u8 vht_channel_freq_s1;
	u8 vht_channel_freq_s2;
	__le16 vht_basic_mcs_set;
	u8 tx_nss;
	u8 rx_nss;
} __packed;

struct mt7921_legacy_bss_info {
	u8 bss_idx;
	u8 connection_state;
	u8 current_op_mode;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	u8 bssid[ETH_ALEN];
	u8 is_qbss;
	u8 version;
	__le16 operational_rate_set;
	__le16 bss_basic_rate_set;
	u8 sta_rec_idx_of_ap;
	u8 pad0;
	__le16 hw_default_fixed_rate_code;
	u8 non_ht_basic_phy_type;
	u8 auth_mode;
	u8 enc_status;
	u8 phy_type_set;
	u8 wapi_mode;
	u8 is_ap_mode;
	u8 bmc_wlan_idx;
	u8 hidden_ssid_mode;
	u8 disconnect_detect_threshold;
	u8 iot_ap_act;
	u8 pad1[2];
	__le32 private_data;
	struct mt7921_legacy_bss_rlm rlm;
	u8 dbdc_band;
	u8 wmm_set;
	u8 dbdc_action;
	u8 nss;
	u8 pad2[2];
	u8 he_op_params[3];
	u8 bss_color_info;
	__le16 he_basic_mcs_set;
	u8 max_bssid_indicator;
	u8 mbssid_index;
	u8 pad[12];
} __packed;

struct mt7921_legacy_sta_rec {
	u8 sta_idx;
	u8 sta_type;
	u8 mac_addr[ETH_ALEN];
	__le16 aid;
	__le16 listen_interval;
	u8 bss_idx;
	u8 desired_phy_type_set;
	__le16 desired_non_ht_rate_set;
	__le16 bss_basic_rate_set;
	u8 is_qos;
	u8 is_uapsd_supported;
	u8 sta_state;
	u8 mcs_set;
	u8 sup_mcs32;
	u8 version;
	u8 rx_mcs_bitmask[10];
	__le16 rx_highest_supported_rate;
	__le32 tx_rate_info;
	__le16 ht_cap_info;
	__le16 ht_extended_cap;
	__le32 tx_beamforming_cap;
	u8 ampdu_param;
	u8 asel_cap;
	u8 rcpi;
	u8 need_resp;
	u8 uapsd_ac;
	u8 uapsd_sp;
	u8 wlan_idx;
	u8 bmc_wlan_idx;
	__le32 vht_cap_info;
	__le16 vht_rx_mcs_map;
	__le16 vht_rx_highest_supported_data_rate;
	__le16 vht_tx_mcs_map;
	__le16 vht_tx_highest_supported_data_rate;
	u8 rts_policy;
	u8 vht_op_mode;
	u8 traffic_data_type;
	u8 tx_gf_mode;
	u8 tx_sgi_mode;
	u8 tx_stbc_mode;
	__le16 hw_default_fixed_rate_code;
	u8 tx_ampdu;
	u8 rx_ampdu;
	u8 pad0[2];
	__le32 fixed_phy_rate;
	__le16 max_link_speed;
	__le16 min_link_speed;
	__le32 flags;
	__le32 ba_size;
	__le16 pfmu_id;
	u8 su_mu;
	u8 etxbf_cap;
	u8 sounding_phy;
	u8 ndpa_rate;
	u8 ndp_rate;
	u8 rept_poll_rate;
	u8 tx_mode;
	u8 nc;
	u8 nr;
	u8 cbw;
	u8 mem[10];
	__le16 smart_ant;
	u8 se_idx;
	u8 ibf_timeout;
	u8 ibf_dbw;
	u8 ibf_ncol;
	u8 ibf_nrow;
	u8 pad1;
	u8 mlr_mode;
	u8 mlr_state;
	u8 tx_amsdu_in_ampdu;
	u8 rx_amsdu_in_ampdu;
	u8 pad2[2];
	__le32 tx_max_amsdu_in_ampdu_len;
	u8 he_mac_cap[6];
	u8 he_phy_cap[11];
	u8 pad3[3];
	__le16 he_rx_mcs_map_bw80;
	__le16 he_tx_mcs_map_bw80;
	__le16 he_rx_mcs_map_bw160;
	__le16 he_tx_mcs_map_bw160;
	__le16 he_rx_mcs_map_bw80p80;
	__le16 he_tx_mcs_map_bw80p80;
	__le16 he_6g_band_cap;
	u8 pad4[30];
	u8 rxsmm_enable;
	u8 pad5[3];
} __packed;

struct mt7921_legacy_remove_sta {
	u8 action;
	u8 sta_idx;
	u8 bss_idx;
	u8 rsv;
} __packed;

struct mt7921_legacy_pm_bss_connected {
	u8 bss_idx;
	u8 dtim_period;
	__le16 aid;
	__le16 beacon_interval;
	__le16 atim_window;
	u8 is_uapsd;
	u8 bmp_delivery_ac;
	u8 bmp_trigger_ac;
	u8 rsv;
} __packed;

struct mt7921_legacy_pm_bss_created {
	u8 bss_idx;
	u8 dtim_period;
	__le16 beacon_interval;
	__le16 atim_window;
	u8 rsv[2];
} __packed;

struct mt7921_legacy_pm_bss_abort {
	u8 bss_idx;
	u8 rsv[3];
} __packed;

struct mt7921_legacy_ch_privilege {
	u8 bss_idx;
	u8 token_id;
	u8 action;
	u8 primary_channel;
	u8 sco;
	u8 band;
	u8 channel_width;
	u8 center_freq_s1;
	u8 center_freq_s2;
	u8 req_type;
	u8 dbdc_band;
	u8 rsv;
	__le32 max_interval;
	u8 rsv2[8];
} __packed;

struct mt7921_legacy_key {
	u8 add_remove;
	u8 tx_key;
	u8 key_type;
	u8 is_authenticator;
	u8 peer_addr[ETH_ALEN];
	u8 bss_idx;
	u8 algorithm_id;
	u8 key_id;
	u8 key_len;
	u8 wlan_idx;
	u8 mgmt_protection;
	u8 key_material[32];
	u8 key_rsc[16];
} __packed;

struct mt7921_legacy_beacon {
	u8 update_method;
	u8 bss_idx;
	u8 rsv[2];
	__le16 capability;
	__le16 ie_len;
	u8 ie[LEGACY_BEACON_MAX_IE_LEN];
} __packed;

#endif /* __MT7921_SOC_MCU_H */
