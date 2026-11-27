// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel / Lantiq WAV500 (WAV514/WAV524, gen5b) clean-room cfg80211 layer.
 *
 * Registers 2.4 GHz (phy0 / wlan0) and 5 GHz (phy1 / wlan1) FullMAC wiphys
 * and wires cfg80211_ops to the WAV500 UMI management IPC.
 */

#include <linux/etherdevice.h>
#include <linux/ieee80211.h>
#include <linux/netdevice.h>
#include <net/cfg80211.h>

#include "wav500.h"

#define CHAN2G(_ch, _freq) { \
	.band = NL80211_BAND_2GHZ, \
	.center_freq = (_freq), \
	.hw_value = (_ch), \
	.max_power = 20, \
}

#define CHAN5G(_ch, _freq) { \
	.band = NL80211_BAND_5GHZ, \
	.center_freq = (_freq), \
	.hw_value = (_ch), \
	.max_power = 23, \
}

static struct ieee80211_channel wav500_2ghz_channels[] = {
	CHAN2G(1, 2412),  CHAN2G(2, 2417),  CHAN2G(3, 2422),  CHAN2G(4, 2427),
	CHAN2G(5, 2432),  CHAN2G(6, 2437),  CHAN2G(7, 2442),  CHAN2G(8, 2447),
	CHAN2G(9, 2452),  CHAN2G(10, 2457), CHAN2G(11, 2462), CHAN2G(12, 2467),
	CHAN2G(13, 2472),
};

static struct ieee80211_channel wav500_5ghz_channels[] = {
	CHAN5G(36, 5180),  CHAN5G(40, 5200),  CHAN5G(44, 5220),  CHAN5G(48, 5240),
	CHAN5G(52, 5260),  CHAN5G(56, 5280),  CHAN5G(60, 5300),  CHAN5G(64, 5320),
	CHAN5G(100, 5500), CHAN5G(104, 5520), CHAN5G(108, 5540), CHAN5G(112, 5560),
	CHAN5G(116, 5580), CHAN5G(120, 5600), CHAN5G(124, 5620), CHAN5G(128, 5640),
	CHAN5G(132, 5660), CHAN5G(136, 5680), CHAN5G(140, 5700), CHAN5G(144, 5720),
	CHAN5G(149, 5745), CHAN5G(153, 5765), CHAN5G(157, 5785), CHAN5G(161, 5805),
	CHAN5G(165, 5825),
};

static struct ieee80211_rate wav500_2ghz_rates[] = {
	{ .bitrate = 10,  .hw_value = 0 },
	{ .bitrate = 20,  .hw_value = 1, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55,  .hw_value = 2, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .hw_value = 3, .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60,  .hw_value = 4 },
	{ .bitrate = 90,  .hw_value = 5 },
	{ .bitrate = 120, .hw_value = 6 },
	{ .bitrate = 180, .hw_value = 7 },
	{ .bitrate = 240, .hw_value = 8 },
	{ .bitrate = 360, .hw_value = 9 },
	{ .bitrate = 480, .hw_value = 10 },
	{ .bitrate = 540, .hw_value = 11 },
};

static struct ieee80211_rate wav500_5ghz_rates[] = {
	{ .bitrate = 60,  .hw_value = 4 },
	{ .bitrate = 90,  .hw_value = 5 },
	{ .bitrate = 120, .hw_value = 6 },
	{ .bitrate = 180, .hw_value = 7 },
	{ .bitrate = 240, .hw_value = 8 },
	{ .bitrate = 360, .hw_value = 9 },
	{ .bitrate = 480, .hw_value = 10 },
	{ .bitrate = 540, .hw_value = 11 },
};

static struct ieee80211_supported_band wav500_band_2ghz = {
	.band = NL80211_BAND_2GHZ,
	.channels = wav500_2ghz_channels,
	.n_channels = ARRAY_SIZE(wav500_2ghz_channels),
	.bitrates = wav500_2ghz_rates,
	.n_bitrates = ARRAY_SIZE(wav500_2ghz_rates),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_SM_PS |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_TX_STBC |
		       (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT),
		.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K,
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_4,
		.mcs = {
			.rx_mask = { 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0, 0 },
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};

static struct ieee80211_supported_band wav500_band_5ghz = {
	.band = NL80211_BAND_5GHZ,
	.channels = wav500_5ghz_channels,
	.n_channels = ARRAY_SIZE(wav500_5ghz_channels),
	.bitrates = wav500_5ghz_rates,
	.n_bitrates = ARRAY_SIZE(wav500_5ghz_rates),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_SM_PS |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_TX_STBC |
		       (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT),
		.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K,
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_4,
		.mcs = {
			.rx_mask = { 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0, 0 },
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
	.vht_cap = {
		.vht_supported = true,
		.cap = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454 |
		       IEEE80211_VHT_CAP_SHORT_GI_80 |
		       IEEE80211_VHT_CAP_TXSTBC |
		       IEEE80211_VHT_CAP_RXSTBC_1 |
		       IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK,
		.vht_mcs = {
			.rx_mcs_map = cpu_to_le16(0xffaa),
			.tx_mcs_map = cpu_to_le16(0xffaa),
		},
	},
};

static const u32 wav500_cipher_suites[] = {
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
};

static int wav500_ndo_open(struct net_device *ndev)
{
	netif_start_queue(ndev);
	return 0;
}

static int wav500_ndo_stop(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	return 0;
}

static netdev_tx_t wav500_ndo_start_xmit(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct wav500_vif *vif = netdev_priv(ndev);
	struct wav500_priv *priv = vif->priv;
	int ret;

	if (!priv->dp_inited || !vif->ap_started) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	ret = wav500_dma_tx_data(priv, vif->vap_id, skb);
	if (ret) {
		ndev->stats.tx_dropped++;
	} else {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops wav500_netdev_ops = {
	.ndo_open = wav500_ndo_open,
	.ndo_stop = wav500_ndo_stop,
	.ndo_start_xmit = wav500_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

static u8 wav500_nl_width_to_umi(enum nl80211_chan_width w)
{
	switch (w) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		return 0;
	case NL80211_CHAN_WIDTH_40:
		return 1;
	case NL80211_CHAN_WIDTH_80:
		return 2;
	default:
		return 0;
	}
}

static int wav500_cfg_change_virtual_intf(struct wiphy *wiphy,
					  struct net_device *dev,
					  enum nl80211_iftype type,
					  struct vif_params *params)
{
	struct wav500_vif *vif = netdev_priv(dev);

	if (type != NL80211_IFTYPE_AP && type != NL80211_IFTYPE_STATION)
		return -EOPNOTSUPP;

	vif->wdev.iftype = type;
	return 0;
}

static int wav500_cfg_start_ap(struct wiphy *wiphy, struct net_device *dev,
			       struct cfg80211_ap_settings *settings)
{
	struct wav500_vif *vif = netdev_priv(dev);
	struct wav500_priv *priv = vif->priv;
	u32 freq = priv->is_5ghz ? 5180 : 2437;
	u32 center_freq = freq;
	u8 width = 0;
	u8 ch;
	int ret;

	if (!priv->fw_ready) {
		ret = wav500_boot_fw(priv);
		if (ret)
			return ret;
	}

	if (settings->chandef.chan) {
		freq = settings->chandef.chan->center_freq;
		center_freq = settings->chandef.center_freq1 ?: freq;
		width = wav500_nl_width_to_umi(settings->chandef.width);
	}
	ch = (u8)ieee80211_frequency_to_channel(freq);

	ret = wav500_umi_pre_activate(priv);
	if (ret)
		return ret;
	ret = wav500_umi_hdk_config(priv, 0, 0x000003ff, 0x000000ff);
	if (ret)
		return ret;

	if (!priv->vap_added) {
		ret = wav500_umi_vap_db_op(priv, vif->vap_id, true);
		if (ret)
			return ret;
	}

	ret = wav500_umi_activate_ap(priv, vif->vap_id, dev->dev_addr,
				     settings->ssid, settings->ssid_len,
				     settings->beacon_interval ?: 100);
	if (ret)
		return ret;

	ret = wav500_umi_set_radio_mode(priv, true);
	if (ret)
		return ret;
	msleep(200);

	ret = wav500_umi_calibrate(priv, ch, width);
	if (ret)
		return ret;

	wav500_umi_set_antennas(priv, 0x0f, 0x0f);
	wav500_umi_set_tx_power_offset(priv, 0);
	wav500_umi_set_tpc_config(priv);
	wav500_umi_set_tpc_gen5(priv, ch);

	ret = wav500_umi_set_chan(priv, freq, width, center_freq);
	if (ret)
		return ret;

	if (settings->beacon.head && settings->beacon.head_len) {
		ret = wav500_umi_set_beacon(priv, vif->vap_id,
					    settings->beacon.head,
					    settings->beacon.head_len,
					    settings->beacon.tail,
					    settings->beacon.tail_len);
		if (ret)
			return ret;
	}

	wav500_umi_set_wmm_default(priv, vif->vap_id);
	ret = wav500_umi_set_bss(priv, vif->vap_id);
	if (ret)
		return ret;

	wav500_dma_enable_irqs(priv);
	vif->ap_started = true;
	netif_carrier_on(dev);
	return 0;
}

static int wav500_cfg_change_beacon(struct wiphy *wiphy, struct net_device *dev,
				    struct cfg80211_ap_update *info)
{
	struct wav500_vif *vif = netdev_priv(dev);

	if (info->beacon.head && info->beacon.head_len)
		return wav500_umi_set_beacon(vif->priv, vif->vap_id,
					     info->beacon.head,
					     info->beacon.head_len,
					     info->beacon.tail,
					     info->beacon.tail_len);
	return 0;
}

static int wav500_cfg_stop_ap(struct wiphy *wiphy, struct net_device *dev,
			      unsigned int link_id)
{
	struct wav500_vif *vif = netdev_priv(dev);

	vif->ap_started = false;
	netif_carrier_off(dev);
	return 0;
}

static int wav500_cfg_add_key(struct wiphy *wiphy, struct net_device *netdev,
			      int link_id, u8 key_index, bool pairwise,
			      const u8 *mac_addr, struct key_params *params)
{
	struct wav500_vif *vif = netdev_priv(netdev);
	struct wav500_priv *priv = vif->priv;
	u16 sid = 0x3ffa;

	if (!priv->fw_ready)
		return -ENODEV;
	if (!priv->vap_added)
		return 0;
	if (pairwise) {
		if (!mac_addr || wav500_sta_find_sid(priv, mac_addr, &sid))
			return -ENOENT;
	}
	return wav500_umi_set_key(priv, vif->vap_id, sid, key_index,
				  pairwise, params);
}

static int wav500_cfg_del_key(struct wiphy *wiphy, struct net_device *netdev,
			      int link_id, u8 key_index, bool pairwise,
			      const u8 *mac_addr)
{
	return 0;
}

static int wav500_cfg_set_default_key(struct wiphy *wiphy,
				      struct net_device *netdev, int link_id,
				      u8 key_index, bool unicast,
				      bool multicast)
{
	struct wav500_vif *vif = netdev_priv(netdev);
	struct wav500_priv *priv = vif->priv;

	if (!priv->fw_ready || !priv->vap_added)
		return 0;
	return wav500_umi_set_default_key(priv, vif->vap_id, 0x3ffa,
					  multicast || !unicast, key_index);
}

static int wav500_cfg_add_station(struct wiphy *wiphy, struct net_device *dev,
				  const u8 *mac,
				  struct station_parameters *params)
{
	struct wav500_vif *vif = netdev_priv(dev);
	struct wav500_priv *priv = vif->priv;
	u16 sid = 0;
	int ret;

	if (!priv->fw_ready)
		return -ENODEV;

	if (wav500_sta_find_sid(priv, mac, &sid)) {
		ret = wav500_umi_request_sid(priv, mac, &sid);
		if (ret)
			return ret;
		ret = wav500_sta_register(priv, mac, sid, vif->vap_id);
		if (ret) {
			wav500_umi_remove_sid(priv, sid);
			return ret;
		}
	}

	return wav500_umi_add_sta(priv, vif->vap_id, sid, mac, params);
}

static int wav500_cfg_change_station(struct wiphy *wiphy, struct net_device *dev,
				     const u8 *mac,
				     struct station_parameters *params)
{
	/* Station was already added to FW in add_station; do not re-send STA_ADD */
	return 0;
}

static int wav500_cfg_del_station(struct wiphy *wiphy, struct net_device *dev,
				  struct station_del_parameters *params)
{
	struct wav500_vif *vif = netdev_priv(dev);
	struct wav500_priv *priv = vif->priv;
	u16 sid = 0;

	if (!priv->fw_ready || !params->mac)
		return 0;
	if (!wav500_sta_find_sid(priv, params->mac, &sid)) {
		wav500_umi_remove_sta(priv, sid);
		wav500_umi_remove_sid(priv, sid);
		wav500_sta_unregister(priv, params->mac);
	}
	return 0;
}

static int wav500_cfg_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
			      struct cfg80211_mgmt_tx_params *params,
			      u64 *cookie)
{
	struct wav500_vif *vif = container_of(wdev, struct wav500_vif, wdev);
	struct wav500_priv *priv = vif->priv;
	u64 c;

	if (!priv->dp_inited || !params->buf || !params->len)
		return -ENODEV;

	c = ++priv->mgmt_cookie;
	if (!c)
		c = ++priv->mgmt_cookie;
	*cookie = c;

	return wav500_dma_tx_mgmt(priv, vif->vap_id, params->buf, params->len, c);
}

static int wav500_cfg_connect(struct wiphy *wiphy, struct net_device *dev,
			      struct cfg80211_connect_params *sme)
{
	return -EOPNOTSUPP;
}

static int wav500_cfg_disconnect(struct wiphy *wiphy, struct net_device *dev,
				 u16 reason_code)
{
	return -EOPNOTSUPP;
}

static int wav500_cfg_get_station(struct wiphy *wiphy, struct net_device *dev,
				  const u8 *mac, struct station_info *sinfo)
{
	struct wav500_vif *vif = netdev_priv(dev);
	u16 sid;

	if (wav500_sta_find_sid(vif->priv, mac, &sid))
		return -ENOENT;
	sinfo->filled = 0;
	return 0;
}

static int wav500_cfg_set_wiphy_params(struct wiphy *wiphy, int radio_idx,
				       u32 changed)
{
	return 0;
}

static void wav500_cfg_update_mgmt_frame_reg(struct wiphy *wiphy,
					     struct wireless_dev *wdev,
					     struct mgmt_frame_regs *upd)
{
}

static const struct ieee80211_txrx_stypes
wav500_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_AP] = {
		.tx = 0xffff,
		.rx = 0xffff,
	},
};

const struct cfg80211_ops wav500_cfg80211_ops = {
	.change_virtual_intf = wav500_cfg_change_virtual_intf,
	.start_ap = wav500_cfg_start_ap,
	.change_beacon = wav500_cfg_change_beacon,
	.stop_ap = wav500_cfg_stop_ap,
	.add_station = wav500_cfg_add_station,
	.change_station = wav500_cfg_change_station,
	.del_station = wav500_cfg_del_station,
	.get_station = wav500_cfg_get_station,
	.add_key = wav500_cfg_add_key,
	.del_key = wav500_cfg_del_key,
	.set_default_key = wav500_cfg_set_default_key,
	.set_wiphy_params = wav500_cfg_set_wiphy_params,
	.connect = wav500_cfg_connect,
	.disconnect = wav500_cfg_disconnect,
	.mgmt_tx = wav500_cfg_mgmt_tx,
	.update_mgmt_frame_registrations = wav500_cfg_update_mgmt_frame_reg,
};

static const struct ieee80211_regdomain wav500_regdom = {
	.n_reg_rules = 3,
	.alpha2 = "99",
	.reg_rules = {
		REG_RULE(2402 - 10, 2472 + 10, 40, 0, 20, 0),
		REG_RULE(5170 - 10, 5250 + 10, 80, 0, 23, 0),
		REG_RULE(5735 - 10, 5835 + 10, 80, 0, 23, 0),
	},
};

int wav500_cfg_init_wiphy(struct wav500_priv *priv)
{
	struct wiphy *wiphy = priv->wiphy;

	wiphy->interface_modes = BIT(NL80211_IFTYPE_AP);
	wiphy->flags |= WIPHY_FLAG_REPORTS_OBSS;
	wiphy->regulatory_flags = REGULATORY_CUSTOM_REG;
	wiphy->available_antennas_tx = 0x0f;
	wiphy->available_antennas_rx = 0x0f;
	wiphy->mgmt_stypes = wav500_mgmt_stypes;
	wiphy->cipher_suites = wav500_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(wav500_cipher_suites);
	wiphy->max_scan_ssids = 4;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	if (priv->is_5ghz)
		wiphy->bands[NL80211_BAND_5GHZ] = &wav500_band_5ghz;
	else
		wiphy->bands[NL80211_BAND_2GHZ] = &wav500_band_2ghz;

	eth_random_addr(priv->mac_addr);
	ether_addr_copy(wiphy->perm_addr, priv->mac_addr);
	wiphy_apply_custom_regulatory(wiphy, &wav500_regdom);
	return 0;
}

int wav500_cfg_register_primary_vif(struct wav500_priv *priv)
{
	struct net_device *ndev;
	struct wav500_vif *vif;
	char name[IFNAMSIZ];
	int ret;

	snprintf(name, sizeof(name), "wlan%u", priv->radio_idx);
	ndev = alloc_netdev(sizeof(*vif), name, NET_NAME_ENUM, ether_setup);
	if (!ndev)
		return -ENOMEM;

	vif = netdev_priv(ndev);
	vif->ndev = ndev;
	vif->priv = priv;
	vif->vap_id = 0;
	vif->wdev.wiphy = priv->wiphy;
	vif->wdev.netdev = ndev;
	vif->wdev.iftype = NL80211_IFTYPE_AP;

	ndev->netdev_ops = &wav500_netdev_ops;
	ndev->ieee80211_ptr = &vif->wdev;
	SET_NETDEV_DEV(ndev, &priv->pdev->dev);
	eth_hw_addr_set(ndev, priv->mac_addr);
	netif_carrier_off(ndev);

	ret = register_netdev(ndev);
	if (ret) {
		free_netdev(ndev);
		return ret;
	}

	priv->primary_vif = vif;
	dev_info(&priv->pdev->dev, "registered %s (%s, MAC=%pM)\n",
		 ndev->name, priv->is_5ghz ? "5GHz" : "2.4GHz", priv->mac_addr);
	return 0;
}

void wav500_cfg_unregister_primary_vif(struct wav500_priv *priv)
{
	if (priv->primary_vif && priv->primary_vif->ndev) {
		struct net_device *ndev = priv->primary_vif->ndev;

		unregister_netdev(ndev);
		free_netdev(ndev);
		priv->primary_vif = NULL;
	}
}
