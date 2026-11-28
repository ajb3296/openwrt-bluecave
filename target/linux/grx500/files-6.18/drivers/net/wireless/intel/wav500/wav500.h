/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Intel / Lantiq WAV500 (WAV514/WAV524, gen5b) clean-room cfg80211 driver.
 */
#ifndef _WAV500_H_
#define _WAV500_H_

#include <linux/completion.h>
#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <net/cfg80211.h>

#include "wav500_regs.h"

#define WAV500_DP_RING_SIZE	64
#define WAV500_DP_HD_SIZE	20
#define WAV500_RX_BUF_SIZE	2048
#define WAV500_BSS_BUF_SIZE	2448
#define WAV500_MAX_STAS		32

struct wav500_hd {
	__le32 w[5];
};

struct wav500_dp_ring {
	struct wav500_hd *hds;
	dma_addr_t hds_dma;
	u32 count;
	u32 read_idx;
	u32 write_idx;
	u32 doorbell_off;
	u32 int_clear_off;
	u32 get_hds_off;
	void *buf_pool;
	dma_addr_t buf_pool_dma;
	u32 buf_count;
	size_t buf_stride;
	size_t buf_pool_len;
	unsigned long in_use[BITS_TO_LONGS(WAV500_DP_RING_SIZE)];
	u64 cookies[WAV500_DP_RING_SIZE];
	u16 tx_lens[WAV500_DP_RING_SIZE];
};

struct wav500_sta_entry {
	bool used;
	u8 mac[ETH_ALEN];
	u16 sid;
	u8 vap_id;
};

struct wav500_priv;

struct wav500_vif {
	struct wireless_dev wdev;
	struct net_device *ndev;
	struct wav500_priv *priv;
	u8 vap_id;
	bool ap_started;
};

struct wav500_chi {
	u32 ind_ring_off;
	u16 ind_ring_cnt;
	u16 ind_ring_idx;

	u32 req_ring_off;
	u16 req_ring_cnt;
	u16 req_ring_idx;

	u32 man_ind_bdr_off;
	u16 man_ind_bdr_cnt;

	u32 man_req_bdr_off;
	u16 man_req_bdr_cnt;

	/* Parsed CHI TLV extensions */
	u32 cal_off;
	u32 cal_size;
	u32 mips_ctrl_off;
	u32 nof_sta;
	u32 nof_vap;
	u16 stats_data_size;
	u16 stats_table_size;
};

struct wav500_priv {
	struct pci_dev *pdev;
	struct wiphy *wiphy;
	struct wav500_vif *primary_vif;
	void __iomem *bar0;
	resource_size_t bar0_len;
	u8 radio_idx;		/* 0 = 2.4GHz (domain 0), 1 = 5GHz (domain 1) */
	bool is_5ghz;
	u16 chip_id;
	u8 hw_type;
	u8 hw_rev;
	u16 xtal_val;
	u8 rssi_cal[128];
	u8 rssi_cal_len;
	u8 tpc_struct_size;
	u8 tpc_count;
	struct {
		u8 chan;
		u8 ant;
		u8 bw;
		u8 raw[20];
	} tpc_entries[48];
	u8 mac_addr[ETH_ALEN];
	bool fw_ready;
	bool pre_activated;
	bool vap_added;
	void *beacon_buf;
	dma_addr_t beacon_dma;
	size_t beacon_alloc_len;
	void *cal_buf;
	dma_addr_t cal_dma;
	size_t cal_alloc_len;
	void *dp_rings_buf;
	dma_addr_t dp_rings_dma;
	size_t dp_rings_len;
	bool dp_inited;
	bool dp_irq_enabled;
	u32 cur_freq;
	struct wav500_dp_ring tx_ring;
	struct wav500_dp_ring rx_ring;
	struct wav500_dp_ring bss_tx_ring;
	struct wav500_dp_ring bss_rx_ring;
	spinlock_t dp_lock;
	struct tasklet_struct dp_tasklet;
	u32 rx_mgmt_count;
	u32 tx_mgmt_count;
	u32 rx_data_count;
	u32 tx_data_count;
	spinlock_t sta_lock;
	struct wav500_sta_entry stas[WAV500_MAX_STAS];
	u64 mgmt_cookie;
	u32 irq_count;
	u32 chi_magic_seen;
	u32 chi_base[WAV500_CHI_BASE_SIZE / 4];

	struct wav500_chi chi;
	spinlock_t ring_lock;
	struct mutex man_lock;
	struct completion man_cfm;
	bool man_pending;
	u16 man_pending_idx;
	u8 man_cfm_buf[WAV500_MAN_REQ_PAYLOAD_MAX];
	u32 man_cfm_count;
	u32 man_ind_count;
};

static inline u32 wav500_r32(struct wav500_priv *priv, u32 reg)
{
	return readl(priv->bar0 + reg);
}

static inline void wav500_w32(struct wav500_priv *priv, u32 reg, u32 val)
{
	writel(val, priv->bar0 + reg);
}

/* wav500_ipc.c */
int wav500_chi_parse(struct wav500_priv *priv);
void wav500_ipc_poll_rx(struct wav500_priv *priv);
int wav500_ipc_send_man_req(struct wav500_priv *priv, u16 msg_id, u8 vap_idx,
			    const void *req, size_t req_len,
			    void *rsp, size_t rsp_len, unsigned int timeout_ms);
int wav500_umi_get_mib(struct wav500_priv *priv, u16 mib_id,
		       u16 *status_out, u8 *val_out, size_t val_len);
int wav500_load_progmodel(struct wav500_priv *priv, const char *filename);
int wav500_load_cal(struct wav500_priv *priv);
int wav500_umi_send_rssi_cal(struct wav500_priv *priv);
int wav500_load_psdb(struct wav500_priv *priv);
int wav500_umi_data_path_init(struct wav500_priv *priv);
int wav500_umi_hw_dep_cfg(struct wav500_priv *priv, u32 xtal_val);
int wav500_umi_pre_activate(struct wav500_priv *priv);
int wav500_umi_hdk_config(struct wav500_priv *priv, u8 ch_mode,
			  u32 offline_mask, u32 online_mask);
int wav500_umi_set_radio_mode(struct wav500_priv *priv, bool enable);
int wav500_umi_vap_db_op(struct wav500_priv *priv, u8 vap_id, bool add);
int wav500_umi_set_chan(struct wav500_priv *priv, u32 freq, u8 width,
			u32 center_freq);
int wav500_umi_activate_ap(struct wav500_priv *priv, u8 vap_id,
			   const u8 *bssid, const u8 *ssid, size_t ssid_len,
			   u16 beacon_int);
int wav500_umi_set_bss(struct wav500_priv *priv, u8 vap_id);
int wav500_umi_set_tpc_default(struct wav500_priv *priv);
int wav500_umi_set_beacon(struct wav500_priv *priv, u8 vap_id,
			  const u8 *head, size_t head_len,
			  const u8 *tail, size_t tail_len);
int wav500_umi_calibrate(struct wav500_priv *priv, u8 ch, u8 width);
int wav500_umi_set_antennas(struct wav500_priv *priv, u8 tx_mask, u8 rx_mask);
int wav500_umi_set_tx_power_offset(struct wav500_priv *priv, u8 offset);
int wav500_umi_set_tpc_config(struct wav500_priv *priv);
int wav500_umi_set_tpc_gen5(struct wav500_priv *priv, u8 target_ch);
int wav500_umi_set_wmm_default(struct wav500_priv *priv, u8 vap_id);
int wav500_umi_request_sid(struct wav500_priv *priv, const u8 *mac, u16 *sid_out);
int wav500_umi_remove_sid(struct wav500_priv *priv, u16 sid);
int wav500_umi_add_sta(struct wav500_priv *priv, u8 vap_id, u16 sid,
		       const u8 *mac, struct station_parameters *params);
int wav500_umi_remove_sta(struct wav500_priv *priv, u16 sid);
int wav500_umi_set_key(struct wav500_priv *priv, u8 vap_id, u16 sid,
		       u8 key_idx, bool pairwise, struct key_params *params);
int wav500_umi_del_key(struct wav500_priv *priv, u8 vap_id, u16 sid,
		       u8 key_idx, bool pairwise);
int wav500_umi_set_default_key(struct wav500_priv *priv, u8 vap_id, u16 sid,
			       bool is_group, u8 key_idx);
int wav500_start_test_ap(struct wav500_priv *priv, int mode);

/* wav500_pci.c */
int wav500_boot_fw(struct wav500_priv *priv);

/* wav500_dma.c */
int wav500_dma_init_rings(struct wav500_priv *priv);
void wav500_dma_cleanup_rings(struct wav500_priv *priv);
void wav500_dma_enable_irqs(struct wav500_priv *priv);
void wav500_dma_tasklet(struct tasklet_struct *t);
int wav500_dma_tx_mgmt(struct wav500_priv *priv, u8 vap_id,
		       const u8 *buf, size_t len, u64 cookie);
int wav500_dma_tx_eapol(struct wav500_priv *priv, u8 vap_id,
			const u8 *buf, size_t len);
int wav500_dma_tx_data(struct wav500_priv *priv, u8 vap_id,
		       struct sk_buff *skb);
int wav500_sta_find_sid(struct wav500_priv *priv, const u8 *mac, u16 *sid_out);
int wav500_sta_register(struct wav500_priv *priv, const u8 *mac, u16 sid, u8 vap_id);
void wav500_sta_unregister(struct wav500_priv *priv, const u8 *mac);

/* wav500_cfg.c */
extern const struct cfg80211_ops wav500_cfg80211_ops;
int wav500_cfg_init_wiphy(struct wav500_priv *priv);
int wav500_cfg_register_primary_vif(struct wav500_priv *priv);
void wav500_cfg_unregister_primary_vif(struct wav500_priv *priv);

#endif /* _WAV500_H_ */
