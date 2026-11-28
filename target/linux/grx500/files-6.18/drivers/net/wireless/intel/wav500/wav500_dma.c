// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel / Lantiq WAV500 (WAV514/WAV524, gen5b) clean-room DMA data-path layer.
 *
 * Implements SW internal rings (mode 0) for TX data, RX data, BSS/Mgmt TX,
 * and BSS/Mgmt RX, plus station SID lookup and tasklet-based completion/RX.
 */

#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ieee80211.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/unaligned.h>
#include <net/cfg80211.h>

#include "wav500.h"

#define WAV500_HD_OWN_BIT	BIT(31)
#define WAV500_HD_SINGLE_FRAG	0x30000000U
#define WAV500_BSS_RX_HDR_LEN	0x58

static void wav500_ring_parse_chi(struct wav500_priv *priv,
				  struct wav500_dp_ring *ring,
				  u32 chi_off)
{
	ring->doorbell_off  = wav500_r32(priv, chi_off + 0);
	ring->int_clear_off = wav500_r32(priv, chi_off + 8);
	ring->get_hds_off   = wav500_r32(priv, chi_off + 12);
}

static inline u32 wav500_ring_get_hds(struct wav500_priv *priv,
				      struct wav500_dp_ring *ring)
{
	if (!ring->get_hds_off)
		return 0;
	return wav500_r32(priv, ring->get_hds_off);
}

static inline void wav500_ring_clear_int(struct wav500_priv *priv,
					 struct wav500_dp_ring *ring,
					 u32 count)
{
	if (ring->int_clear_off && count)
		wav500_w32(priv, ring->int_clear_off, count);
}

static inline void wav500_ring_doorbell(struct wav500_priv *priv,
					struct wav500_dp_ring *ring)
{
	wmb();
	if (ring->doorbell_off)
		wav500_w32(priv, ring->doorbell_off, 1);
}

static void wav500_ring_push_hd(struct wav500_priv *priv,
				struct wav500_dp_ring *ring,
				const struct wav500_hd *hd)
{
	struct wav500_hd tmp = *hd;
	u32 w3 = le32_to_cpu(tmp.w[3]) & ~WAV500_HD_OWN_BIT;

	tmp.w[3] = cpu_to_le32(w3);
	memcpy(&ring->hds[ring->write_idx], &tmp, sizeof(tmp));
	ring->write_idx++;
	if (ring->write_idx >= ring->count)
		ring->write_idx = 0;
	wav500_ring_doorbell(priv, ring);
}

static int wav500_ring_alloc_pool(struct wav500_priv *priv,
				  struct wav500_dp_ring *ring,
				  size_t stride, u32 buf_count)
{
	struct device *dev = &priv->pdev->dev;

	ring->buf_count = buf_count;
	ring->buf_stride = stride;
	ring->buf_pool_len = ALIGN(stride * buf_count, PAGE_SIZE);
	ring->buf_pool = dma_alloc_coherent(dev, ring->buf_pool_len,
					    &ring->buf_pool_dma, GFP_KERNEL);
	if (!ring->buf_pool)
		return -ENOMEM;
	memset(ring->buf_pool, 0, ring->buf_pool_len);
	return 0;
}

static void wav500_ring_free_pool(struct wav500_priv *priv,
				  struct wav500_dp_ring *ring)
{
	if (ring->buf_pool) {
		dma_free_coherent(&priv->pdev->dev, ring->buf_pool_len,
				  ring->buf_pool, ring->buf_pool_dma);
		ring->buf_pool = NULL;
	}
}

static void wav500_populate_rd_pool(struct wav500_priv *priv,
				    u32 rd_base, u32 rd_esz,
				    u32 rd_dma_off, u32 rd_idx_off,
				    u32 start_idx,
				    struct wav500_dp_ring *ring)
{
	u32 i;

	for (i = 0; i < ring->buf_count; i++) {
		u32 entry = rd_base + (start_idx + i) * rd_esz;
		u32 dma = (u32)(ring->buf_pool_dma + i * ring->buf_stride);
		u32 idx_word_off = entry + (rd_idx_off & ~3U);
		u32 val = wav500_r32(priv, idx_word_off);

		if (rd_idx_off & 2U)
			val = (val & 0x0000ffffU) | ((i & 0xffffU) << 16);
		else
			val = (val & 0xffff0000U) | (i & 0xffffU);

		wav500_w32(priv, entry + rd_dma_off, dma);
		wav500_w32(priv, idx_word_off, val);
	}
}

int wav500_dma_init_rings(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	const u32 ring_cnt = WAV500_DP_RING_SIZE;
	const u32 ring_bytes = ring_cnt * WAV500_DP_HD_SIZE;
	u32 rd_base, rd_esz, rd_dma_off, rd_idx_off;
	u32 data_rd_cnt, mgmt_rd_cnt;
	int ret;

	if (priv->dp_inited)
		return 0;

	if (!priv->dp_rings_buf) {
		priv->dp_rings_len = ALIGN(ring_bytes * 4, PAGE_SIZE);
		priv->dp_rings_buf = dma_alloc_coherent(dev, priv->dp_rings_len,
							&priv->dp_rings_dma,
							GFP_KERNEL);
		if (!priv->dp_rings_buf)
			return -ENOMEM;
	}
	memset(priv->dp_rings_buf, 0, priv->dp_rings_len);

	memset(&priv->tx_ring, 0, sizeof(priv->tx_ring));
	memset(&priv->rx_ring, 0, sizeof(priv->rx_ring));
	memset(&priv->bss_tx_ring, 0, sizeof(priv->bss_tx_ring));
	memset(&priv->bss_rx_ring, 0, sizeof(priv->bss_rx_ring));

	priv->tx_ring.count = ring_cnt;
	priv->tx_ring.hds = priv->dp_rings_buf;
	priv->tx_ring.hds_dma = priv->dp_rings_dma;

	priv->rx_ring.count = ring_cnt;
	priv->rx_ring.hds = (void *)((u8 *)priv->dp_rings_buf + ring_bytes);
	priv->rx_ring.hds_dma = priv->dp_rings_dma + ring_bytes;

	priv->bss_tx_ring.count = ring_cnt;
	priv->bss_tx_ring.hds = (void *)((u8 *)priv->dp_rings_buf + ring_bytes * 2);
	priv->bss_tx_ring.hds_dma = priv->dp_rings_dma + ring_bytes * 2;

	priv->bss_rx_ring.count = ring_cnt;
	priv->bss_rx_ring.hds = (void *)((u8 *)priv->dp_rings_buf + ring_bytes * 3);
	priv->bss_rx_ring.hds_dma = priv->dp_rings_dma + ring_bytes * 3;

	wav500_ring_parse_chi(priv, &priv->tx_ring, WAV500_CHI_RING_TX_OFF);
	wav500_ring_parse_chi(priv, &priv->rx_ring, WAV500_CHI_RING_RX_OFF);
	wav500_ring_parse_chi(priv, &priv->bss_tx_ring, WAV500_CHI_RING_BSS_TX_OFF);
	wav500_ring_parse_chi(priv, &priv->bss_rx_ring, WAV500_CHI_RING_BSS_RX_OFF);

	/* Read SHRAM RX RD pool descriptors (CHI + 0xb0..0xc4) */
	rd_base     = wav500_r32(priv, 0xb0);
	rd_esz      = wav500_r32(priv, 0xb4);
	rd_dma_off  = wav500_r32(priv, 0xb8);
	rd_idx_off  = wav500_r32(priv, 0xbc);
	data_rd_cnt = wav500_r32(priv, 0xc0);
	mgmt_rd_cnt = wav500_r32(priv, 0xc4);

	dev_info(dev,
		 "RD pool: base=0x%x esz=%u dma_off=%u idx_off=%u data_cnt=%u mgmt_cnt=%u\n",
		 rd_base, rd_esz, rd_dma_off, rd_idx_off, data_rd_cnt, mgmt_rd_cnt);

	if (!data_rd_cnt || data_rd_cnt > 1024)
		data_rd_cnt = ring_cnt;
	if (!mgmt_rd_cnt || mgmt_rd_cnt > 256)
		mgmt_rd_cnt = ring_cnt;

	ret = wav500_ring_alloc_pool(priv, &priv->tx_ring, WAV500_RX_BUF_SIZE, ring_cnt);
	if (ret)
		goto err_free;
	ret = wav500_ring_alloc_pool(priv, &priv->rx_ring, WAV500_RX_BUF_SIZE, data_rd_cnt);
	if (ret)
		goto err_free;
	ret = wav500_ring_alloc_pool(priv, &priv->bss_tx_ring, WAV500_BSS_BUF_SIZE, ring_cnt);
	if (ret)
		goto err_free;
	ret = wav500_ring_alloc_pool(priv, &priv->bss_rx_ring, WAV500_BSS_BUF_SIZE, mgmt_rd_cnt);
	if (ret)
		goto err_free;

	/* Populate SHRAM RD pool with coherent RX data & BSS/mgmt buffers */
	if (rd_base && rd_esz) {
		wav500_populate_rd_pool(priv, rd_base, rd_esz, rd_dma_off,
					rd_idx_off, 0, &priv->rx_ring);
		wav500_populate_rd_pool(priv, rd_base, rd_esz, rd_dma_off,
					rd_idx_off, data_rd_cnt, &priv->bss_rx_ring);
	}

	wmb();
	ret = wav500_umi_data_path_init(priv);
	if (ret)
		goto err_free;

	dev_info(dev,
		 "DP rings ready: tx_db=0x%x rx_db=0x%x bss_tx_db=0x%x bss_rx_db=0x%x\n",
		 priv->tx_ring.doorbell_off, priv->rx_ring.doorbell_off,
		 priv->bss_tx_ring.doorbell_off, priv->bss_rx_ring.doorbell_off);
	return 0;

err_free:
	wav500_dma_cleanup_rings(priv);
	return ret;
}

void wav500_dma_cleanup_rings(struct wav500_priv *priv)
{
	priv->dp_irq_enabled = false;
	priv->dp_inited = false;
	wav500_ring_free_pool(priv, &priv->tx_ring);
	wav500_ring_free_pool(priv, &priv->rx_ring);
	wav500_ring_free_pool(priv, &priv->bss_tx_ring);
	wav500_ring_free_pool(priv, &priv->bss_rx_ring);
}

void wav500_dma_enable_irqs(struct wav500_priv *priv)
{
	if (!priv->dp_inited)
		return;
	priv->dp_irq_enabled = true;
	wav500_w32(priv, WAV500_IRQ_MASK, 0x2f);
}

int wav500_sta_find_sid(struct wav500_priv *priv, const u8 *mac, u16 *sid_out)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&priv->sta_lock, flags);
	for (i = 0; i < WAV500_MAX_STAS; i++) {
		if (priv->stas[i].used && ether_addr_equal(priv->stas[i].mac, mac)) {
			*sid_out = priv->stas[i].sid;
			spin_unlock_irqrestore(&priv->sta_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&priv->sta_lock, flags);
	return -ENOENT;
}

int wav500_sta_register(struct wav500_priv *priv, const u8 *mac, u16 sid, u8 vap_id)
{
	unsigned long flags;
	int i, free_idx = -1;

	spin_lock_irqsave(&priv->sta_lock, flags);
	for (i = 0; i < WAV500_MAX_STAS; i++) {
		if (priv->stas[i].used && ether_addr_equal(priv->stas[i].mac, mac)) {
			priv->stas[i].sid = sid;
			priv->stas[i].vap_id = vap_id;
			spin_unlock_irqrestore(&priv->sta_lock, flags);
			return 0;
		}
		if (!priv->stas[i].used && free_idx < 0)
			free_idx = i;
	}
	if (free_idx < 0) {
		spin_unlock_irqrestore(&priv->sta_lock, flags);
		return -ENOSPC;
	}
	priv->stas[free_idx].used = true;
	ether_addr_copy(priv->stas[free_idx].mac, mac);
	priv->stas[free_idx].sid = sid;
	priv->stas[free_idx].vap_id = vap_id;
	spin_unlock_irqrestore(&priv->sta_lock, flags);
	return 0;
}

void wav500_sta_unregister(struct wav500_priv *priv, const u8 *mac)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&priv->sta_lock, flags);
	for (i = 0; i < WAV500_MAX_STAS; i++) {
		if (priv->stas[i].used && ether_addr_equal(priv->stas[i].mac, mac)) {
			priv->stas[i].used = false;
			break;
		}
	}
	spin_unlock_irqrestore(&priv->sta_lock, flags);
}

static void wav500_deliver_eth_frame(struct wav500_priv *priv,
				     const u8 *data, size_t len)
{
	struct wav500_vif *vif = priv->primary_vif;
	struct net_device *ndev;
	struct sk_buff *skb;

	if (!vif || !vif->ndev || len < ETH_HLEN || len > WAV500_RX_BUF_SIZE)
		return;
	ndev = vif->ndev;

	skb = netdev_alloc_skb_ip_align(ndev, len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}
	skb_put_data(skb, data, len);
	skb->protocol = eth_type_trans(skb, ndev);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;
	priv->rx_data_count++;
	netif_rx(skb);
}

static void wav500_drain_bss_rx(struct wav500_priv *priv)
{
	struct wav500_dp_ring *ring = &priv->bss_rx_ring;
	struct wav500_vif *vif = priv->primary_vif;
	u32 pending = wav500_ring_get_hds(priv, ring);
	u32 consumed = 0;
	u32 budget = min_t(u32, pending, ring->count);

	while (consumed < budget) {
		struct wav500_hd *hd = &ring->hds[ring->read_idx];
		u32 w0, w2, w3, idx, off, len;
		dma_addr_t buf_dma;
		size_t pool_off;
		u8 *buf;

		rmb();
		w3 = le32_to_cpu(hd->w[3]);
		if (!(w3 & WAV500_HD_OWN_BIT))
			break;

		w0  = le32_to_cpu(hd->w[0]);
		w2  = le32_to_cpu(hd->w[2]);
		idx = le32_to_cpu(hd->w[4]);
		off = (w3 >> 23) & 0x7;
		len = w3 & 0xffff;

		buf_dma = (dma_addr_t)w2 + off;
		if (buf_dma >= ring->buf_pool_dma &&
		    buf_dma + len <= ring->buf_pool_dma + ring->buf_pool_len &&
		    len > 0) {
			pool_off = (size_t)(buf_dma - ring->buf_pool_dma);
			buf = (u8 *)ring->buf_pool + pool_off;

			if (priv->rx_mgmt_count < 3 && len > WAV500_BSS_RX_HDR_LEN) {
				const u8 *f = buf + WAV500_BSS_RX_HDR_LEN;

				dev_dbg(&priv->pdev->dev,
					"BSS_RX[%u]: w0=0x%08x w2=0x%08x w3=0x%08x w4=0x%08x len=%u fc=%02x%02x da=%pM sa=%pM\n",
					priv->rx_mgmt_count, w0, w2, w3, idx, len,
					f[0], f[1], f + 4, f + 10);
			}

			if (((w0 >> 15) & 3) == 3 && ((w0 >> 24) & 3) == 2) {
				/* EAPOL 802.3 frame */
				wav500_deliver_eth_frame(priv, buf, len);
			} else if (len > WAV500_BSS_RX_HDR_LEN && vif) {
				s8 rssi = (s8)buf[0x50];
				int a;

				for (a = 1; a < 4; a++)
					if ((s8)buf[0x50 + a] > rssi)
						rssi = (s8)buf[0x50 + a];
				priv->rx_mgmt_count++;
				cfg80211_rx_mgmt(&vif->wdev,
						 priv->cur_freq ? priv->cur_freq :
						 (priv->is_5ghz ? 5180 : 2437),
						 ((int)rssi - 65) * 100,
						 buf + WAV500_BSS_RX_HDR_LEN,
						 len - WAV500_BSS_RX_HDR_LEN, 0);
			}
		}

		/* Return the exact same HD to FW with OWN_BIT cleared */
		{
			struct wav500_hd rhd = *hd;

			ring->read_idx++;
			if (ring->read_idx >= ring->count)
				ring->read_idx = 0;
			wav500_ring_push_hd(priv, ring, &rhd);
		}
		consumed++;
	}

	if (consumed)
		wav500_ring_clear_int(priv, ring, consumed);
}

static void wav500_drain_bss_tx(struct wav500_priv *priv)
{
	struct wav500_dp_ring *ring = &priv->bss_tx_ring;
	struct wav500_vif *vif = priv->primary_vif;
	u32 pending = wav500_ring_get_hds(priv, ring);
	u32 consumed = 0;
	u32 budget = min_t(u32, pending, ring->count);

	while (consumed < budget) {
		struct wav500_hd *hd = &ring->hds[ring->read_idx];
		u32 idx;

		rmb();
		idx = le32_to_cpu(hd->w[4]);
		ring->read_idx++;
		if (ring->read_idx >= ring->count)
			ring->read_idx = 0;

		if (idx < ring->count) {
			u64 cookie = ring->cookies[idx];
			u16 flen = ring->tx_lens[idx];
			const u8 *fbuf = (const u8 *)ring->buf_pool +
					 idx * ring->buf_stride;

			ring->cookies[idx] = 0;
			ring->tx_lens[idx] = 0;
			if (cookie && vif && flen)
				cfg80211_mgmt_tx_status(&vif->wdev, cookie,
							fbuf, flen, true,
							GFP_ATOMIC);
			clear_bit(idx, ring->in_use);
		}
		consumed++;
	}

	if (consumed)
		wav500_ring_clear_int(priv, ring, consumed);
}

static void wav500_drain_data_rx(struct wav500_priv *priv)
{
	struct wav500_dp_ring *ring = &priv->rx_ring;
	u32 pending = wav500_ring_get_hds(priv, ring);
	u32 consumed = 0;
	u32 budget = min_t(u32, pending, ring->count);

	while (consumed < budget) {
		struct wav500_hd *hd = &ring->hds[ring->read_idx];
		u32 w0, w3, idx, off, len;
		u8 pkt_type;

		rmb();
		w3 = le32_to_cpu(hd->w[3]);
		if (!(w3 & WAV500_HD_OWN_BIT))
			break;

		w0 = le32_to_cpu(hd->w[0]);
		idx = le32_to_cpu(hd->w[4]);
		off = (w3 >> 23) & 0x7;
		len = w3 & 0xffff;
		pkt_type = w0 & 0xff;

		if (idx < ring->buf_count && pkt_type != 0x80 &&
		    len >= ETH_HLEN && off + len <= ring->buf_stride) {
			const u8 *buf = (const u8 *)ring->buf_pool +
					idx * ring->buf_stride + off;

			wav500_deliver_eth_frame(priv, buf, len);
		}

		/* Return the exact same RX HD to FW with OWN_BIT cleared */
		{
			struct wav500_hd rhd = *hd;

			ring->read_idx++;
			if (ring->read_idx >= ring->count)
				ring->read_idx = 0;
			wav500_ring_push_hd(priv, ring, &rhd);
		}
		consumed++;
	}

	if (consumed)
		wav500_ring_clear_int(priv, ring, consumed);
}

static void wav500_drain_data_tx(struct wav500_priv *priv)
{
	struct wav500_dp_ring *ring = &priv->tx_ring;
	u32 pending = wav500_ring_get_hds(priv, ring);
	u32 consumed = 0;
	u32 budget = min_t(u32, pending, ring->count);

	while (consumed < budget) {
		struct wav500_hd *hd = &ring->hds[ring->read_idx];
		u32 idx;

		rmb();
		idx = le32_to_cpu(hd->w[4]);
		ring->read_idx++;
		if (ring->read_idx >= ring->count)
			ring->read_idx = 0;
		if (idx < ring->count)
			clear_bit(idx, ring->in_use);
		consumed++;
	}

	if (consumed)
		wav500_ring_clear_int(priv, ring, consumed);
}

void wav500_dma_tasklet(struct tasklet_struct *t)
{
	struct wav500_priv *priv = from_tasklet(priv, t, dp_tasklet);
	unsigned long flags;

	if (!priv->dp_inited)
		return;

	spin_lock_irqsave(&priv->dp_lock, flags);
	wav500_drain_bss_rx(priv);
	wav500_drain_bss_tx(priv);
	wav500_drain_data_rx(priv);
	wav500_drain_data_tx(priv);
	if (priv->dp_irq_enabled)
		wav500_w32(priv, WAV500_IRQ_MASK, 0x2f);
	spin_unlock_irqrestore(&priv->dp_lock, flags);
}

static int wav500_bss_tx_submit(struct wav500_priv *priv, u8 vap_id,
				const u8 *buf, size_t len,
				bool is_eapol, u64 cookie)
{
	struct wav500_dp_ring *ring = &priv->bss_tx_ring;
	unsigned long flags;
	struct wav500_hd hd = { };
	dma_addr_t dma;
	u32 w0, slot;
	u16 sid = 0xfb; /* broadcast/unassociated default */
	bool is_mcast;

	if (!priv->dp_inited || !buf || len == 0 || len > ring->buf_stride)
		return -EINVAL;

	if (is_eapol) {
		if (len < ETH_HLEN || is_multicast_ether_addr(buf))
			return -EINVAL;
		if (wav500_sta_find_sid(priv, buf, &sid))
			return -ENOENT;
		is_mcast = false;
	} else {
		if (len < 16)
			return -EINVAL;
		is_mcast = is_multicast_ether_addr(buf + 4);
		if (!is_mcast)
			wav500_sta_find_sid(priv, buf + 4, &sid);
	}

	spin_lock_irqsave(&priv->dp_lock, flags);
	slot = find_first_zero_bit(ring->in_use, ring->count);
	if (slot >= ring->count) {
		spin_unlock_irqrestore(&priv->dp_lock, flags);
		return -EBUSY;
	}
	set_bit(slot, ring->in_use);
	ring->cookies[slot] = cookie;
	ring->tx_lens[slot] = (u16)len;

	memcpy((u8 *)ring->buf_pool + slot * ring->buf_stride, buf, len);
	dma = ring->buf_pool_dma + slot * ring->buf_stride;

	if (is_eapol) {
		w0 = 0x22018000U | ((u32)(vap_id & 0xf) << 8) | (sid & 0xff);
	} else {
		u32 subtype = (buf[0] >> 4) & 0xf;
		u32 dst_bits = is_mcast ? 0x4000U : (sid & 0xff);

		w0 = (subtype << 26) | ((u32)(vap_id & 0xf) << 8) | dst_bits;
	}

	hd.w[0] = cpu_to_le32(w0);
	hd.w[1] = cpu_to_le32(7);
	hd.w[2] = cpu_to_le32((u32)(dma & ~7ULL));
	hd.w[3] = cpu_to_le32((((u32)dma & 7U) << 23) |
			      WAV500_HD_SINGLE_FRAG | (u32)len);
	hd.w[4] = cpu_to_le32(slot);

	priv->tx_mgmt_count++;
	wav500_ring_push_hd(priv, ring, &hd);
	spin_unlock_irqrestore(&priv->dp_lock, flags);
	return 0;
}

int wav500_dma_tx_mgmt(struct wav500_priv *priv, u8 vap_id,
		       const u8 *buf, size_t len, u64 cookie)
{
	return wav500_bss_tx_submit(priv, vap_id, buf, len, false, cookie);
}

int wav500_dma_tx_eapol(struct wav500_priv *priv, u8 vap_id,
			const u8 *buf, size_t len)
{
	return wav500_bss_tx_submit(priv, vap_id, buf, len, true, 0);
}

int wav500_dma_tx_data(struct wav500_priv *priv, u8 vap_id,
		       struct sk_buff *skb)
{
	struct wav500_dp_ring *ring = &priv->tx_ring;
	unsigned long flags;
	struct wav500_hd hd = { };
	dma_addr_t dma;
	u32 w0, slot;
	u16 sid = 0;
	bool is_mcast;

	if (!priv->dp_inited || skb->len < ETH_HLEN || skb->len > ring->buf_stride)
		return -EINVAL;

	/* Route 802.1X EAPOL frames over BSS_TX for reliable control delivery */
	if (skb->protocol == htons(ETH_P_PAE))
		return wav500_dma_tx_eapol(priv, vap_id, skb->data, skb->len);

	is_mcast = is_multicast_ether_addr(skb->data);
	if (!is_mcast && wav500_sta_find_sid(priv, skb->data, &sid))
		return -ENOENT;

	spin_lock_irqsave(&priv->dp_lock, flags);
	slot = find_first_zero_bit(ring->in_use, ring->count);
	if (slot >= ring->count) {
		spin_unlock_irqrestore(&priv->dp_lock, flags);
		return -EBUSY;
	}
	set_bit(slot, ring->in_use);

	memcpy((u8 *)ring->buf_pool + slot * ring->buf_stride, skb->data, skb->len);
	dma = ring->buf_pool_dma + slot * ring->buf_stride;

	w0 = ((u32)(vap_id & 0xf) << 8) | (is_mcast ? 0x4000U : (sid & 0xff));
	hd.w[0] = cpu_to_le32(w0);
	hd.w[1] = 0;
	hd.w[2] = cpu_to_le32((u32)(dma & ~7ULL));
	hd.w[3] = cpu_to_le32((((u32)dma & 7U) << 23) |
			      WAV500_HD_SINGLE_FRAG | (u32)skb->len);
	hd.w[4] = cpu_to_le32(slot);

	priv->tx_data_count++;
	wav500_ring_push_hd(priv, ring, &hd);
	spin_unlock_irqrestore(&priv->dp_lock, flags);
	return 0;
}
