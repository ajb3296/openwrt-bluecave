// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel / Lantiq WAV500 (WAV514/WAV524, gen5b) clean-room IPC & CHI layer.
 *
 * Implements CHI base + TLV extension parsing, Host<->FW management rings,
 * and synchronous UMI management request/confirm transactions.
 */

#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/rtnetlink.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "wav500.h"

static void wav500_shram_read_bytes(struct wav500_priv *priv, u32 off,
				    void *dst, size_t len)
{
	__le32 *out = dst;
	size_t words = len / 4;
	size_t i;

	for (i = 0; i < words; i++)
		out[i] = cpu_to_le32(wav500_r32(priv, off + i * 4));
}

static void wav500_shram_write_bytes(struct wav500_priv *priv, u32 off,
				     const void *src, size_t len)
{
	const __le32 *in = src;
	size_t words = len / 4;
	size_t i;

	for (i = 0; i < words; i++)
		wav500_w32(priv, off + i * 4, le32_to_cpu(in[i]));
	wmb();
}

int wav500_chi_parse(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	struct wav500_chi *chi = &priv->chi;
	u32 off;
	int ext_count = 0;

	memset(chi, 0, sizeof(*chi));

	chi->ind_ring_off = priv->chi_base[WAV500_CHI_IND_RING_OFF_IDX];
	chi->ind_ring_cnt = priv->chi_base[WAV500_CHI_IND_RING_CNT_IDX] & 0xffff;
	chi->req_ring_off = priv->chi_base[WAV500_CHI_REQ_RING_OFF_IDX];
	chi->req_ring_cnt = priv->chi_base[WAV500_CHI_REQ_RING_CNT_IDX] & 0xffff;

	chi->man_ind_bdr_off = priv->chi_base[WAV500_CHI_MAN_IND_OFF_IDX];
	chi->man_ind_bdr_cnt = priv->chi_base[WAV500_CHI_MAN_IND_CNT_IDX] & 0xff;
	chi->man_req_bdr_off = priv->chi_base[WAV500_CHI_MAN_REQ_OFF_IDX];
	chi->man_req_bdr_cnt = priv->chi_base[WAV500_CHI_MAN_REQ_CNT_IDX] & 0xffff;

	dev_info(dev,
		 "CHI rings: ind=0x%06x/%u req=0x%06x/%u man_ind=0x%06x/%u man_req=0x%06x/%u\n",
		 chi->ind_ring_off, chi->ind_ring_cnt,
		 chi->req_ring_off, chi->req_ring_cnt,
		 chi->man_ind_bdr_off, chi->man_ind_bdr_cnt,
		 chi->man_req_bdr_off, chi->man_req_bdr_cnt);

	if (!chi->ind_ring_cnt || !chi->req_ring_cnt || !chi->man_req_bdr_cnt) {
		dev_err(dev, "CHI ring parameters invalid\n");
		return -EIO;
	}

	/* Walk TLV extensions immediately following CHI_MAGIC at SHRAM+0xcc */
	off = WAV500_CHI_EXT_OFF;
	while (off + 12 < 0x1000 && ext_count < 16) {
		u32 magic = wav500_r32(priv, off);
		u32 ext_id, ext_len;

		if (magic != WAV500_CHI_MAGIC)
			break;

		ext_id = wav500_r32(priv, off + 4);
		ext_len = wav500_r32(priv, off + 8);
		off += 12;

		if (ext_len > 0x200 || (ext_len & 3)) {
			dev_warn(dev, "CHI ext %u invalid len %u\n", ext_id, ext_len);
			break;
		}

		switch (ext_id) {
		case WAV500_CHI_EXT_CAL:
			if (ext_len == 8) {
				chi->cal_off = wav500_r32(priv, off);
				chi->cal_size = wav500_r32(priv, off + 4);
			}
			break;
		case WAV500_CHI_EXT_MIPS_CTRL:
			if (ext_len == 4)
				chi->mips_ctrl_off = wav500_r32(priv, off);
			break;
		case WAV500_CHI_EXT_NOF_STA:
			if (ext_len == 4)
				chi->nof_sta = wav500_r32(priv, off);
			break;
		case WAV500_CHI_EXT_NOF_VAP:
			if (ext_len == 4)
				chi->nof_vap = wav500_r32(priv, off);
			break;
		case WAV500_CHI_EXT_STATS_MGR:
			if (ext_len >= 4) {
				u32 w0 = wav500_r32(priv, off);

				chi->stats_data_size = w0 & 0xffff;
				chi->stats_table_size = (w0 >> 16) & 0xffff;
			}
			break;
		default:
			break;
		}

		off += ext_len;
		ext_count++;
	}

	dev_info(dev,
		 "CHI extensions (%d): cal=0x%06x/%u mips_ctrl=0x%06x max_sta=%u max_vap=%u stats=%u/%u\n",
		 ext_count, chi->cal_off, chi->cal_size,
		 chi->mips_ctrl_off, chi->nof_sta, chi->nof_vap,
		 chi->stats_data_size, chi->stats_table_size);
	return 0;
}

static void wav500_ipc_ring_push(struct wav500_priv *priv, u32 entry)
{
	struct wav500_chi *chi = &priv->chi;
	unsigned long flags;

	spin_lock_irqsave(&priv->ring_lock, flags);
	wav500_w32(priv, chi->req_ring_off + chi->req_ring_idx * 4, entry);
	chi->req_ring_idx++;
	if (chi->req_ring_idx >= chi->req_ring_cnt)
		chi->req_ring_idx = 0;
	wmb();
	wav500_w32(priv, WAV500_HOST_DOORBELL, 1);
	wmb();
	spin_unlock_irqrestore(&priv->ring_lock, flags);
}

void wav500_ipc_poll_rx(struct wav500_priv *priv)
{
	struct wav500_chi *chi = &priv->chi;
	struct device *dev = &priv->pdev->dev;
	unsigned int budget;

	if (!priv->fw_ready || !chi->ind_ring_cnt)
		return;

	for (budget = 0; budget < chi->ind_ring_cnt; budget++) {
		u32 slot_off = chi->ind_ring_off + chi->ind_ring_idx * 4;
		u32 entry = wav500_r32(priv, slot_off);
		u32 type = entry & WAV500_RING_TYPE_MASK;
		u16 idx = (entry & 0x7fff) >> 5;

		if (!type)
			break;

		/* Consume ring slot and acknowledge host interrupt */
		wav500_w32(priv, slot_off, 0);
		wmb();
		wav500_w32(priv, WAV500_IRQ_CLEAR, 1);

		chi->ind_ring_idx++;
		if (chi->ind_ring_idx >= chi->ind_ring_cnt)
			chi->ind_ring_idx = 0;

		if (type == WAV500_RING_TYPE_MAN_REQ) {
			u32 bdr_off = chi->man_req_bdr_off +
				      idx * WAV500_MAN_REQ_SLOT_SIZE;

			priv->man_cfm_count++;
			if (priv->man_pending && idx == priv->man_pending_idx) {
				wav500_shram_read_bytes(priv,
							bdr_off + WAV500_MAN_REQ_HDR_SIZE,
							priv->man_cfm_buf,
							WAV500_MAN_REQ_PAYLOAD_MAX);
				priv->man_pending = false;
				complete(&priv->man_cfm);
			}
		} else if (type == WAV500_RING_TYPE_MAN_IND) {
			u32 bdr_off = chi->man_ind_bdr_off +
				      idx * WAV500_MAN_IND_SLOT_SIZE;
			u32 hdr1 = wav500_r32(priv, bdr_off + 4);
			u32 p0 = wav500_r32(priv, bdr_off + 0x10);
			u32 p1 = wav500_r32(priv, bdr_off + 0x14);
			u16 ind_id = (hdr1 >> 16) & 0xffff;

			priv->man_ind_count++;
			if (ind_id == 0x3338 && (p0 & 0xff) == 4) {
				u32 origin = wav500_r32(priv, bdr_off + 0x18);
				u32 ts     = wav500_r32(priv, bdr_off + 0x1c);
				u32 line   = wav500_r32(priv, bdr_off + 0x20);
				u32 fid    = wav500_r32(priv, bdr_off + 0x2c);
				u32 gid    = wav500_r32(priv, bdr_off + 0x30);

				dev_err(dev,
					"MAC_FATAL (%s): GroupID=%u FileID=%u Line=%u TS=0x%08x (p0=0x%08x p1=0x%08x) - marking radio offline\n",
					origin ? "upper" : "lower",
					gid, fid, line, ts, p0, p1);
				priv->fw_ready = false;
				priv->dp_irq_enabled = false;
				priv->vap_added = false;
				if (priv->primary_vif) {
					priv->primary_vif->ap_started = false;
					if (priv->primary_vif->ndev)
						netif_carrier_off(priv->primary_vif->ndev);
				}
				if (priv->man_pending) {
					priv->man_pending = false;
					complete(&priv->man_cfm);
				}
			} else if (ind_id != 0x332a) {
				dev_dbg(dev, "MAN_IND rx: idx=%u msg_id=0x%04x p0=0x%08x p1=0x%08x\n",
					idx, ind_id, p0, p1);
			}
			/* Return indication descriptor to firmware */
			wav500_ipc_ring_push(priv,
					     WAV500_RING_TYPE_MAN_IND |
					     ((u32)(idx & 0x3ff) << 5));
		} else {
			dev_dbg(dev, "FW ring msg: type=%u idx=%u raw=0x%08x\n",
				type, idx, entry);
		}
	}
}

int wav500_ipc_send_man_req(struct wav500_priv *priv, u16 msg_id, u8 vap_idx,
			    const void *req, size_t req_len,
			    void *rsp, size_t rsp_len, unsigned int timeout_ms)
{
	struct wav500_chi *chi = &priv->chi;
	u8 aligned_buf[WAV500_MAN_REQ_PAYLOAD_MAX];
	u32 slot_off, ring_word;
	u16 slot_idx = 0;
	unsigned long deadline;
	int ret = 0;

	if (!priv->fw_ready || !chi->man_req_bdr_cnt)
		return -ENODEV;
	if (req_len > WAV500_MAN_REQ_PAYLOAD_MAX ||
	    rsp_len > WAV500_MAN_REQ_PAYLOAD_MAX)
		return -EINVAL;

	mutex_lock(&priv->man_lock);

	slot_off = chi->man_req_bdr_off + slot_idx * WAV500_MAN_REQ_SLOT_SIZE;

	/* Write 16-byte BDR header (msg_id in word 1 bits [31:16]) */
	wav500_w32(priv, slot_off + 0x00, 0);
	wav500_w32(priv, slot_off + 0x04, ((u32)msg_id) << 16);
	wav500_w32(priv, slot_off + 0x08, 0);
	wav500_w32(priv, slot_off + 0x0c, 0);

	/* Write 32-bit aligned LE payload */
	memset(aligned_buf, 0, sizeof(aligned_buf));
	if (req && req_len)
		memcpy(aligned_buf, req, req_len);
	wav500_shram_write_bytes(priv, slot_off + WAV500_MAN_REQ_HDR_SIZE,
				 aligned_buf, ALIGN(max_t(size_t, req_len, 4), 4));

	reinit_completion(&priv->man_cfm);
	priv->man_pending_idx = slot_idx;
	priv->man_pending = true;

	ring_word = WAV500_RING_TYPE_MAN_REQ |
		    ((u32)(slot_idx & 0x3ff) << 5) |
		    ((u32)(vap_idx & 0xf) << 15);
	wav500_ipc_ring_push(priv, ring_word);

	/* Wait for IRQ completion, with short polling fallback for bring-up */
	deadline = jiffies + msecs_to_jiffies(timeout_ms);
	while (priv->man_pending && time_before(jiffies, deadline)) {
		if (wait_for_completion_timeout(&priv->man_cfm,
						msecs_to_jiffies(20)))
			break;
		wav500_ipc_poll_rx(priv);
	}

	if (priv->man_pending) {
		priv->man_pending = false;
		ret = -ETIMEDOUT;
	} else if (rsp && rsp_len) {
		memcpy(rsp, priv->man_cfm_buf, rsp_len);
	}

	mutex_unlock(&priv->man_lock);
	return ret;
}

int wav500_umi_get_mib(struct wav500_priv *priv, u16 mib_id,
		       u16 *status_out, u8 *val_out, size_t val_len)
{
	u8 req[0x54] = { };
	u8 rsp[0x54] = { };
	u16 st;
	int ret;

	req[0] = mib_id & 0xff;
	req[1] = (mib_id >> 8) & 0xff;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_GET_MIB_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 2000);
	if (ret)
		return ret;

	st = rsp[2] | ((u16)rsp[3] << 8);
	if (status_out)
		*status_out = st;
	if (val_out && val_len)
		memcpy(val_out, rsp + 4, min_t(size_t, val_len, 0x50));
	return 0;
}

int wav500_load_progmodel(struct wav500_priv *priv, const char *filename)
{
	struct device *dev = &priv->pdev->dev;
	const struct firmware *fw;
	size_t pos = 0;
	u32 pm_ver;
	int ret;

	ret = request_firmware(&fw, filename, dev);
	if (ret) {
		dev_err(dev, "missing ProgModel '%s' (%d)\n", filename, ret);
		return ret;
	}
	if (!fw->size || (fw->size & 3)) {
		dev_err(dev, "invalid ProgModel size %zu for '%s'\n",
			fw->size, filename);
		release_firmware(fw);
		return -EINVAL;
	}

	/* Step 1: Prepare firmware for ProgModel download (UMI 0x041b) */
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_PREPARE_PM_REQ, 0,
				      NULL, 0, NULL, 0, 5000);
	if (ret) {
		dev_err(dev, "UMI PREPARE_PM (0x041b) failed: %d\n", ret);
		release_firmware(fw);
		return ret;
	}

	/* Step 2: Stream ProgModel in <=8000-byte chunks to SHRAM+0x14 */
	while (pos < fw->size) {
		size_t chunk = min_t(size_t, fw->size - pos, WAV500_PM_CHUNK_MAX);

		wav500_shram_write_bytes(priv, WAV500_PM_SHRAM_OFF,
					 fw->data + pos, chunk);
		/* Trailing 12-byte zero terminator expected by FW */
		wav500_w32(priv, WAV500_PM_SHRAM_OFF + chunk + 0, 0);
		wav500_w32(priv, WAV500_PM_SHRAM_OFF + chunk + 4, 0);
		wav500_w32(priv, WAV500_PM_SHRAM_OFF + chunk + 8, 0);
		wmb();

		ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_DOWNLOAD_PM_REQ, 0,
					      NULL, 0, NULL, 0, 5000);
		if (ret) {
			dev_err(dev, "UMI DOWNLOAD_PM (0x0402) failed at %zu/%zu: %d\n",
				pos, fw->size, ret);
			release_firmware(fw);
			return ret;
		}
		pos += chunk;
	}

	release_firmware(fw);

	pm_ver = wav500_r32(priv, WAV500_PROGMODEL_VER);
	dev_info(dev, "ProgModel '%s' (%zu B) LOADED! hw_ver=%u (raw=0x%08x)\n",
		 filename, pos, pm_ver & 0xffff, pm_ver);
	return 0;
}

int wav500_load_cal(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	const char *cal_name = priv->radio_idx == 0 ?
		"wav500/cal_wlan0.bin" : "wav500/cal_wlan1.bin";
	const struct firmware *fw;
	size_t off;
	int ret;

	priv->xtal_val = 0x8c;
	priv->rssi_cal_len = 0;
	priv->tpc_struct_size = 0;
	priv->tpc_count = 0;

	ret = request_firmware(&fw, cal_name, dev);
	if (ret) {
		dev_warn(dev, "calibration '%s' not found (%d), keeping current MAC %pM\n",
			 cal_name, ret, priv->mac_addr);
		return ret;
	}

	/* Walk v5 CIS TLV list starting at offset 0x29 */
	for (off = 0x29; off + 2 <= fw->size; ) {
		u8 tag = fw->data[off];
		u8 len = fw->data[off + 1];
		const u8 *val = &fw->data[off + 2];

		if (tag == 0xff || tag == 0x00 || off + 2 + len > fw->size)
			break;

		if (tag == 0x80 && len >= 13) {
			priv->hw_type = val[0];
			priv->hw_rev = val[1];
			if (is_valid_ether_addr(&val[5])) {
				struct net_device *ndev = priv->primary_vif ?
					priv->primary_vif->ndev : NULL;

				/*
				 * Only adopt calibration MAC when the interface is
				 * not yet UP; changing dev_addr mid-start_ap breaks
				 * hostapd's SA check in cfg80211_mlme_mgmt_tx().
				 */
				if (ndev && netif_running(ndev)) {
					ether_addr_copy(priv->mac_addr, ndev->dev_addr);
				} else {
					ether_addr_copy(priv->mac_addr, &val[5]);
					ether_addr_copy(priv->wiphy->perm_addr,
							priv->mac_addr);
					if (ndev) {
						bool locked = rtnl_is_locked();

						if (!locked)
							rtnl_lock();
						eth_hw_addr_set(ndev, priv->mac_addr);
						if (!locked)
							rtnl_unlock();
					}
				}
			}
		} else if (tag == 0x84 && len >= 2) {
			priv->xtal_val = get_unaligned_le16(val);
		} else if (tag == 0x8b && len <= sizeof(priv->rssi_cal)) {
			memcpy(priv->rssi_cal, val, len);
			priv->rssi_cal_len = len;
		} else if ((tag == 0x8a || tag == 0x87) && len > 1) {
			u8 ss = val[0];
			size_t p = 1;

			if (ss >= 7 && ss <= 20) {
				priv->tpc_struct_size = ss;
				while (p + 1 + 4 * ss <= len) {
					u8 ch = val[p++];
					int a;

					for (a = 0; a < 4; a++) {
						if (priv->tpc_count < ARRAY_SIZE(priv->tpc_entries)) {
							int idx = priv->tpc_count++;

							priv->tpc_entries[idx].chan = ch;
							priv->tpc_entries[idx].ant = val[p] >> 6;
							priv->tpc_entries[idx].bw = val[p + 1] >> 6;
							memcpy(priv->tpc_entries[idx].raw, &val[p], ss);
						}
						p += ss;
					}
				}
			}
		}
		off += 2 + len;
	}

	dev_info(dev, "loaded '%s' (%zu B): hw_type=0x%02x('%c') hw_rev=0x%02x('%c') xtal=0x%04x rssi=%uB tpc=%u(ss=%u) MAC=%pM\n",
		 cal_name, fw->size, priv->hw_type, priv->hw_type,
		 priv->hw_rev, priv->hw_rev, priv->xtal_val,
		 priv->rssi_cal_len, priv->tpc_count, priv->tpc_struct_size,
		 priv->mac_addr);

	release_firmware(fw);
	return 0;
}

int wav500_umi_send_rssi_cal(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[132] = { };
	int ret;

	if (!priv->rssi_cal_len)
		return 0;

	req[0] = priv->rssi_cal_len;
	memcpy(&req[1], priv->rssi_cal, priv->rssi_cal_len);
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_RSSI_CAL_REQ, 0,
				      req, 1 + priv->rssi_cal_len,
				      NULL, 0, 5000);
	if (ret) {
		dev_err(dev, "UMI SET_RSSI_CAL (0x045b) failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "UMI SET_RSSI_CAL (0x045b, %u B) cfm: ok\n",
		 priv->rssi_cal_len);
	return 0;
}

int wav500_load_psdb(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	const struct firmware *fw;
	const u8 *d;
	size_t off, blk_off = 0;
	u32 nrows = 0, ncols = 0, r;
	u8 target_type = priv->hw_type ? priv->hw_type : (priv->is_5ghz ? 'P' : 'V');
	u8 target_rev = priv->hw_rev ? priv->hw_rev : (priv->is_5ghz ? 'S' : 'C');
	int tables_loaded = 0, ret;

	ret = request_firmware(&fw, "wav500/PSD.bin", dev);
	if (ret) {
		dev_warn(dev, "PSD.bin not found (%d)\n", ret);
		return ret;
	}
	d = fw->data;

	/* Scan 0xaaaa5555 platform blocks to find our (chip_id, hw_type, hw_rev) */
	for (off = 0; off + 20 < fw->size; off += 4) {
		u32 marker = get_unaligned_le32(d + off);
		u32 cid = 0, ht = 0, hr = 0;

		if (marker == 0x66669999)
			break;
		if (marker != 0xaaaa5555)
			continue;

		nrows = get_unaligned_le32(d + off + 4);
		ncols = get_unaligned_le32(d + off + 8);
		if (ncols != 2 || nrows == 0 || nrows > 120 ||
		    off + 12 + nrows * 8 > fw->size)
			continue;

		for (r = 0; r < nrows; r++) {
			u32 k = get_unaligned_le32(d + off + 12 + r * 8);
			u32 v = get_unaligned_le32(d + off + 16 + r * 8);

			if (k == 1)
				cid = v;
			else if (k == 0x0b)
				ht = v;
			else if (k == 0x0c)
				hr = v;
		}
		if (cid == priv->chip_id && ht == target_type && hr == target_rev) {
			blk_off = off;
			break;
		}
	}

	if (!blk_off) {
		dev_err(dev, "PSD.bin: no block for chip=0x%04x type=0x%02x rev=0x%02x\n",
			priv->chip_id, target_type, target_rev);
		release_firmware(fw);
		return -ENOENT;
	}

	/* 1) Send PSDB fields array via UMI 0x0457 ([nrows, ncols, k0, v0, ...]) */
	if (8 + nrows * 8 <= WAV500_MAN_REQ_PAYLOAD_MAX) {
		ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_PSDB_FIELDS_REQ, 0,
					      d + blk_off + 4, 8 + nrows * 8,
					      NULL, 0, 5000);
		if (ret) {
			dev_err(dev, "UMI PSDB_FIELDS (0x0457) failed: %d\n", ret);
			release_firmware(fw);
			return ret;
		}
	}

	/* 2) Send every FW table referenced in the platform block via UMI 0x0458 */
	for (r = 0; r < nrows; r++) {
		u32 key = get_unaligned_le32(d + blk_off + 12 + r * 8);
		u32 toff = get_unaligned_le32(d + blk_off + 16 + r * 8);
		u32 tr, tc, twords;
		u8 req[4];

		if (key < 0x10000 || key == 0x00010000 ||
		    key == 0x00140000 || key == 0x00150000 ||
		    key == 0x00160000 || key == 0x00170000 || !toff)
			continue;
		if (toff + 12 > fw->size)
			continue;
		if (get_unaligned_le32(d + toff) != 0x66669999)
			continue;

		tr = get_unaligned_le32(d + toff + 4);
		tc = get_unaligned_le32(d + toff + 8);
		twords = 2 + tr * tc;
		if (toff + 4 + twords * 4 > fw->size || twords * 4 > 8000)
			continue;

		wav500_shram_write_bytes(priv, WAV500_PM_SHRAM_OFF,
					 d + toff + 4, twords * 4);
		put_unaligned_le32(key, req);
		ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_PSDB_TABLE_REQ, 0,
					      req, sizeof(req), NULL, 0, 5000);
		if (ret) {
			dev_warn(dev, "UMI PSDB_TABLE(0x%08x) failed: %d\n", key, ret);
			continue;
		}
		tables_loaded++;
	}

	release_firmware(fw);
	dev_info(dev, "PSD.bin loaded (blk@0x%zx type='%c' rev='%c'): fields=%u tables=%d\n",
		 blk_off, target_type, target_rev, nrows, tables_loaded);
	return 0;
}

int wav500_umi_data_path_init(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	const u32 ring_bytes = WAV500_DP_RING_SIZE * WAV500_DP_HD_SIZE;
	u8 req[0x48] = { };
	u8 rsp[0x48] = { };
	int ret;

	if (priv->dp_inited)
		return 0;

	/* SW internal rings mode (mode 0): TX in/out, RX in/out, BSS_TX, BSS_RX */
	put_unaligned_le32((u32)priv->tx_ring.hds_dma,     &req[0x00]);
	put_unaligned_le32(ring_bytes,                     &req[0x04]);
	put_unaligned_le32((u32)priv->tx_ring.hds_dma,     &req[0x08]);
	put_unaligned_le32(ring_bytes,                     &req[0x0c]);
	put_unaligned_le32((u32)priv->rx_ring.hds_dma,     &req[0x10]);
	put_unaligned_le32(ring_bytes,                     &req[0x14]);
	put_unaligned_le32((u32)priv->rx_ring.hds_dma,     &req[0x18]);
	put_unaligned_le32(ring_bytes,                     &req[0x1c]);
	put_unaligned_le32((u32)priv->bss_tx_ring.hds_dma, &req[0x20]);
	put_unaligned_le32(ring_bytes,                     &req[0x24]);
	put_unaligned_le32((u32)priv->bss_rx_ring.hds_dma, &req[0x28]);
	put_unaligned_le32(ring_bytes,                     &req[0x2c]);
	req[0x40] = 0; /* SW mode 0 */
	req[0x41] = 1;
	req[0x42] = 1;
	req[0x43] = priv->radio_idx;
	req[0x44] = 1; /* rtlog max_nof_streams >= 1 */

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_DATA_PATH_INIT, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI DATA_PATH_INIT (0x040e) failed: %d\n", ret);
		return ret;
	}
	priv->dp_inited = true;
	dev_info(dev, "UMI DATA_PATH_INIT (0x040e, dma=%pad) cfm: ok\n",
		 &priv->dp_rings_dma);
	return 0;
}

int wav500_umi_hw_dep_cfg(struct wav500_priv *priv, u32 xtal_val)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[12] = { };
	u8 rsp[12] = { };
	int ret;

	put_unaligned_le32(xtal_val, &req[0]);
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_HW_DEP_CFG_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI HW_DEP_CFG (0x041c) failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "UMI HW_DEP_CFG (0x041c, xtal=0x%02x) cfm: ok\n", xtal_val);
	return 0;
}

static int wav500_umi_set_mib_raw(struct wav500_priv *priv, u16 mib_id,
				  const u8 *val, size_t len)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x54] = { };
	u8 rsp[0x54] = { };
	u16 st;
	int ret;

	put_unaligned_le16(mib_id, &req[0]);
	if (val && len)
		memcpy(&req[4], val, min_t(size_t, len, 0x50));
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_MIB_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI SET_MIB(0x%04x) failed: %d\n", mib_id, ret);
		return ret;
	}
	st = get_unaligned_le16(&rsp[2]);
	dev_info(dev, "UMI SET_MIB(0x%04x) cfm: status=0x%04x\n", mib_id, st);
	return 0;
}

int wav500_umi_pre_activate(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	u8 mib_pre[0x50] = { };
	u8 mib_spec[4] = { 0, 0, 0, 0 };
	u8 req[16] = { };
	u8 rsp[16] = { };
	u16 st;
	int ret;

	if (priv->pre_activated)
		return 0;

	/* Set MIB_PRE_ACTIVATE (0x0109), spectrum mode (0x1202), and IEEE MAC (0x0040) */
	mib_pre[3] = priv->is_5ghz ? 8 : 5; /* 11anac / 11bgn */
	put_unaligned_le32(priv->is_5ghz ? 0x00000ff0 : 0x00000fff, &mib_pre[8]);
	put_unaligned_le32(priv->is_5ghz ? 0x00000150 : 0x0000000f, &mib_pre[12]);
	wav500_umi_set_mib_raw(priv, 0x0109, mib_pre, sizeof(mib_pre));
	wav500_umi_set_mib_raw(priv, 0x1202, mib_spec, 1);
	wav500_umi_set_mib_raw(priv, 0x0040, priv->mac_addr, ETH_ALEN);

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_PRE_ACTIVATE_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI PRE_ACTIVATE (0x0452) failed: %d\n", ret);
		return ret;
	}

	st = rsp[0] | ((u16)rsp[1] << 8);
	dev_info(dev, "UMI PRE_ACTIVATE (0x0452) cfm: status=0x%04x\n", st);
	if (st == 0)
		priv->pre_activated = true;
	return st ? -EIO : 0;
}

int wav500_umi_hdk_config(struct wav500_priv *priv, u8 ch_mode,
			  u32 offline_mask, u32 online_mask)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[24] = { };
	u8 rsp[24] = { };
	size_t cal_sz = priv->chi.cal_size ? priv->chi.cal_size : 131072;
	int ret;

	if (!priv->cal_buf) {
		priv->cal_alloc_len = ALIGN(cal_sz, PAGE_SIZE);
		priv->cal_buf = dma_alloc_coherent(dev, priv->cal_alloc_len,
						   &priv->cal_dma, GFP_KERNEL);
		if (!priv->cal_buf)
			return -ENOMEM;
		memset(priv->cal_buf, 0, priv->cal_alloc_len);
	}

	/* 0x00..0x03: host calibration DMA buffer physical address (LE) */
	put_unaligned_le32((u32)priv->cal_dma, &req[0]);
	/* 0x04: setChannelMode (0=FULL, 1=SKIP_AND_SET) */
	req[4] = ch_mode;
	/* 0x08: max TX antennas (4), 0x09: max RX antennas (4) */
	req[8] = 4;
	req[9] = 4;
	/* 0x0a..0x0b: EEPROM version (5), 0x0c..0x0d: TPC points */
	req[10] = 5;
	req[11] = 0;
	req[12] = 2;
	req[13] = 2;
	/* 0x0e: band (0 = 5GHz, 1 = 2.4GHz) */
	req[14] = priv->is_5ghz ? 0 : 1;
	put_unaligned_le32(offline_mask, &req[16]);
	put_unaligned_le32(online_mask, &req[20]);

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_HDK_CONFIG_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI HDK_CONFIG (0x0450) failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "UMI HDK_CONFIG (0x0450, mode=%u band=%u off=0x%x on=0x%x cal_dma=%pad) cfm: status=%02x %02x\n",
		 ch_mode, req[14], offline_mask, online_mask,
		 &priv->cal_dma, rsp[0], rsp[1]);
	return 0;
}

int wav500_umi_set_radio_mode(struct wav500_priv *priv, bool enable)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[4] = { enable ? 1 : 0, 0, 0, 0 };
	u8 rsp[4] = { };
	int ret;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_RADIO_MODE_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI RADIO_MODE (0x045c, en=%d) failed: %d\n",
			enable, ret);
		return ret;
	}

	dev_info(dev, "UMI RADIO_MODE (0x045c, en=%d) cfm: val=%02x %02x\n",
		 enable, rsp[0], rsp[1]);
	return 0;
}

int wav500_umi_vap_db_op(struct wav500_priv *priv, u8 vap_id, bool add)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[4] = { add ? 1 : 0, vap_id, 0, 0 };
	u8 rsp[4] = { };
	u16 st;
	int ret;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_VAP_DB_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI VAP_DB_%s(vap=%u) failed: %d\n",
			add ? "ADD" : "DEL", vap_id, ret);
		return ret;
	}

	st = rsp[2] | ((u16)rsp[3] << 8);
	dev_info(dev, "UMI VAP_DB_%s(vap=%u) cfm: status=0x%04x\n",
		 add ? "ADD" : "DEL", vap_id, st);
	if (st == 0)
		priv->vap_added = add;
	return st ? -EIO : 0;
}

int wav500_umi_calibrate(struct wav500_priv *priv, u8 ch, u8 width)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x2c] = { };
	u8 rsp[0x2c] = { };
	u32 status;
	int ret;

	req[8] = ch;
	req[0x28] = 1; /* num_chans = 1 */
	req[0x29] = width;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_CALIBRATE_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 20000);
	if (ret) {
		dev_err(dev, "UMI CALIBRATE(ch=%u w=%u) failed: %d\n",
			ch, width, ret);
		return ret;
	}
	status = get_unaligned_le32(&rsp[4]);
	dev_info(dev, "UMI CALIBRATE(ch=%u w=%u) cfm: status=0x%08x\n",
		 ch, width, status);
	return status ? -EIO : 0;
}

int wav500_umi_set_chan(struct wav500_priv *priv, u32 freq, u8 width,
			u32 center_freq)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[20] = { };
	u8 rsp[20] = { };
	int low_ch, pri_ch;
	u32 bw_half;
	int ret;

	if (!center_freq)
		center_freq = freq;

	bw_half = (width == 4) ? 40 : ((20U << (width & 0x1f)) >> 1);
	low_ch = ieee80211_frequency_to_channel((center_freq + 10) - bw_half);
	pri_ch = ieee80211_frequency_to_channel(freq);

	req[0] = (u8)low_ch;
	req[2] = width;
	req[3] = (u8)((pri_ch - low_ch) >> 2);
	req[14] = 1; /* chan switch type: normal */

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_CHAN_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI SET_CHAN(freq=%u ch=%d w=%u) failed: %d\n",
			freq, pri_ch, width, ret);
		return ret;
	}

	priv->cur_freq = freq;
	dev_info(dev, "UMI SET_CHAN(freq=%u ch=%d low=%d w=%u) cfm: status=%02x %02x\n",
		 freq, pri_ch, low_ch, width, rsp[1], rsp[10]);
	return 0;
}

int wav500_umi_activate_ap(struct wav500_priv *priv, u8 vap_id,
			   const u8 *bssid, const u8 *ssid, size_t ssid_len,
			   u16 beacon_int)
{
	struct device *dev = &priv->pdev->dev;
	static const u8 basic_5g[] = { 0x0c, 0x18, 0x30 };       /* 6, 12, 24 Mbps */
	static const u8 basic_2g[] = { 0x02, 0x04, 0x0b, 0x16 }; /* 1, 2, 5.5, 11 Mbps */
	const u8 *rates = priv->is_5ghz ? basic_5g : basic_2g;
	size_t rlen = priv->is_5ghz ? sizeof(basic_5g) : sizeof(basic_2g);
	u8 req[64] = { };
	u8 rsp[64] = { };
	u16 st;
	int ret;

	ether_addr_copy(&req[0], bssid);
	put_unaligned_le16(beacon_int ? beacon_int : 100, &req[6]); /* beacon_period */
	put_unaligned_le16(2, &req[8]);                             /* dtim_period */
	req[10] = vap_id;
	req[11] = 0; /* AP mode */
	/* 0x0c..0x2b: basic rates in 500kbps units (no 0x80 bit), 0x2c: count */
	memcpy(&req[12], rates, rlen);
	req[44] = (u8)rlen;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_ACTIVATE_REQ, vap_id,
				      req, 63, rsp, 63, 5000);
	if (ret) {
		dev_err(dev, "UMI ACTIVATE_AP(vap=%u) failed: %d\n", vap_id, ret);
		return ret;
	}

	st = rsp[6] | ((u16)rsp[7] << 8);
	dev_info(dev, "UMI ACTIVATE_AP(vap=%u bssid=%pM rates=%zu) cfm: bcn_int=%u\n",
		 vap_id, bssid, rlen, st);
	return 0;
}

int wav500_umi_set_bss(struct wav500_priv *priv, u8 vap_id)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x38] = { };
	u8 rsp[0x38] = { };
	static const u8 basic_5g[] = { 0x0c, 0x18, 0x30 };
	static const u8 basic_2g[] = { 0x02, 0x04, 0x0b, 0x16 };
	int ret;

	req[0] = vap_id;
	req[1] = 0; /* protection_mode = none */
	req[2] = 1; /* short_slot = 1 */
	/* 0x03..0x22: basic rates in 500kbps units (no 0x80 bit), 0x23: count */
	if (priv->is_5ghz) {
		memcpy(&req[3], basic_5g, sizeof(basic_5g));
		req[0x23] = sizeof(basic_5g);
	} else {
		memcpy(&req[3], basic_2g, sizeof(basic_2g));
		req[0x23] = sizeof(basic_2g);
	}
	/* HT MCS 0..31 (4x4) */
	req[0x24] = 0xff;
	req[0x25] = 0xff;
	req[0x26] = 0xff;
	req[0x27] = 0xff;
	/* VHT MCS 0..9 on 4 streams */
	if (priv->is_5ghz) {
		req[0x2e] = 0xaa;
		req[0x2f] = 0xff;
	}
	req[0x37] = 1; /* short_preamble */

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_BSS_SET_BSS_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI SET_BSS(vap=%u) failed: %d\n", vap_id, ret);
		return ret;
	}
	dev_info(dev, "UMI SET_BSS(vap=%u) cfm: ok\n", vap_id);
	return 0;
}

int wav500_umi_set_tpc_config(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x18] = { };
	u8 rsp[0x18] = { };
	int i, ret;

	/* 10 power limit entries in 1/8 dBm (160 = 20.0 dBm) */
	for (i = 0; i < 10; i++)
		put_unaligned_le16(160, &req[i * 2]);
	req[20] = 0;    /* gen5b (0x0810) */
	req[21] = 1;    /* closed-loop TPC ON (PDB 0xe = 1) */
	req[22] = 0xff;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_TPC_CFG_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI SET_TPC_CFG (0x044f) failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "UMI SET_TPC_CFG (0x044f, 20dBm) cfm: ok\n");
	return 0;
}

int wav500_umi_set_antennas(struct wav500_priv *priv, u8 tx_mask, u8 rx_mask)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[4] = { tx_mask, rx_mask, 0, 0 };
	u8 rsp[4] = { };
	int ret;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_ANTENNAS_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_warn(dev, "UMI SET_ANTENNAS(0x%02x/0x%02x) failed: %d\n",
			 tx_mask, rx_mask, ret);
		return ret;
	}
	dev_info(dev, "UMI SET_ANTENNAS(tx=0x%02x rx=0x%02x) cfm: st=%u\n",
		 tx_mask, rx_mask, rsp[2]);
	return 0;
}

int wav500_umi_set_tx_power_offset(struct wav500_priv *priv, u8 offset)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[4] = { offset, 0, 0, 0 };
	u8 rsp[4] = { };
	int ret;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_TX_PWR_LIM, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_warn(dev, "UMI SET_TX_PWR_LIM(%u) failed: %d\n", offset, ret);
		return ret;
	}
	dev_info(dev, "UMI SET_TX_PWR_LIM(offset=%u) cfm: ok\n", offset);
	return 0;
}

int wav500_umi_set_tpc_gen5(struct wav500_priv *priv, u8 target_ch)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x94];
	u8 rsp[0x94];
	int ant, bw, i, ret, last_slots = 0;

	if (!priv->tpc_count || !priv->tpc_struct_size)
		return 0;

	for (ant = 0; ant < 4; ant++) {
		int slot = 0;

		memset(req, 0, sizeof(req));
		memset(rsp, 0, sizeof(rsp));
		req[0] = (u8)ant;
		req[1] = priv->tpc_struct_size;

		for (bw = 0; bw < 3; bw++) {
			int lo = -1, hi = -1;

			for (i = 0; i < priv->tpc_count; i++) {
				u8 c;

				if (priv->tpc_entries[i].ant != ant ||
				    priv->tpc_entries[i].bw != bw)
					continue;
				c = priv->tpc_entries[i].chan;
				if (c <= target_ch) {
					if (lo < 0 || c > priv->tpc_entries[lo].chan)
						lo = i;
				}
				if (c >= target_ch) {
					if (hi < 0 || c < priv->tpc_entries[hi].chan)
						hi = i;
				}
			}
			if (lo >= 0 && slot < 6) {
				u8 *dst = &req[4 + slot * 24];

				dst[0] = priv->tpc_entries[lo].chan;
				memcpy(&dst[1], priv->tpc_entries[lo].raw,
				       priv->tpc_struct_size);
				slot++;
			}
			if (hi >= 0 && hi != lo && slot < 6) {
				u8 *dst = &req[4 + slot * 24];

				dst[0] = priv->tpc_entries[hi].chan;
				memcpy(&dst[1], priv->tpc_entries[hi].raw,
				       priv->tpc_struct_size);
				slot++;
			}
		}
		last_slots = slot;

		ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_TPC_REQ, 0,
					      req, sizeof(req), rsp, sizeof(rsp), 5000);
		if (ret) {
			dev_err(dev, "UMI SET_TPC_GEN5(ant=%d) failed: %d\n", ant, ret);
			return ret;
		}
	}
	dev_info(dev, "UMI SET_TPC_GEN5(ch=%u ss=%u slots=%d ant=0..3) cfm: ok\n",
		 target_ch, priv->tpc_struct_size, last_slots);
	return 0;
}

int wav500_umi_set_tpc_default(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x94];
	u8 rsp[0x94];
	int ant, ret;

	for (ant = 0; ant < 4; ant++) {
		memset(req, 0, sizeof(req));
		memset(rsp, 0, sizeof(rsp));
		req[0] = (u8)ant;
		req[1] = 0x18; /* TPC frequency / valid flag */
		/* Moderate safe bring-up max power (120 = 15.0 dBm in 1/8 dBm units) */
		req[4] = 120;
		req[5] = 120;
		req[6] = 120;
		req[7] = 120;
		ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_TPC_REQ, 0,
					      req, sizeof(req), rsp, sizeof(rsp), 5000);
		if (ret) {
			dev_warn(dev, "UMI SET_TPC(ant=%d) failed: %d\n", ant, ret);
			return ret;
		}
	}
	dev_info(dev, "UMI SET_TPC(ant=0..3) cfm: ok\n");
	return 0;
}

int wav500_umi_set_beacon(struct wav500_priv *priv, u8 vap_id,
			  const u8 *head, size_t head_len,
			  const u8 *tail, size_t tail_len)
{
	struct device *dev = &priv->pdev->dev;
	size_t total = head_len + tail_len;
	u8 req[16] = { };
	u8 rsp[16] = { };
	int ret;

	if (!head || !head_len || total > 1024)
		return -EINVAL;

	if (!priv->beacon_buf) {
		priv->beacon_alloc_len = 1024;
		priv->beacon_buf = dma_alloc_coherent(dev, priv->beacon_alloc_len,
						      &priv->beacon_dma, GFP_KERNEL);
		if (!priv->beacon_buf)
			return -ENOMEM;
	}

	memcpy(priv->beacon_buf, head, head_len);
	if (tail && tail_len)
		memcpy((u8 *)priv->beacon_buf + head_len, tail, tail_len);
	wmb();

	{
		size_t after_tim1 = 0, after_tim2 = 0, after_tim3 = 0;
		const u8 *p = tail;
		const u8 *end = tail ? tail + tail_len : NULL;

		while (p && p + 2 <= end) {
			u8 eid = p[0];
			size_t elen = (size_t)p[1] + 2;

			if (p + elen > end)
				break;
			switch (eid) {
			case 7: case 8: case 9: case 32:
				after_tim1 += elen;
				break;
			case 35: case 40: case 41: case 42: case 48: case 50:
				after_tim2 += elen;
				break;
			default:
				after_tim3 += elen;
				break;
			}
			p += elen;
		}
		put_unaligned_le32((u32)priv->beacon_dma, &req[0]);
		put_unaligned_le16((u16)head_len, &req[4]);
		put_unaligned_le16((u16)after_tim1, &req[6]);
		put_unaligned_le16((u16)after_tim2, &req[8]);
		put_unaligned_le16((u16)after_tim3, &req[10]);
	}
	req[12] = vap_id;
	req[13] = 0;
	req[14] = 0;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_BEACON_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI SET_BEACON(vap=%u) failed: %d\n", vap_id, ret);
		return ret;
	}

	dev_dbg(dev, "UMI SET_BEACON(vap=%u dma=%pad len=%zu+%zu) cfm: status=%u\n",
		vap_id, &priv->beacon_dma, head_len, tail_len, rsp[13]);
	return rsp[13] ? -EIO : 0;
}

static void wav500_check_inbound_dma(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	dma_addr_t dma;
	u32 *buf;
	u8 req[8] = { };
	u8 rsp[8] = { };
	int ret;

	buf = dma_alloc_coherent(dev, 4096, &dma, GFP_KERNEL);
	if (!buf)
		return;
	buf[0] = 0xdeadbeef;
	buf[1] = 0xcafebabe;
	buf[2] = 0x12345678;
	buf[3] = 0x87654321;
	wmb();

	put_unaligned_le32((u32)dma, &req[0]);
	put_unaligned_le16(256, &req[4]);
	req[6] = 0;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_GET_STATS_TBL, 0,
				      req, 7, rsp, 7, 5000);
	rmb();
	dev_info(dev,
		 "INBOUND DMA TEST (0x042e dma=%pad ret=%d st=%u): buf[0..3]=%08x %08x %08x %08x\n",
		 &dma, ret, rsp[6], buf[0], buf[1], buf[2], buf[3]);
	dma_free_coherent(dev, 4096, buf, dma);
}

int wav500_umi_set_wmm_default(struct wav500_priv *priv, u8 vap_id)
{
	struct device *dev = &priv->pdev->dev;
	/* 0x22 bytes: [0..1] reserved/vap, then 4 x 8B AC records (BE, BK, VI, VO) */
	static const u8 ac_cwmin[4] = { 4, 4, 3, 2 };
	static const u8 ac_cwmax[4] = { 6, 10, 4, 3 };
	static const u8 ac_aifs[4]  = { 3, 7, 1, 1 };
	static const u16 ac_txop[4] = { 0, 0, 94, 47 };
	u8 req[0x22] = { };
	u8 rsp[0x22] = { };
	int i, ret;

	for (i = 0; i < 4; i++) {
		u8 *p = &req[2 + i * 8];

		put_unaligned_le16(ac_txop[i], &p[0]);
		put_unaligned_le16((1U << ac_cwmax[i]) - 1, &p[2]);
		p[4] = (1U << ac_cwmin[i]) - 1;
		p[5] = ac_aifs[i];
		p[6] = 0;
		p[7] = 0;
	}
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_WMM_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret)
		dev_warn(dev, "UMI SET_WMM (0x040a) failed: %d\n", ret);
	else
		dev_info(dev, "UMI SET_WMM (0x040a) cfm: ok\n");
	return ret;
}

static void wav500_check_beacon_stats(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	u16 sz = priv->chi.stats_data_size;
	size_t alloc_sz;
	dma_addr_t dma;
	u32 *buf, *snap;
	u8 req[8] = { };
	u8 rsp[8] = { };
	int i, diffs = 0, ret;

	if (!sz)
		return;
	alloc_sz = ALIGN(sz, PAGE_SIZE);
	buf = dma_alloc_coherent(dev, alloc_sz, &dma, GFP_KERNEL);
	snap = kmalloc(sz, GFP_KERNEL);
	if (!buf || !snap) {
		kfree(snap);
		if (buf)
			dma_free_coherent(dev, alloc_sz, buf, dma);
		return;
	}
	memset(buf, 0, sz);
	wmb();

	put_unaligned_le32((u32)dma, &req[0]);
	put_unaligned_le16(sz, &req[4]);
	req[6] = 0;
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_GET_STATS_DATA, 0,
				      req, 7, rsp, 7, 5000);
	rmb();
	memcpy(snap, buf, sz);

	msleep(600);

	req[6] = 0;
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_GET_STATS_DATA, 0,
				      req, 7, rsp, 7, 5000);
	rmb();
	for (i = 0; i < sz / 4 && diffs < 16; i++) {
		u32 v0 = le32_to_cpu((__le32)snap[i]);
		u32 v1 = le32_to_cpu((__le32)buf[i]);

		if (v0 != v1) {
			dev_info(dev, "STATS_DELTA off=0x%04x: %u -> %u (+%u)\n",
				 i * 4, v0, v1, v1 - v0);
			diffs++;
		}
	}
	if (!diffs)
		dev_info(dev, "STATS_DELTA: no counters changed in 600ms (sz=%u ret=%d st=%u)\n",
			 sz, ret, rsp[6]);

	/* Also fetch 0x042e stats schema table to dump live 1-byte RF/TX-power fields */
	if (priv->chi.stats_table_size && priv->chi.stats_table_size <= 4096) {
		dma_addr_t tdma;
		u8 *tbuf = dma_alloc_coherent(dev, 4096, &tdma, GFP_KERNEL);

		if (tbuf) {
			u8 treq[8] = { }, trsp[8] = { };
			u8 *db = (u8 *)buf;
			int nent = priv->chi.stats_table_size / 8;
			int printed = 0;

			memset(tbuf, 0, priv->chi.stats_table_size);
			wmb();
			put_unaligned_le32((u32)tdma, &treq[0]);
			put_unaligned_le16(priv->chi.stats_table_size, &treq[4]);
			if (!wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_GET_STATS_TBL, 0,
						     treq, 7, trsp, 7, 5000)) {
				rmb();
				for (i = 0; i < nent && printed < 20; i++) {
					u16 w0 = (u16)(tbuf[i * 8] << 8) | tbuf[i * 8 + 1];
					u16 off = (u16)(tbuf[i * 8 + 4] << 8) | tbuf[i * 8 + 5];
					u8 cnt = tbuf[i * 8 + 6];
					u8 usz = (w0 & 0x7fff) >> 12;
					u8 typ = (w0 & 0x0fff) >> 8;

					if (usz == 1 && off + 4 <= sz &&
					    (cnt == 4 || cnt == 1 || cnt == 0)) {
						dev_info(dev,
							 "RF_STAT[%d] typ=%u cnt=%u off=0x%04x: %02x %02x %02x %02x\n",
							 i, typ, cnt, off,
							 db[off], db[off + 1],
							 db[off + 2], db[off + 3]);
						printed++;
					}
				}
			}
			dma_free_coherent(dev, 4096, tbuf, tdma);
		}
	}

	kfree(snap);
	dma_free_coherent(dev, alloc_sz, buf, dma);
}

int wav500_start_test_ap(struct wav500_priv *priv, int mode)
{
	const char *ssid = priv->is_5ghz ? "OpenWrt-BlueCave-5G" : "OpenWrt-BlueCave-2G";
	size_t slen = strlen(ssid);
	u32 freq = priv->is_5ghz ? 5180 : 2437;
	u8 ch = priv->is_5ghz ? 36 : 6;
	u8 bcn[128] = { };
	size_t pos = 0;
	int ret;

	if (!priv->fw_ready)
		return -ENODEV;

	wav500_check_inbound_dma(priv);

	ret = wav500_umi_pre_activate(priv);
	if (ret)
		return ret;

	if (mode == 0)
		ret = wav500_umi_hdk_config(priv, 1, 0, 0);
	else if (mode == 1)
		ret = wav500_umi_hdk_config(priv, 0, 0x000003ff, 0x000000ff);
	else
		ret = wav500_umi_hdk_config(priv, 0, 0, 0);
	if (ret)
		return ret;

	if (!priv->vap_added) {
		ret = wav500_umi_vap_db_op(priv, 0, true);
		if (ret)
			return ret;
	}

	ret = wav500_umi_activate_ap(priv, 0, priv->mac_addr,
				     ssid, slen, 100);
	if (ret)
		return ret;

	if (mode != 2) {
		ret = wav500_umi_set_radio_mode(priv, true);
		if (ret)
			return ret;
		msleep(200);
	}

	if (mode == 1) {
		u32 *cb;
		size_t nz = 0, j;

		ret = wav500_umi_calibrate(priv, ch, 0);
		if (ret)
			return ret;
		rmb();
		cb = priv->cal_buf;
		if (cb) {
			for (j = 0; j < priv->cal_alloc_len / 4; j++)
				if (cb[j])
					nz++;
			dev_info(&priv->pdev->dev,
				 "cal_buf: nonzero_words=%zu/%zu\n",
				 nz, priv->cal_alloc_len / 4);
		}
	}

	wav500_umi_set_antennas(priv, 0x0f, 0x0f);
	wav500_umi_set_tx_power_offset(priv, 0);
	wav500_umi_set_tpc_config(priv);
	wav500_umi_set_tpc_gen5(priv, ch);

	ret = wav500_umi_set_chan(priv, freq, 0, freq);
	if (ret)
		return ret;

	if (mode == 2) {
		ret = wav500_umi_set_radio_mode(priv, true);
		if (ret)
			return ret;
	}

	/* Minimal 802.11 Beacon frame header (24B MAC + 12B fixed) */
	bcn[0] = 0x80; bcn[1] = 0x00;			/* Frame Control: Beacon */
	memset(&bcn[4], 0xff, 6);			/* DA: Broadcast */
	ether_addr_copy(&bcn[10], priv->mac_addr);	/* SA */
	ether_addr_copy(&bcn[16], priv->mac_addr);	/* BSSID */
	/* Fixed fields: timestamp(8B=0), beacon_int(100 TU), cap(ESS) */
	bcn[32] = 100; bcn[33] = 0;
	bcn[34] = 0x01; bcn[35] = 0x04;			/* ESS | Short Slot */
	pos = 36;

	/* SSID IE */
	bcn[pos++] = 0;
	bcn[pos++] = slen;
	memcpy(&bcn[pos], ssid, slen);
	pos += slen;

	/* Supported Rates IE */
	bcn[pos++] = 1;
	if (priv->is_5ghz) {
		static const u8 rates_5g[] = { 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c };

		bcn[pos++] = sizeof(rates_5g);
		memcpy(&bcn[pos], rates_5g, sizeof(rates_5g));
		pos += sizeof(rates_5g);
	} else {
		static const u8 rates_2g[] = { 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24 };

		bcn[pos++] = sizeof(rates_2g);
		memcpy(&bcn[pos], rates_2g, sizeof(rates_2g));
		pos += sizeof(rates_2g);
	}

	/* DS Parameter Set IE (Channel) */
	bcn[pos++] = 3;
	bcn[pos++] = 1;
	bcn[pos++] = ch;

	{
		/* Tail IEs after TIM: ERP (2.4G) + WMM Parameter Element */
		static const u8 wmm_ie[] = {
			0xdd, 0x18, 0x00, 0x50, 0xf2, 0x02, 0x01, 0x01,
			0x80, 0x00,
			0x03, 0xa4, 0x00, 0x00,
			0x27, 0xa4, 0x00, 0x00,
			0x42, 0x43, 0x5e, 0x00,
			0x62, 0x32, 0x2f, 0x00,
		};
		u8 tail[40] = { };
		size_t tpos = 0;

		if (!priv->is_5ghz) {
			/* ERP Information IE (id=42, len=1, val=0) */
			tail[tpos++] = 42;
			tail[tpos++] = 1;
			tail[tpos++] = 0x00;
		}
		memcpy(&tail[tpos], wmm_ie, sizeof(wmm_ie));
		tpos += sizeof(wmm_ie);

		ret = wav500_umi_set_beacon(priv, 0, bcn, pos, tail, tpos);
	}
	if (ret)
		return ret;

	wav500_umi_set_wmm_default(priv, 0);

	ret = wav500_umi_set_bss(priv, 0);
	if (!ret) {
		wav500_dma_enable_irqs(priv);
		wav500_check_beacon_stats(priv);
	}
	return ret;
}

int wav500_umi_request_sid(struct wav500_priv *priv, const u8 *mac, u16 *sid_out)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[12] = { };
	u8 rsp[12] = { };
	int ret;

	ether_addr_copy(&req[0], mac);
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_REQUEST_SID_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret)
		return ret;
	if (rsp[8]) {
		dev_err(dev, "UMI REQUEST_SID(%pM) status=%u\n", mac, rsp[8]);
		return -EIO;
	}
	*sid_out = get_unaligned_le16(&rsp[6]);
	dev_info(dev, "UMI REQUEST_SID(%pM) -> sid=%u\n", mac, *sid_out);
	return 0;
}

int wav500_umi_remove_sid(struct wav500_priv *priv, u16 sid)
{
	u8 req[4] = { };
	u8 rsp[4] = { };
	int ret;

	put_unaligned_le16(sid, &req[0]);
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_REMOVE_SID_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret)
		return ret;
	return rsp[2] ? -EIO : 0;
}

int wav500_umi_add_sta(struct wav500_priv *priv, u8 vap_id, u16 sid,
		       const u8 *mac, struct station_parameters *params)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x54] = { };
	u8 rsp[0x54] = { };
	u8 flags = 0x06; /* bit 1 = WME, bit 2 = Authorized (bit 0 is WDS, must be 0!) */
	int ret;

	if (params && (params->sta_flags_set & BIT(NL80211_STA_FLAG_MFP)))
		flags |= 0x08; /* bit 3 = MFP */

	put_unaligned_le16(sid, &req[0]);
	req[2] = vap_id;
	req[4] = 0; /* band */
	req[5] = 0;
	if (params && params->aid)
		put_unaligned_le16(params->aid, &req[8]);
	else
		put_unaligned_le16(sid + 1, &req[8]);

	ether_addr_copy(&req[10], mac);

	/* Supported rates (strip 0x80 basic bit) */
	if (params && params->link_sta_params.supported_rates &&
	    params->link_sta_params.supported_rates_len) {
		size_t n = min_t(size_t, params->link_sta_params.supported_rates_len, 32);
		size_t i;

		for (i = 0; i < n; i++)
			req[16 + i] = params->link_sta_params.supported_rates[i] & 0x7f;
		req[0x3d] = (u8)n;
	} else if (priv->is_5ghz) {
		static const u8 r5[] = { 12, 18, 24, 36, 48, 72, 96, 108 };

		memcpy(&req[16], r5, sizeof(r5));
		req[0x3d] = sizeof(r5);
	} else {
		static const u8 r2[] = { 2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108 };

		memcpy(&req[16], r2, sizeof(r2));
		req[0x3d] = sizeof(r2);
	}

	if (params && params->link_sta_params.ht_capa) {
		const struct ieee80211_ht_cap *ht = params->link_sta_params.ht_capa;

		flags |= 0x10; /* bit 4 = HT */
		put_unaligned_le16(le16_to_cpu(ht->cap_info), &req[0x30]);
		req[0x32] = ht->ampdu_params_info;
		memcpy(&req[0x33], ht->mcs.rx_mask, 10);
	}
	if (params && params->link_sta_params.vht_capa) {
		const struct ieee80211_vht_cap *vht = params->link_sta_params.vht_capa;

		flags |= 0x20; /* bit 5 = VHT */
		put_unaligned_le32(le32_to_cpu(vht->vht_cap_info), &req[0x40]);
		memcpy(&req[0x44], &vht->supp_mcs, 8);
	}
	req[0x3e] = flags;
	req[0x3f] = 0;

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_STA_ADD_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI STA_ADD(sid=%u %pM) failed: %d\n", sid, mac, ret);
		return ret;
	}
	dev_info(dev, "UMI STA_ADD(vap=%u sid=%u %pM flags=0x%02x) cfm: st=%u\n",
		 vap_id, sid, mac, flags, rsp[3]);
	return rsp[3] ? -EIO : 0;
}

int wav500_umi_remove_sta(struct wav500_priv *priv, u16 sid)
{
	u8 req[4] = { };
	u8 rsp[4] = { };
	int ret;

	put_unaligned_le16(sid, &req[0]);
	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_STA_REMOVE_REQ, 0,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret)
		return ret;
	return rsp[2] ? -EIO : 0;
}

int wav500_umi_set_key(struct wav500_priv *priv, u8 vap_id, u16 sid,
		       u8 key_idx, bool pairwise, struct key_params *params)
{
	struct device *dev = &priv->pdev->dev;
	u8 req[0x3a] = { };
	u8 rsp[0x3a] = { };
	u16 cipher_id;
	u16 use_sid;
	int ret;

	if (!params || !params->key || !params->key_len)
		return 0;

	/* WAV500 UMI cipher IDs: 0=TKIP, 1=WEP40, 2=CCMP, 4=BIP(CMAC), 5=WEP104 */
	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_TKIP:
		cipher_id = 0;
		break;
	case WLAN_CIPHER_SUITE_WEP40:
		cipher_id = 1;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		cipher_id = 2;
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		cipher_id = 4;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		cipher_id = 5;
		break;
	default:
		return 0;
	}

	/* [0..1] status, [2..3] sid, [4..5] key_type, [6..7] cipher,
	 * [8..9] key_idx, [10..17] rx_seq, [18..25] tx_seq, [26..57] key (32B)
	 */
	use_sid = pairwise ? sid : 0x3ffa;
	put_unaligned_le16(use_sid, &req[2]);
	put_unaligned_le16(pairwise ? 0 : 1, &req[4]);
	put_unaligned_le16(cipher_id, &req[6]);
	put_unaligned_le16(key_idx, &req[8]);
	if (params->seq && params->seq_len)
		memcpy(&req[10], params->seq, min_t(size_t, params->seq_len, 8));
	req[18] = 1; /* initial TxSeqNum */
	memcpy(&req[26], params->key, min_t(size_t, params->key_len, 32));

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_KEY_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret) {
		dev_err(dev, "UMI SET_KEY(sid=0x%x idx=%u) failed: %d\n", use_sid, key_idx, ret);
		return ret;
	}
	{
		u16 st = get_unaligned_le16(&rsp[0]);

		dev_info(dev, "UMI SET_KEY(vap=%u sid=0x%x idx=%u cipher=%u pw=%d) cfm: st=%u\n",
			 vap_id, use_sid, key_idx, cipher_id, pairwise, st);
		return st ? -EIO : 0;
	}
}

int wav500_umi_del_key(struct wav500_priv *priv, u8 vap_id, u16 sid,
		       u8 key_idx, bool pairwise)
{
	u8 req[0x3a] = { };
	u8 rsp[0x3a] = { };
	u16 use_sid = pairwise ? sid : 0x3ffa;
	int ret;

	put_unaligned_le16(use_sid, &req[2]);
	put_unaligned_le16(pairwise ? 0 : 1, &req[4]);
	put_unaligned_le16(0, &req[6]); /* cipher = NONE (clear key) */
	put_unaligned_le16(key_idx, &req[8]);

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_KEY_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret)
		return ret;
	return get_unaligned_le16(&rsp[0]) ? -EIO : 0;
}

int wav500_umi_set_default_key(struct wav500_priv *priv, u8 vap_id, u16 sid,
			       bool is_group, u8 key_idx)
{
	u8 req[8] = { };
	u8 rsp[8] = { };
	int ret;

	put_unaligned_le16(is_group ? 0x3ffa : sid, &req[2]);
	req[4] = is_group ? 1 : 0;
	put_unaligned_le16(key_idx, &req[6]);

	ret = wav500_ipc_send_man_req(priv, WAV500_UMI_MAN_SET_DEF_KEY_REQ, vap_id,
				      req, sizeof(req), rsp, sizeof(rsp), 5000);
	if (ret)
		return ret;
	return get_unaligned_le16(&rsp[0]) ? -EIO : 0;
}
