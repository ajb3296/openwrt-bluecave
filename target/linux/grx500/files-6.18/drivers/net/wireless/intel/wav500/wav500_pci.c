// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel / Lantiq WAV500 (WAV514/WAV524, gen5b) clean-room cfg80211 driver.
 *
 * Stage 5-2 Step 2: PCIe probe, BAR0 MMIO, hardware reset/PLL bring-up,
 * runtime firmware loader (/lib/firmware/wav500/), and CHI_MAGIC handshake.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <net/cfg80211.h>

#include "wav500.h"

struct wav500_fw_region {
	const char *filename;
	u32 bar0_off;
	u32 max_size;
};

static const struct wav500_fw_region wav500_gen5b_wrx514_fw[] = {
	{ "wav500/ap_upper_gen5b_wrx_514.bin", WAV500_UMAC_IRAM_OFF, WAV500_UMAC_IRAM_MAX_SIZE },
	{ "wav500/ap_lower_gen5b_wrx_514.bin", WAV500_LMAC_IRAM_OFF, WAV500_LMAC_IRAM_MAX_SIZE },
	{ "wav500/tx_sender_gen5b.bin",        WAV500_TX_SENDER_OFF, WAV500_TX_SENDER_MAX_SIZE },
	{ "wav500/host_interface_gen5b.bin",   WAV500_HOST_IF_OFF,   WAV500_HOST_IF_MAX_SIZE },
	{ "wav500/rx_handler_gen5b.bin",       WAV500_RX_HANDLER_OFF, WAV500_RX_HANDLER_MAX_SIZE },
};

static const struct wav500_fw_region wav500_gen5b_wrx500_fw[] = {
	{ "wav500/ap_upper_gen5b_wrx_500.bin", WAV500_UMAC_IRAM_OFF, WAV500_UMAC_IRAM_MAX_SIZE },
	{ "wav500/ap_lower_gen5b_wrx_500.bin", WAV500_LMAC_IRAM_OFF, WAV500_LMAC_IRAM_MAX_SIZE },
	{ "wav500/tx_sender_gen5b.bin",        WAV500_TX_SENDER_OFF, WAV500_TX_SENDER_MAX_SIZE },
	{ "wav500/host_interface_gen5b.bin",   WAV500_HOST_IF_OFF,   WAV500_HOST_IF_MAX_SIZE },
	{ "wav500/rx_handler_gen5b.bin",       WAV500_RX_HANDLER_OFF, WAV500_RX_HANDLER_MAX_SIZE },
};

static void wav500_umac_sec_write(struct wav500_priv *priv, u32 reg, u32 val)
{
	wav500_w32(priv, WAV500_UMAC_SEC_UNLOCK, WAV500_SEC_UNLOCK_KEY1);
	wav500_w32(priv, WAV500_UMAC_SEC_UNLOCK, WAV500_SEC_UNLOCK_KEY2);
	wav500_w32(priv, reg, val);
	wmb();
}

static void wav500_lmac_sec_write(struct wav500_priv *priv, u32 reg, u32 val)
{
	wav500_w32(priv, WAV500_LMAC_SEC_UNLOCK, WAV500_SEC_UNLOCK_KEY1);
	wav500_w32(priv, WAV500_LMAC_SEC_UNLOCK, WAV500_SEC_UNLOCK_KEY2);
	wav500_w32(priv, reg, val);
	wmb();
}

static irqreturn_t wav500_irq_handler(int irq, void *data)
{
	struct wav500_priv *priv = data;
	u32 stat = wav500_r32(priv, WAV500_IRQ_STATUS);
	u32 mask = priv->dp_irq_enabled ? 0x2f : 0x20;

	if (!(stat & mask))
		return IRQ_NONE;

	priv->irq_count++;

	/* Acknowledge UMI/management host interrupt and drain FW->Host ring */
	if (stat & 0x20) {
		wav500_w32(priv, WAV500_IRQ_CLEAR, 1);
		wav500_ipc_poll_rx(priv);
	}

	/* Mask data-path ring IRQs until tasklet drains & clears them */
	if (priv->dp_irq_enabled && (stat & 0x0f)) {
		wav500_w32(priv, WAV500_IRQ_MASK, 0x20);
		tasklet_schedule(&priv->dp_tasklet);
	}
	return IRQ_HANDLED;
}

static void wav500_put_cpus_to_reset(struct wav500_priv *priv)
{
	wav500_lmac_sec_write(priv, WAV500_LMAC_CPU_BOOT_CTRL, 0);
	msleep(20);
	wav500_umac_sec_write(priv, WAV500_UMAC_CPU_BOOT_CTRL, 0);
	msleep(20);
}

static void wav500_release_cpus_from_reset(struct wav500_priv *priv)
{
	wav500_lmac_sec_write(priv, WAV500_LMAC_CPU_BOOT_CTRL, 0x10);
	msleep(10);
	wav500_umac_sec_write(priv, WAV500_UMAC_CPU_BOOT_CTRL, 0x10);
	msleep(10);
}

static void wav500_reset_and_release_ctl(struct wav500_priv *priv)
{
	u32 val;

	/* Hold CPUs in reset first */
	wav500_put_cpus_to_reset(priv);

	/* Clear misc bit 0 */
	val = wav500_r32(priv, WAV500_MAC_CTRL_MISC);
	wav500_w32(priv, WAV500_MAC_CTRL_MISC, val & ~BIT(0));

	/* Release MAC subsystem controllers from reset; 0x0810 is gen5b/B11 */
	wav500_w32(priv, WAV500_PAS_CTL_EN1, 0xffffffff);
	wav500_w32(priv, WAV500_PAS_CTL_EN0, 0xffffffff);
	wav500_umac_sec_write(priv, WAV500_UMAC_B11_CTRL, 1);
}

static int wav500_init_pll(struct wav500_priv *priv)
{
	u32 val;
	int i;

	val = wav500_r32(priv, WAV500_UMAC_PLL_CTRL);
	wav500_umac_sec_write(priv, WAV500_UMAC_PLL_CTRL, val | 0x80);
	msleep(50);

	val = wav500_r32(priv, WAV500_UMAC_PLL_CTRL);
	wav500_umac_sec_write(priv, WAV500_UMAC_PLL_CTRL, val | 0x180);
	msleep(1);

	for (i = 0; i < 100; i++) {
		if (wav500_r32(priv, WAV500_UMAC_PLL_STAT) & BIT(0))
			break;
		msleep(1);
	}
	if (i == 100) {
		dev_err(&priv->pdev->dev, "PLL lock timeout (stat=0x%08x)\n",
			wav500_r32(priv, WAV500_UMAC_PLL_STAT));
		return -ETIMEDOUT;
	}

	wav500_umac_sec_write(priv, WAV500_UMAC_PLL_CFG0, 0x104);
	wav500_umac_sec_write(priv, WAV500_UMAC_PLL_CFG1, 0x11);
	wav500_umac_sec_write(priv, WAV500_UMAC_CLK_CTRL, 2);
	dev_info(&priv->pdev->dev, "PLL locked and clocks configured\n");
	return 0;
}

static int wav500_load_fw_regions(struct wav500_priv *priv)
{
	struct device *dev = &priv->pdev->dev;
	const struct wav500_fw_region *table = priv->is_5ghz ?
		wav500_gen5b_wrx500_fw : wav500_gen5b_wrx514_fw;
	size_t count = priv->is_5ghz ?
		ARRAY_SIZE(wav500_gen5b_wrx500_fw) :
		ARRAY_SIZE(wav500_gen5b_wrx514_fw);
	size_t idx;
	int ret;

	for (idx = 0; idx < count; idx++) {
		const struct wav500_fw_region *r = &table[idx];
		const struct firmware *fw;
		const __le32 *src;
		u32 words, i;

		ret = request_firmware(&fw, r->filename, dev);
		if (ret) {
			dev_err(dev, "missing firmware '%s' (%d)\n",
				r->filename, ret);
			return ret;
		}
		if (fw->size == 0 || fw->size > r->max_size || (fw->size & 3)) {
			dev_err(dev, "invalid size %zu for '%s' (max %u)\n",
				fw->size, r->filename, r->max_size);
			release_firmware(fw);
			return -EINVAL;
		}

		src = (const __le32 *)fw->data;
		words = fw->size / 4;
		for (i = 0; i < words; i++)
			wav500_w32(priv, r->bar0_off + i * 4, le32_to_cpu(src[i]));
		wmb();

		dev_info(dev, "loaded '%s' (%zu B) -> BAR0+0x%06x\n",
			 r->filename, fw->size, r->bar0_off);
		release_firmware(fw);
	}
	return 0;
}

static int wav500_boot_firmware(struct wav500_priv *priv, bool load_psdb)
{
	struct device *dev = &priv->pdev->dev;
	u32 magic = 0, i;
	int ret;

	priv->fw_ready = false;
	priv->pre_activated = false;
	priv->vap_added = false;
	priv->dp_irq_enabled = false;
	priv->irq_count = 0;

	/* Free any previous DMA ring/buffer pools before re-initializing */
	wav500_dma_cleanup_rings(priv);
	priv->dp_inited = false;

	/* Mask interrupts during bring-up */
	wav500_w32(priv, WAV500_IRQ_MASK, 0);

	wav500_reset_and_release_ctl(priv);

	ret = wav500_init_pll(priv);
	if (ret)
		return ret;

	/* Clear CHI handshake region in SHRAM and write host efuse/boot size (0xf0) */
	for (i = 0; i < WAV500_CHI_BASE_SIZE; i += 4)
		wav500_w32(priv, WAV500_CHI_BASE_OFF + i, 0);
	wav500_w32(priv, WAV500_CHI_BASE_OFF, 0x000000f0);
	wmb();

	ret = wav500_load_fw_regions(priv);
	if (ret)
		return ret;

	/* Clear pending IRQs, enable PCIe IRQ routing, unmask, and release CPUs */
	wav500_w32(priv, WAV500_IRQ_MASK, 0);
	if (wav500_r32(priv, WAV500_IRQ_STATUS) & 0x20)
		wav500_w32(priv, WAV500_IRQ_CLEAR, 1);
	wav500_w32(priv, WAV500_PCIE_IRQ_EN0,
		   wav500_r32(priv, WAV500_PCIE_IRQ_EN0) | 0xff);
	wav500_w32(priv, WAV500_PCIE_IRQ_EN1,
		   wav500_r32(priv, WAV500_PCIE_IRQ_EN1) | 1);
	/* Enable UMI/management host interrupt only (data-path rings masked until wired) */
	wav500_w32(priv, WAV500_IRQ_MASK, 0x20);
	wmb();

	dev_info(dev, "pre-release: soc_rst=0x%08x chi_c8=0x%08x umac0=0x%08x lmac0=0x%08x\n",
		 wav500_r32(priv, WAV500_SOC_RESET_CTRL),
		 wav500_r32(priv, WAV500_CHI_MAGIC_OFF),
		 wav500_r32(priv, WAV500_UMAC_IRAM_OFF),
		 wav500_r32(priv, WAV500_LMAC_IRAM_OFF));

	wav500_release_cpus_from_reset(priv);

	/* Poll SHRAM CHI_MAGIC (0xbeadfeed) for up to 2 seconds */
	for (i = 0; i < 100; i++) {
		magic = wav500_r32(priv, WAV500_CHI_MAGIC_OFF);
		if (magic == WAV500_CHI_MAGIC ||
		    swab32(magic) == WAV500_CHI_MAGIC)
			break;
		msleep(20);
	}

	priv->chi_magic_seen = magic;
	for (i = 0; i < ARRAY_SIZE(priv->chi_base); i++)
		priv->chi_base[i] = wav500_r32(priv, WAV500_CHI_BASE_OFF + i * 4);

	if (magic == WAV500_CHI_MAGIC || swab32(magic) == WAV500_CHI_MAGIC) {
		u16 mib_st = 0xffff;
		u8 mib_val[8] = { };
		int ipc_ret;

		priv->fw_ready = true;
		dev_info(dev,
			 "FW BOOT SUCCESS! CHI_MAGIC=0x%08x irqs=%u irq_stat=0x%08x\n",
			 magic, priv->irq_count,
			 wav500_r32(priv, WAV500_IRQ_STATUS));

		ret = wav500_chi_parse(priv);
		if (ret)
			return ret;

		ipc_ret = wav500_umi_get_mib(priv, 0x0020, &mib_st,
					     mib_val, sizeof(mib_val));
		if (ipc_ret == 0)
			dev_info(dev,
				 "UMI GET_MIB(0x0020) SUCCESS! status=0x%04x val=%02x %02x %02x %02x irqs=%u cfms=%u inds=%u\n",
				 mib_st, mib_val[0], mib_val[1], mib_val[2], mib_val[3],
				 priv->irq_count, priv->man_cfm_count, priv->man_ind_count);
		else
			dev_warn(dev,
				 "UMI GET_MIB(0x0020) returned %d (irqs=%u cfms=%u inds=%u irq_stat=0x%08x)\n",
				 ipc_ret, priv->irq_count, priv->man_cfm_count,
				 priv->man_ind_count,
				 wav500_r32(priv, WAV500_IRQ_STATUS));

		/* Vendor order: DATA_PATH_INIT -> cal -> HW_DEP_CFG -> ProgModel -> RSSI_CAL -> PSDB */
		wav500_dma_init_rings(priv);
		wav500_load_cal(priv);
		wav500_umi_hw_dep_cfg(priv, priv->xtal_val ? priv->xtal_val : 0x8c);
		ret = wav500_load_progmodel(priv,
			priv->is_5ghz ?
			"wav500/ProgModel_gen5b_A.bin" :
			"wav500/ProgModel_gen5b_BG_wrx5xx.bin");
		if (ret)
			dev_warn(dev, "ProgModel load returned %d\n", ret);
		wav500_umi_send_rssi_cal(priv);
		if (load_psdb)
			wav500_load_psdb(priv);
		return 0;
	}

	dev_err(dev,
		"CHI_MAGIC timeout: got 0x%08x (irqs=%u, irq_stat=0x%08x, umac_boot=0x%08x, lmac_boot=0x%08x)\n",
		magic, priv->irq_count,
		wav500_r32(priv, WAV500_IRQ_STATUS),
		wav500_r32(priv, WAV500_UMAC_CPU_BOOT_CTRL),
		wav500_r32(priv, WAV500_LMAC_CPU_BOOT_CTRL));
	for (i = 0; i < ARRAY_SIZE(priv->chi_base); i++) {
		if (priv->chi_base[i])
			dev_err(dev, "  SHRAM+0x%02x = 0x%08x\n", i * 4, priv->chi_base[i]);
	}
	return -ETIMEDOUT;
}

int wav500_boot_fw(struct wav500_priv *priv)
{
	if (priv->fw_ready)
		return 0;
	return wav500_boot_firmware(priv, true);
}

static ssize_t fw_boot_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wav500_priv *priv = pci_get_drvdata(pdev);
	int val = 1, ret;

	if (kstrtoint(buf, 0, &val))
		val = 1;
	ret = wav500_boot_firmware(priv, val != 2);
	return ret ? ret : count;
}

static ssize_t fw_boot_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wav500_priv *priv = pci_get_drvdata(pdev);

	return sysfs_emit(buf, "ready=%d chi_magic=0x%08x irqs=%u rx_mgmt=%u tx_mgmt=%u rx_data=%u tx_data=%u\n",
			  priv->fw_ready, priv->chi_magic_seen, priv->irq_count,
			  priv->rx_mgmt_count, priv->tx_mgmt_count,
			  priv->rx_data_count, priv->tx_data_count);
}
static ssize_t test_ap_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wav500_priv *priv = pci_get_drvdata(pdev);
	int mode = 0, ret;

	if (kstrtoint(buf, 0, &mode))
		mode = 0;
	ret = wav500_start_test_ap(priv, mode);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(test_ap);

static DEVICE_ATTR_RW(fw_boot);

static int wav500_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct wiphy *wiphy;
	struct wav500_priv *priv;
	u32 chip_raw, prog_ver;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
	if (ret)
		return ret;

	pci_set_master(pdev);
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	wiphy = wiphy_new(&wav500_cfg80211_ops, sizeof(*priv));
	if (!wiphy)
		return -ENOMEM;

	priv = wiphy_priv(wiphy);
	priv->pdev = pdev;
	priv->wiphy = wiphy;
	priv->bar0 = pcim_iomap_table(pdev)[0];
	priv->bar0_len = pci_resource_len(pdev, 0);
	priv->radio_idx = pci_domain_nr(pdev->bus) ? 1 : 0;
	priv->is_5ghz = (priv->radio_idx == 1);
	spin_lock_init(&priv->ring_lock);
	spin_lock_init(&priv->dp_lock);
	spin_lock_init(&priv->sta_lock);
	tasklet_setup(&priv->dp_tasklet, wav500_dma_tasklet);
	mutex_init(&priv->man_lock);
	init_completion(&priv->man_cfm);

	if (priv->bar0_len < WAV500_BAR0_MIN_LEN) {
		wiphy_free(wiphy);
		return -EINVAL;
	}

	chip_raw = wav500_r32(priv, WAV500_UMAC_CHIP_ID);
	priv->chip_id = chip_raw & 0xffff;
	prog_ver = wav500_r32(priv, WAV500_PROGMODEL_VER);

	dev_info(&pdev->dev,
		 "WAV500 gen5b (%s) probed: BAR0=%pR chip_id=0x%04x (raw=0x%08x) prog_ver=0x%08x irq=%d\n",
		 priv->is_5ghz ? "5GHz" : "2.4GHz",
		 &pdev->resource[0], priv->chip_id, chip_raw, prog_ver, pdev->irq);

	if (pdev->irq > 0) {
		ret = devm_request_irq(&pdev->dev, pdev->irq, wav500_irq_handler,
				       IRQF_SHARED, KBUILD_MODNAME, priv);
		if (ret)
			dev_warn(&pdev->dev, "request_irq(%d) failed: %d\n",
				 pdev->irq, ret);
	}

	set_wiphy_dev(wiphy, &pdev->dev);
	wav500_cfg_init_wiphy(priv);

	ret = wiphy_register(wiphy);
	if (ret) {
		wiphy_free(wiphy);
		return ret;
	}

	ret = wav500_cfg_register_primary_vif(priv);
	if (ret) {
		wiphy_unregister(wiphy);
		wiphy_free(wiphy);
		return ret;
	}

	pci_set_drvdata(pdev, priv);
	device_create_file(&pdev->dev, &dev_attr_fw_boot);
	device_create_file(&pdev->dev, &dev_attr_test_ap);
	dev_info(&pdev->dev, "registered cfg80211 wiphy %s (trigger FW boot via sysfs fw_boot)\n",
		 wiphy_name(wiphy));
	return 0;
}

static void wav500_pci_remove(struct pci_dev *pdev)
{
	struct wav500_priv *priv = pci_get_drvdata(pdev);

	if (!priv)
		return;
	device_remove_file(&pdev->dev, &dev_attr_test_ap);
	device_remove_file(&pdev->dev, &dev_attr_fw_boot);
	priv->dp_irq_enabled = false;
	wav500_w32(priv, WAV500_IRQ_MASK, 0);
	tasklet_kill(&priv->dp_tasklet);
	wav500_dma_cleanup_rings(priv);
	wav500_cfg_unregister_primary_vif(priv);
	wav500_put_cpus_to_reset(priv);
	if (priv->beacon_buf)
		dma_free_coherent(&pdev->dev, priv->beacon_alloc_len,
				  priv->beacon_buf, priv->beacon_dma);
	if (priv->cal_buf)
		dma_free_coherent(&pdev->dev, priv->cal_alloc_len,
				  priv->cal_buf, priv->cal_dma);
	wiphy_unregister(priv->wiphy);
	wiphy_free(priv->wiphy);
}

static const struct pci_device_id wav500_pci_ids[] = {
	{ PCI_DEVICE(WAV500_PCI_VENDOR_ID, WAV500_PCI_DEVICE_ID_GEN5B) },
	{ }
};
MODULE_DEVICE_TABLE(pci, wav500_pci_ids);

static struct pci_driver wav500_pci_driver = {
	.name = "wav500",
	.id_table = wav500_pci_ids,
	.probe = wav500_pci_probe,
	.remove = wav500_pci_remove,
};
module_pci_driver(wav500_pci_driver);

MODULE_DESCRIPTION("Intel/Lantiq WAV500 (gen5b) clean-room cfg80211 Wi-Fi driver");
MODULE_LICENSE("GPL");
