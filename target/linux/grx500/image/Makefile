# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

KERNEL_LOADADDR := 0xa0020000

define Device/Default
  PROFILES := Default
  FILESYSTEMS := squashfs
  KERNEL_DEPENDS = $$(wildcard $$(DTS_DIR)/$$(DEVICE_DTS).dts)
  KERNEL_INITRAMFS_NAME = $$(KERNEL_NAME)-initramfs
  KERNEL := kernel-bin | append-dtb | lzma | uImage lzma
  KERNEL_INITRAMFS := kernel-bin | append-dtb | lzma | uImage lzma
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | append-rootfs | pad-rootfs | append-metadata
endef

define Device/asus_blue-cave
  DEVICE_VENDOR := ASUS
  DEVICE_MODEL := Blue Cave
  DEVICE_DTS := grx350_asus_bluecave
  BLOCKSIZE := 128k
  PAGESIZE := 2048
endef
TARGET_DEVICES += asus_blue-cave

$(eval $(call BuildImage))
