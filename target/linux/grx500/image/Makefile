# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

ifeq ($(SUBTARGET),)
  DEVICE_CHECK_PROFILE = $(CONFIG_TARGET_$(call target_conf,$(BOARD))_$(1))
endif

KERNEL_LOADADDR := 0xa0020000

define Build/grx500-uImage
	ENTRY=$$($(TARGET_CROSS)readelf -h $(if $(findstring initramfs,$@),$(KDIR)/vmlinux-initramfs.elf,$(KDIR)/vmlinux.elf) | awk '/Entry point address:/ {print $$4}' | sed 's/^0x60/0xa0/; s/^0x80/0xa0/'); \
	mkimage -A $(LINUX_KARCH) -O linux -T kernel -C $(word 1,$(1)) \
		-a $(KERNEL_LOADADDR) -e "$$ENTRY" \
		-n '$(call toupper,$(LINUX_KARCH)) $(VERSION_DIST) Linux-$(LINUX_VERSION)' \
		-d $@ $@.new
	mv $@.new $@
endef

define Device/Default
  FILESYSTEMS := squashfs
  DEVICE_DTS_DIR := ../dts
  KERNEL_DEPENDS = $$(wildcard $$(DTS_DIR)/$$(DEVICE_DTS).dts)
  KERNEL_INITRAMFS_NAME = $$(KERNEL_NAME)-initramfs
  KERNEL := kernel-bin | append-dtb | lzma | grx500-uImage lzma
  KERNEL_INITRAMFS := kernel-bin | append-dtb | lzma | grx500-uImage lzma
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := sysupgrade-tar | append-metadata
endef

define Device/asus_blue-cave
  DEVICE_VENDOR := ASUS
  DEVICE_MODEL := Blue Cave
  DEVICE_DTS := grx350_asus_bluecave
  BLOCKSIZE := 128k
  PAGESIZE := 2048
  UBINIZE_OPTS := -E 5
endef
TARGET_DEVICES += asus_blue-cave

$(eval $(call BuildImage))
