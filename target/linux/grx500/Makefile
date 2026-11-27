# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=mips
BOARD:=grx500
BOARDNAME:=Intel/Lantiq GRX350/GRX500 (xRX500)
FEATURES:=ramdisk squashfs nand nomips16
CPU_TYPE:=24kc

KERNEL_PATCHVER:=6.18

DEFAULT_PACKAGES += \
	iw \
	wifi-scripts \
	wireless-regdb \
	wpad-basic-mbedtls

include $(INCLUDE_DIR)/target.mk

KERNELNAME:=vmlinux

define Target/Description
	Build firmware images for Intel/Lantiq GRX350/GRX500 (xRX500) SoCs
endef

$(eval $(call BuildTarget))
