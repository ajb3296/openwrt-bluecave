# SPDX-License-Identifier: GPL-2.0-only

PART_NAME=firmware
REQUIRE_IMAGE_METADATA=1

RAMFS_COPY_BIN='fw_printenv fw_setenv'
RAMFS_COPY_DATA='/etc/fw_env.config /var/lock/fw_printenv.lock'

platform_check_image() {
	return 0
}

platform_do_upgrade() {
	local board=$(board_name)

	case "$board" in
	asus,blue-cave)
		# Stock U-Boot attaches UBI on MTD "system_sw" and boots
		# volume "kernelA" with rootfs on "rootfsA" (+ rootfs_data).
		# Updating volumes in-place preserves "bootcoreA"/"bootcoreB".
		CI_UBIPART="system_sw"
		CI_KERNPART="kernelA"
		CI_ROOTPART="rootfsA"
		nand_do_upgrade "$1"
		;;
	*)
		default_do_upgrade "$1"
		;;
	esac
}
