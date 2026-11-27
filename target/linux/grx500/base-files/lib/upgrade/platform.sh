# SPDX-License-Identifier: GPL-2.0-only

PART_NAME=firmware
REQUIRE_IMAGE_METADATA=1

RAMFS_COPY_BIN='fw_printenv fw_setenv'
RAMFS_COPY_DATA='/etc/fw_env.config /var/lock/fw_printenv.lock'

bluecave_get_active_bank() {
	local bank

	# 1) Prefer active_bank passed by stock U-Boot on kernel cmdline
	bank=$(sed -n 's/.*active_bank=\([AB]\).*/\1/p' /proc/cmdline)
	[ -n "$bank" ] && { echo "$bank"; return 0; }

	# 2) Fallback to U-Boot environment via fw_printenv if available
	if command -v fw_printenv >/dev/null 2>&1; then
		bank=$(fw_printenv -n active_bank 2>/dev/null)
		case "$bank" in
			A|B) echo "$bank"; return 0 ;;
		esac
	fi

	# 3) Fallback to raw ubootconfigA MTD scan (e.g. from initramfs)
	local mtd
	mtd=$(find_mtd_part ubootconfigA 2>/dev/null)
	if [ -n "$mtd" ]; then
		bank=$(strings "$mtd" 2>/dev/null | sed -n 's/^active_bank=\([AB]\)$/\1/p' | head -n 1)
		[ -n "$bank" ] && { echo "$bank"; return 0; }
	fi

	echo "A"
}

platform_check_image() {
	return 0
}

platform_do_upgrade() {
	local board=$(board_name)

	case "$board" in
	asus,blue-cave)
		# Stock U-Boot attaches UBI on MTD "system_sw" and boots the
		# active A/B bank (kernelA/rootfsA or kernelB/rootfsB).
		# Updating the active bank volumes in-place preserves
		# bootcoreA/bootcoreB and stays in sync with U-Boot active_bank.
		local bank=$(bluecave_get_active_bank)

		CI_UBIPART="system_sw"
		if [ "$bank" = "B" ]; then
			CI_KERNPART="kernelB"
			CI_ROOTPART="rootfsB"
		else
			CI_KERNPART="kernelA"
			CI_ROOTPART="rootfsA"
		fi

		if command -v fw_setenv >/dev/null 2>&1; then
			fw_setenv update_chk 0 2>/dev/null || true
		fi

		nand_do_upgrade "$1"
		;;
	*)
		default_do_upgrade "$1"
		;;
	esac
}
