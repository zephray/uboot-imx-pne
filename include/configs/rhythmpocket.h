/*
 * Copyright (C) 2019 Wenting Zhang <zephray@outlook.com>
 *
 * Configuration settings for the RhythmPocket.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX7D_SABRESD_CONFIG_H
#define __MX7D_SABRESD_CONFIG_H

#include "mx7_common.h"
#include "imx_env.h"

#define CONFIG_DBG_MONITOR
#define PHYS_SDRAM_SIZE			SZ_256M

#define CONFIG_MXC_UART_BASE            UART1_IPS_BASE_ADDR

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(32 * SZ_1M)

/* MMC Config*/
#define CONFIG_SYS_FSL_ESDHC_ADDR       0

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define MFG_NAND_PARTITION ""

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
        "uart_from_osc " \
		"root=/dev/mmcblk0p1 " \
        "rootwait " \
		"\0" \
	"kboot="MFG_BOOT_CMD"\0"\
	"bootcmd_mfg=run mfgtool_args;" \
        "bootz 0x80800000 - 0x83000000\0" \
	"initrd_addr=0x86800000\0" \
	"initrd_high=0xffffffff\0" \
	"emmc_dev=1\0"\
	"sd_dev=0\0" \
	"mtdparts=" MFG_NAND_PARTITION \
	"\0"\

#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/mmcblk0p1 rootwait\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"ext4load mmc 0 0x80800000 boot/zImage; " \
        "ext4load mmc 0 0x83000000 boot/imx7s-rhythmpocket.dtb; " \
		"bootz 0x80800000 - 0x83000000\0" \

#define CONFIG_BOOTCOMMAND \
	"run mmcboot; " \

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* environment organization */
#define CONFIG_ENV_SIZE			SZ_8K

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(14 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(896 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(60 << 20)
#define CONFIG_ENV_SECT_SIZE		(128 << 10)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#endif

#define CONFIG_SYS_FSL_USDHC_NUM	1

/* MMC Config*/
#define CONFIG_SYS_FSL_ESDHC_ADDR       0
#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC1 */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"  /* USDHC1 */

/* USB Configs */
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)

#define CONFIG_IMX_THERMAL

/* #define CONFIG_SPLASH_SCREEN*/
/* #define CONFIG_MXC_EPDC*/

#if defined(CONFIG_MXC_EPDC) && defined(CONFIG_FSL_QSPI)
#error "EPDC Pins conflicts QSPI, Either EPDC or QSPI can be enabled!"
#endif

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define CONFIG_SF_DEFAULT_BUS		1 /* SOFT SPI occupies the BUS 0, so change the QSPI1 to BUS 1*/
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_64M
#define QSPI0_BASE_ADDR			QSPI1_IPS_BASE_ADDR
#define QSPI0_AMBA_BASE			QSPI0_ARB_BASE_ADDR
#endif

#define CONFIG_USBD_HS

#endif	/* __CONFIG_H */
