/*
 * SPL specific code for Compulab CM-FX6 board
 *
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/mx6_ddr_regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/crm_regs.h>
#include <asm/imx-common/iomux-v3.h>
#include <fsl_esdhc.h>
#include "common.h"

DECLARE_GLOBAL_DATA_PTR;

enum ddr_config {
	DDR_16BIT_256MB,
	DDR_32BIT_512MB,
	DDR_32BIT_1GB,
	DDR_64BIT_1GB,
	DDR_64BIT_2GB,
	DDR_64BIT_4GB,
	DDR_UNKNOWN,
};

static void spl_mx6s_dram_setup_iomux(void)
{
	volatile struct mx6sdl_iomux_ddr_regs *ddr_iomux;
	volatile struct mx6sdl_iomux_grp_regs *grp_iomux;
	ddr_iomux = (struct mx6sdl_iomux_ddr_regs *)MX6SDL_IOM_DDR_BASE;
	grp_iomux = (struct mx6sdl_iomux_grp_regs *)MX6SDL_IOM_GRP_BASE;

	ddr_iomux->dram_sdqs0	= 0x00000038;
	ddr_iomux->dram_sdqs1	= 0x00000038;
	ddr_iomux->dram_sdqs2	= 0x00000038;
	ddr_iomux->dram_sdqs3	= 0x00000038;
	ddr_iomux->dram_sdqs4	= 0x00000038;
	ddr_iomux->dram_sdqs5	= 0x00000038;
	ddr_iomux->dram_sdqs6	= 0x00000038;
	ddr_iomux->dram_sdqs7	= 0x00000038;
	ddr_iomux->dram_dqm0	= 0x00000038;
	ddr_iomux->dram_dqm1	= 0x00000038;
	ddr_iomux->dram_dqm2	= 0x00000038;
	ddr_iomux->dram_dqm3	= 0x00000038;
	ddr_iomux->dram_dqm4	= 0x00000038;
	ddr_iomux->dram_dqm5	= 0x00000038;
	ddr_iomux->dram_dqm6	= 0x00000038;
	ddr_iomux->dram_dqm7	= 0x00000038;
	ddr_iomux->dram_cas	= 0x00000038;
	ddr_iomux->dram_ras	= 0x00000038;
	ddr_iomux->dram_sdclk_0	= 0x00000038;
	ddr_iomux->dram_sdclk_1	= 0x00000038;
	/*
	 * Below DRAM_RESET[DDR_SEL] = 0 which is incorrect according to
	 * Freescale QRM, but this is exactly the value used by the automatic
	 * calibration script and it works also in all our tests, so we leave
	 * it as is at this point.
	 */
	ddr_iomux->dram_reset	= 0x00000038;
	ddr_iomux->dram_sdba2	= 0x00000000;
	ddr_iomux->dram_sdodt0	= 0x00000038;
	ddr_iomux->dram_sdodt1	= 0x00000038;
	grp_iomux->grp_b0ds	= 0x00000038;
	grp_iomux->grp_b1ds	= 0x00000038;
	grp_iomux->grp_b2ds	= 0x00000038;
	grp_iomux->grp_b3ds	= 0x00000038;
	grp_iomux->grp_b4ds	= 0x00000038;
	grp_iomux->grp_b5ds	= 0x00000038;
	grp_iomux->grp_b6ds	= 0x00000038;
	grp_iomux->grp_b7ds	= 0x00000038;
	grp_iomux->grp_addds	= 0x00000038;
	grp_iomux->grp_ddrmode_ctl = 0x00020000;
	grp_iomux->grp_ddrpke	= 0x00000000;
	grp_iomux->grp_ddrmode	= 0x00020000;
	grp_iomux->grp_ctlds	= 0x00000038;
	grp_iomux->grp_ddr_type	= 0x000C0000;
}

static void spl_mx6q_dram_setup_iomux(void)
{
	volatile struct mx6qd_iomux_ddr_regs *ddr_iomux;
	volatile struct mx6qd_iomux_grp_regs *grp_iomux;
	ddr_iomux = (struct mx6qd_iomux_ddr_regs *)MX6DQ_IOM_DDR_BASE;
	grp_iomux = (struct mx6qd_iomux_grp_regs *)MX6DQ_IOM_GRP_BASE;

	ddr_iomux->dram_sdqs0	= 0x00000038;
	ddr_iomux->dram_sdqs1	= 0x00000038;
	ddr_iomux->dram_sdqs2	= 0x00000038;
	ddr_iomux->dram_sdqs3	= 0x00000038;
	ddr_iomux->dram_sdqs4	= 0x00000038;
	ddr_iomux->dram_sdqs5	= 0x00000038;
	ddr_iomux->dram_sdqs6	= 0x00000038;
	ddr_iomux->dram_sdqs7	= 0x00000038;
	ddr_iomux->dram_dqm0	= 0x00000038;
	ddr_iomux->dram_dqm1	= 0x00000038;
	ddr_iomux->dram_dqm2	= 0x00000038;
	ddr_iomux->dram_dqm3	= 0x00000038;
	ddr_iomux->dram_dqm4	= 0x00000038;
	ddr_iomux->dram_dqm5	= 0x00000038;
	ddr_iomux->dram_dqm6	= 0x00000038;
	ddr_iomux->dram_dqm7	= 0x00000038;
	ddr_iomux->dram_cas	= 0x00000038;
	ddr_iomux->dram_ras	= 0x00000038;
	ddr_iomux->dram_sdclk_0	= 0x00000038;
	ddr_iomux->dram_sdclk_1	= 0x00000038;
	/*
	 * Below DRAM_RESET[DDR_SEL] = 0 which is incorrect according to
	 * Freescale QRM, but this is exactly the value used by the automatic
	 * calibration script and it works also in all our tests, so we leave
	 * it as is at this point.
	 */
	ddr_iomux->dram_reset	= 0x00000038;
	ddr_iomux->dram_sdba2	= 0x00000000;
	ddr_iomux->dram_sdodt0	= 0x00000038;
	ddr_iomux->dram_sdodt1	= 0x00000038;
	grp_iomux->grp_b0ds	= 0x00000038;
	grp_iomux->grp_b1ds	= 0x00000038;
	grp_iomux->grp_b2ds	= 0x00000038;
	grp_iomux->grp_b3ds	= 0x00000038;
	grp_iomux->grp_b4ds	= 0x00000038;
	grp_iomux->grp_b5ds	= 0x00000038;
	grp_iomux->grp_b6ds	= 0x00000038;
	grp_iomux->grp_b7ds	= 0x00000038;
	grp_iomux->grp_addds	= 0x00000038;
	grp_iomux->grp_ddrmode_ctl = 0x00020000;
	grp_iomux->grp_ddrpke	= 0x00000000;
	grp_iomux->grp_ddrmode	= 0x00020000;
	grp_iomux->grp_ctlds	= 0x00000038;
	grp_iomux->grp_ddr_type	= 0x000C0000;
}

static void spl_mx6s_dram_init(enum ddr_config dram_config, int reset)
{
	volatile struct mmdc_p_regs *mmdc_p0;
	volatile struct mmdc_p_regs *mmdc_p1;
	mmdc_p0 = (struct mmdc_p_regs *)MX6_MMDC_P0_MDCTL;
	mmdc_p1 = (struct mmdc_p_regs *)MX6_MMDC_P1_MDCTL;

	if (reset)
		mmdc_p0->mdmisc = 2;

	mmdc_p0->mdscr  = 0x00008000;
	while (!(mmdc_p0->mdscr & (1 << 14)))
		;

	mmdc_p0->mpzqhwctrl  = 0xA1390003;
	/* Wait for ZQ_HW_FOR to finish the calibration on both MMDCs */
	while (mmdc_p0->mpzqhwctrl & 0x00010000)
		;

	mmdc_p0->mpwldectrl0 = 0x005B0061;
	mmdc_p0->mpwldectrl1 = 0x004F0055;
	mmdc_p0->mpdgctrl0   = 0x0314030C;
	mmdc_p0->mpdgctrl1   = 0x025C0268;
	mmdc_p0->mprddlctl   = 0x42464646;
	mmdc_p0->mpwrdlctl   = 0x36322C34;
	mmdc_p0->mprddqby0dl = 0x33333333;
	mmdc_p0->mprddqby1dl = 0x33333333;
	mmdc_p0->mprddqby2dl = 0x33333333;
	mmdc_p0->mprddqby3dl = 0x33333333;
	mmdc_p1->mprddqby0dl = 0x33333333;
	mmdc_p1->mprddqby1dl = 0x33333333;
	mmdc_p1->mprddqby2dl = 0x33333333;
	mmdc_p1->mprddqby3dl = 0x33333333;
	mmdc_p0->mpmur0	     = 0x00000800;
	mmdc_p0->mdpdc  = 0x0002002D;
	mmdc_p0->mdotc  = 0x1B444040;
	mmdc_p0->mdcfg0 = 0x676B5335;
	mmdc_p0->mdcfg1 = 0xB68E8F64;
	mmdc_p0->mdcfg2 = 0x01FF00DB;
	mmdc_p0->mdmisc = 0x00091740;
	mmdc_p0->mdrwd  = 0x000026D2;
	mmdc_p0->mdor   = 0x006B1023;

	switch (dram_config) {
	case DDR_16BIT_256MB:
		mmdc_p0->mdctl = 0x83180000;
		break;
	case DDR_32BIT_512MB:
		mmdc_p0->mdctl = 0x83190000;
		break;
	case DDR_32BIT_1GB:
		mmdc_p0->mdctl = 0xC3190000;
		break;
	default:
		puts("Tried to setup invalid DDR configuration\n");
		hang();
	}

	mmdc_p0->mdscr = 0x00088032;
	mmdc_p0->mdscr = 0x0008803A;
	mmdc_p0->mdscr = 0x00008033;
	mmdc_p0->mdscr = 0x0000803B;
	mmdc_p0->mdscr = 0x00068031;
	mmdc_p0->mdscr = 0x00068039;
	mmdc_p0->mdscr = 0x09408030;
	mmdc_p0->mdscr = 0x09408038;
	mmdc_p0->mdscr = 0x04008040;
	mmdc_p0->mdscr = 0x04008048;
	mmdc_p0->mdref = 0x00007800;
	mmdc_p0->mpodtctrl = 0x00022227;
	mmdc_p0->mdpdc = 0x000255ED;
	mmdc_p0->mapsr = 0x00001006;
	mmdc_p0->mdscr = 0x00000000;
	udelay(100);
}

static void spl_mx6q_dram_init(enum ddr_config dram_config, int reset)
{
	volatile struct mmdc_p_regs *mmdc_p0;
	volatile struct mmdc_p_regs *mmdc_p1;
	mmdc_p0 = (struct mmdc_p_regs *)MX6_MMDC_P0_MDCTL;
	mmdc_p1 = (struct mmdc_p_regs *)MX6_MMDC_P1_MDCTL;

	if (reset)
		mmdc_p0->mdmisc = 2;

	mmdc_p0->mdscr  = 0x00008000;
	while (!(mmdc_p0->mdscr & (1 << 14)))
		;

	mmdc_p0->mpzqhwctrl  = 0xA1390003;
	/* Wait for ZQ_HW_FOR to finish the calibration on both MMDCs */
	while (mmdc_p0->mpzqhwctrl & 0x00010000)
		;

	mmdc_p0->mpwldectrl0 = 0x00630068;
	mmdc_p0->mpwldectrl1 = 0x0068005D;
	mmdc_p1->mpwldectrl0 = 0x0035004C;
	mmdc_p1->mpwldectrl1 = 0x00170026;
	mmdc_p0->mpdgctrl0   = 0x04140428;
	mmdc_p0->mpdgctrl1   = 0x037C037C;
	mmdc_p1->mpdgctrl0   = 0x0374037C;
	mmdc_p1->mpdgctrl1   = 0x0350032C;
	mmdc_p0->mprddlctl   = 0x3C30303A;
	mmdc_p1->mprddlctl   = 0x30322A3C;
	mmdc_p0->mpwrdlctl   = 0x3A344038;
	mmdc_p1->mpwrdlctl   = 0x48304A3E;
	mmdc_p0->mprddqby0dl = 0x33333333;
	mmdc_p0->mprddqby1dl = 0x33333333;
	mmdc_p0->mprddqby2dl = 0x33333333;
	mmdc_p0->mprddqby3dl = 0x33333333;
	mmdc_p1->mprddqby0dl = 0x33333333;
	mmdc_p1->mprddqby1dl = 0x33333333;
	mmdc_p1->mprddqby2dl = 0x33333333;
	mmdc_p1->mprddqby3dl = 0x33333333;
	mmdc_p0->mpmur0	     = 0x00000800;
	mmdc_p1->mpmur0	     = 0x00000800;
	/* MMDC init: in DDR3, 64-bit mode, only MMDC0 is initiated: */
	mmdc_p0->mdpdc  = 0x00020036;
	mmdc_p0->mdotc  = 0x09444040;
	mmdc_p0->mdcfg0 = 0x8A8F79A5;
	mmdc_p0->mdcfg1 = 0xFF738F64;
	mmdc_p0->mdcfg2 = 0x01FF00DD;
	mmdc_p0->mdmisc = 0x00091740;
	mmdc_p0->mdrwd  = 0x000026d2;
	mmdc_p0->mdor   = 0x008F1023;

	switch (dram_config) {
	case DDR_16BIT_256MB:
		mmdc_p0->mdctl = 0x83180000;
		break;
	case DDR_32BIT_512MB:
		mmdc_p0->mdctl = 0x83190000;
		break;
	case DDR_64BIT_1GB:
		mmdc_p0->mdctl = 0x831A0000;
		break;
	case DDR_64BIT_2GB:
		mmdc_p0->mdctl = 0xC31A0000;
		break;
	case DDR_64BIT_4GB:
		mmdc_p0->mdctl = 0xC41A0000;
		break;
	default:
		puts("Tried to setup invalid DDR configuration\n");
		hang();
	}

	mmdc_p0->mdscr = 0x00088032;
	mmdc_p0->mdscr = 0x0008803A;
	mmdc_p0->mdscr = 0x00008033;
	mmdc_p0->mdscr = 0x0000803B;
	mmdc_p0->mdscr = 0x00068031;
	mmdc_p0->mdscr = 0x00068039;
	mmdc_p0->mdscr = 0x09408030;
	mmdc_p0->mdscr = 0x09408038;
	mmdc_p0->mdscr = 0x04008040;
	mmdc_p0->mdscr = 0x04008048;
	mmdc_p0->mdref = 0x00007800;
	mmdc_p0->mpodtctrl = 0x00022227;
	mmdc_p1->mpodtctrl = 0x00022227;
	mmdc_p0->mdpdc = 0x000255F6;
	mmdc_p0->mapsr = 0x00001006;
	mmdc_p0->mdscr = 0x00000000;
	udelay(100);
}

static int cm_fx6_spl_dram_init(void)
{
	u32 cpurev, imxtype;
	unsigned long bank1_size, bank2_size;

	cpurev = get_cpu_rev();
	imxtype = (cpurev & 0xFF000) >> 12;

	switch (imxtype) {
	case MXC_CPU_MX6SOLO:
		spl_mx6s_dram_setup_iomux();

		spl_mx6s_dram_init(DDR_32BIT_1GB, 0);
		bank1_size = get_ram_size((long int *)PHYS_SDRAM_1, 0x80000000);
		if (bank1_size == 0x40000000)
			return 0;

		if (bank1_size == 0x20000000) {
			spl_mx6s_dram_init(DDR_32BIT_512MB, 1);
			return 0;
		}

		spl_mx6s_dram_init(DDR_16BIT_256MB, 1);
		bank1_size = get_ram_size((long int *)PHYS_SDRAM_1, 0x80000000);
		if (bank1_size == 0x10000000)
			return 0;

		break;
	case MXC_CPU_MX6D:
	case MXC_CPU_MX6Q:
		spl_mx6q_dram_setup_iomux();

		spl_mx6q_dram_init(DDR_64BIT_4GB, 0);
		bank1_size = get_ram_size((long int *)PHYS_SDRAM_1, 0x80000000);
		if (bank1_size == 0x80000000)
			return 0;

		if (bank1_size == 0x40000000) {
			bank2_size = get_ram_size((long int *)PHYS_SDRAM_2,
								0x80000000);
			if (bank2_size == 0x40000000) {
				/* Don't do a full reset here */
				spl_mx6q_dram_init(DDR_64BIT_2GB, 0);
			} else {
				spl_mx6q_dram_init(DDR_64BIT_1GB, 1);
			}

			return 0;
		}

		spl_mx6q_dram_init(DDR_32BIT_512MB, 1);
		bank1_size = get_ram_size((long int *)PHYS_SDRAM_1, 0x80000000);
		if (bank1_size == 0x20000000)
			return 0;

		spl_mx6q_dram_init(DDR_16BIT_256MB, 1);
		bank1_size = get_ram_size((long int *)PHYS_SDRAM_1, 0x80000000);
		if (bank1_size == 0x10000000)
			return 0;

		break;
	}

	return -1;
}

static void cm_fx6_setup_uart(void)
{
	iomux_v3_cfg_t uart_pad_ctl = MUX_PAD_CTRL(UART_PAD_CTRL);
	MX6QDL_SET_PAD(PAD_KEY_COL0__UART4_TX_DATA, uart_pad_ctl);
	MX6QDL_SET_PAD(PAD_KEY_ROW0__UART4_RX_DATA, uart_pad_ctl);
	enable_uart_clk(1);
}

#ifdef CONFIG_SPL_SPI_SUPPORT
static void cm_fx6_setup_ecspi(void)
{
	enable_cspi_clock(1, 0);
	cm_fx6_set_ecspi_iomux();
}
#else
static void cm_fx6_setup_ecspi(void) { }
#endif

void board_init_f(ulong dummy)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	gd = &gdata;
	/*
	 * We don't use DMA in SPL, but we do need it in U-Boot. U-Boot
	 * initializes DMA very early (before all board code), so the only
	 * opportunity we have to initialize APBHDMA clocks is in SPL.
	 */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
	enable_usdhc_clk(1, 2);
	arch_cpu_init();
	timer_init();
	cm_fx6_setup_ecspi();
	cm_fx6_setup_uart();
	get_clocks();
	preloader_console_init();
	gpio_direction_output(CM_FX6_GREEN_LED, 1);
	if (cm_fx6_spl_dram_init()) {
		puts("!!!ERROR!!! DRAM detection failed!!!\n"
				"Dumping MMDC registers:\n");
		dump_mmdc_registers();
		hang();
	}

	memset(__bss_start, 0, __bss_end - __bss_start);
	board_init_r(NULL, 0);
}

static u32 boot_dev;

void spl_board_init(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;

	if (bt_mem_ctl == 0x3 && !bt_mem_type) {
		puts("Booting from SPI flash\n");
		boot_dev = BOOT_DEVICE_SPI;
	} else if (bt_mem_ctl == 0x4 || bt_mem_ctl == 0x5) {
		puts("Booting from MMC\n");
		boot_dev = BOOT_DEVICE_MMC1;
	} else {
		puts("Unknown boot device\n");
		boot_dev = -1;
	}
}

u32 spl_boot_device(void)
{
	return boot_dev;
}

u32 spl_boot_mode(void)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC1:
		return MMCSD_MODE_RAW;
		break;
	default:
		puts("spl: unsupported boot device\n");
		hang();
	}
}

#ifdef CONFIG_SPL_MMC_SUPPORT
int board_mmc_init(bd_t *bis)
{
	cm_fx6_set_usdhc_iomux();

	usdhc_cfg[2].sdhc_clk = mxc_get_clock(usdhc_clk[2]);
	usdhc_cfg[2].max_bus_width = 4;

	return fsl_esdhc_initialize(bis, &usdhc_cfg[2]);
}
#endif
