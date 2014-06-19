/*
 * Board functions for Compulab CM-FX6 board
 *
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/mx6_ddr_regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include "common.h"

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_NAND_MXS
static void cm_fx6_setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	MX6QDL_SET_PAD(PAD_NANDF_CLE__NAND_CLE, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_ALE__NAND_ALE, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_CS0__NAND_CE0_B, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_RB0__NAND_READY_B, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D0__NAND_DATA00, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D1__NAND_DATA01, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D2__NAND_DATA02, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D3__NAND_DATA03, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D4__NAND_DATA04, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D5__NAND_DATA05, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D6__NAND_DATA06, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_NANDF_D7__NAND_DATA07, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD4_CMD__NAND_RE_B, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD4_CLK__NAND_WE_B, MUX_PAD_CTRL(NO_PAD_CTRL));

	setbits_le32(&mxc_ccm->CCGR6,
		     MXC_CCM_CCGR6_USDHC3_MASK | MXC_CCM_CCGR6_USDHC4_MASK);

	clrbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_MASK);

	clrbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

	/* config gpmi and bch clock to 11Mhz*/
	clrsetbits_le32(&mxc_ccm->cs2cdr,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF(0xf) |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED(1)   |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL(0));

	setbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);
	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_MASK);
}
#else
static void cm_fx6_setup_gpmi_nand(void) {}
#endif

#ifdef CONFIG_FSL_ESDHC
int board_mmc_init(bd_t *bis)
{
	int i;

	cm_fx6_set_usdhc_iomux();
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		usdhc_cfg[i].sdhc_clk = mxc_get_clock(usdhc_clk[i]);
		usdhc_cfg[i].max_bus_width = 4;
		fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		enable_usdhc_clk(1, i);
	}

	return 0;
}
#endif

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;
	cm_fx6_setup_gpmi_nand();

	return 0;
}

int checkboard(void)
{
	puts("Board: CM-FX6\n");
	return 0;
}

static ulong bank1_size;
static ulong bank2_size;

static int probe_mmdc_config(void)
{
	struct mmdc_p_regs *mmdc_p0 = (struct mmdc_p_regs *)MX6_MMDC_P0_MDCTL;

	switch (mmdc_p0->mdctl) {
	case 0x83180000: /* DDR_16BIT_256MB */
		gd->ram_size	= 0x10000000;
		bank1_size	= 0x10000000;
		bank2_size	= 0;
		break;
	case 0x83190000: /* DDR_32BIT_512MB */
		gd->ram_size	= 0x20000000;
		bank1_size	= 0x20000000;
		bank2_size	= 0;
		break;
	case 0xC3190000: /* DDR_32BIT_1GB */
		gd->ram_size	= 0x40000000;
		bank1_size	= 0x20000000;
		bank2_size	= 0x20000000;
		break;
	case 0x831A0000: /* DDR_64BIT_1GB */
		gd->ram_size	= 0x40000000;
		bank1_size	= 0x40000000;
		bank2_size	= 0;
		break;
	case 0xC31A0000: /* DDR_64BIT_2GB */
		gd->ram_size	= 0x80000000;
		bank1_size	= 0x40000000;
		bank2_size	= 0x40000000;
		break;
	case 0xC41A0000: /* DDR_64BIT_4GB */
		gd->ram_size	= 0xEFF00000;
		bank1_size	= 0x70000000;
		bank2_size	= 0x7FF00000;
		break;
	default:
		puts("!!!ERROR!!! Unsupported DRAM configuration.\n"
		     "Dumping MMDC registers:\n");
		dump_mmdc_registers();
		return -1;
	}

	return 0;
}

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = bank1_size;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
	gd->bd->bi_dram[1].size = bank2_size;
}

int dram_init(void)
{
	return probe_mmdc_config();
}

u32 get_board_rev(void)
{
	return 100;
}
