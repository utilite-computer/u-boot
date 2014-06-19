/*
 * Code used by both U-Boot and SPL for Compulab CM-FX6
 *
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx6_ddr_regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/gpio.h>
#include "common.h"

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_FSL_ESDHC
#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

void cm_fx6_set_usdhc_iomux(void)
{
	MX6QDL_SET_PAD(PAD_SD1_CLK__SD1_CLK,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD1_CMD__SD1_CMD,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD1_DAT0__SD1_DATA0,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD1_DAT1__SD1_DATA1,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD1_DAT2__SD1_DATA2,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD1_DAT3__SD1_DATA3,	MUX_PAD_CTRL(USDHC_PAD_CTRL));

	MX6QDL_SET_PAD(PAD_SD2_CLK__SD2_CLK,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_CMD__SD2_CMD,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT0__SD2_DATA0,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT1__SD2_DATA1,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT2__SD2_DATA2,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT3__SD2_DATA3,	MUX_PAD_CTRL(USDHC_PAD_CTRL));

	MX6QDL_SET_PAD(PAD_SD3_CLK__SD3_CLK,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_CMD__SD3_CMD,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT0__SD3_DATA0,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT1__SD3_DATA1,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT2__SD3_DATA2,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT3__SD3_DATA3,	MUX_PAD_CTRL(USDHC_PAD_CTRL));

	MX6QDL_SET_PAD(PAD_SD3_DAT4__SD3_DATA4,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT5__SD3_DATA5,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT6__SD3_DATA6,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD3_DAT7__SD3_DATA7,	MUX_PAD_CTRL(USDHC_PAD_CTRL));
}

/* CINS bit doesn't work, so always try to access the MMC card */
int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}
#endif

#ifdef CONFIG_MXC_SPI
#define ECSPI_PAD_CTRL (PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED | \
		PAD_CTL_PUS_100K_DOWN | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

void cm_fx6_set_ecspi_iomux(void)
{
	MX6QDL_SET_PAD(PAD_EIM_D16__ECSPI1_SCLK, MUX_PAD_CTRL(ECSPI_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_EIM_D17__ECSPI1_MISO, MUX_PAD_CTRL(ECSPI_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_EIM_D18__ECSPI1_MOSI, MUX_PAD_CTRL(ECSPI_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_EIM_EB2__GPIO2_IO30,  MUX_PAD_CTRL(ECSPI_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_EIM_D19__ECSPI1_SS1,  MUX_PAD_CTRL(ECSPI_PAD_CTRL));
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (CM_FX6_ECSPI_BUS0_CS0) : -1;
}
#endif

void dump_mmdc_registers(void)
{
	struct mmdc_p_regs *mmdc_p0 = (struct mmdc_p_regs *)MX6_MMDC_P0_MDCTL;
	struct mmdc_p_regs *mmdc_p1 = (struct mmdc_p_regs *)MX6_MMDC_P1_MDCTL;
	struct mx6qd_iomux_ddr_regs *ddr_iomux =
			(struct mx6qd_iomux_ddr_regs *)MX6DQ_IOM_DDR_BASE;
	struct mx6qd_iomux_grp_regs *grp_iomux =
			(struct mx6qd_iomux_grp_regs *)MX6DQ_IOM_GRP_BASE;

	printf("IOMUX DDR\n");
	printf("dram_cas     0x%x\n", ddr_iomux->dram_cas);
	printf("dram_dqm0    0x%x\n", ddr_iomux->dram_dqm0);
	printf("dram_dqm1    0x%x\n", ddr_iomux->dram_dqm1);
	printf("dram_dqm2    0x%x\n", ddr_iomux->dram_dqm2);
	printf("dram_dqm3    0x%x\n", ddr_iomux->dram_dqm3);
	printf("dram_dqm4    0x%x\n", ddr_iomux->dram_dqm4);
	printf("dram_dqm5    0x%x\n", ddr_iomux->dram_dqm5);
	printf("dram_dqm6    0x%x\n", ddr_iomux->dram_dqm6);
	printf("dram_dqm7    0x%x\n", ddr_iomux->dram_dqm7);
	printf("dram_ras     0x%x\n", ddr_iomux->dram_ras);
	printf("dram_reset   0x%x\n", ddr_iomux->dram_reset);
	printf("dram_sdba2   0x%x\n", ddr_iomux->dram_sdba2);
	printf("dram_sdcke0  0x%x\n", ddr_iomux->dram_sdcke0);
	printf("dram_sdcke1  0x%x\n", ddr_iomux->dram_sdcke1);
	printf("dram_sdclk_0 0x%x\n", ddr_iomux->dram_sdclk_0);
	printf("dram_sdclk_1 0x%x\n", ddr_iomux->dram_sdclk_1);
	printf("dram_sdodt0  0x%x\n", ddr_iomux->dram_sdodt0);
	printf("dram_sdodt1  0x%x\n", ddr_iomux->dram_sdodt1);
	printf("dram_sdqs0   0x%x\n", ddr_iomux->dram_sdqs0);
	printf("dram_sdqs1   0x%x\n", ddr_iomux->dram_sdqs1);
	printf("dram_sdqs2   0x%x\n", ddr_iomux->dram_sdqs2);
	printf("dram_sdqs3   0x%x\n", ddr_iomux->dram_sdqs3);
	printf("dram_sdqs4   0x%x\n", ddr_iomux->dram_sdqs4);
	printf("dram_sdqs5   0x%x\n", ddr_iomux->dram_sdqs5);
	printf("dram_sdqs6   0x%x\n", ddr_iomux->dram_sdqs6);
	printf("dram_sdqs7   0x%x\n", ddr_iomux->dram_sdqs7);

	printf("GPR DDR\n");
	printf("grp_addds       0x%x\n", grp_iomux->grp_addds);
	printf("grp_b0ds        0x%x\n", grp_iomux->grp_b0ds);
	printf("grp_b1ds        0x%x\n", grp_iomux->grp_b1ds);
	printf("grp_b2ds        0x%x\n", grp_iomux->grp_b2ds);
	printf("grp_b3ds        0x%x\n", grp_iomux->grp_b3ds);
	printf("grp_b4ds        0x%x\n", grp_iomux->grp_b4ds);
	printf("grp_b5ds        0x%x\n", grp_iomux->grp_b5ds);
	printf("grp_b6ds        0x%x\n", grp_iomux->grp_b6ds);
	printf("grp_b7ds        0x%x\n", grp_iomux->grp_b7ds);
	printf("grp_ctlds       0x%x\n", grp_iomux->grp_ctlds);
	printf("grp_ddr_type    0x%x\n", grp_iomux->grp_ddr_type);
	printf("grp_ddrmode     0x%x\n", grp_iomux->grp_ddrmode);
	printf("grp_ddrmode_ctl 0x%x\n", grp_iomux->grp_ddrmode_ctl);
	printf("grp_ddrpke      0x%x\n", grp_iomux->grp_ddrpke);

	printf("MMDC P0\n");
	printf("mdctl       0x%x\n",	mmdc_p0->mdctl);
	printf("mdpdc       0x%x\n",	mmdc_p0->mdpdc);
	printf("mdotc       0x%x\n",	mmdc_p0->mdotc);
	printf("mdcfg0      0x%x\n",	mmdc_p0->mdcfg0);
	printf("mdcfg1      0x%x\n",	mmdc_p0->mdcfg1);
	printf("mdcfg2      0x%x\n",	mmdc_p0->mdcfg2);
	printf("mdmisc      0x%x\n",	mmdc_p0->mdmisc);
	printf("mdscr       0x%x\n",	mmdc_p0->mdscr);
	printf("mdref       0x%x\n",	mmdc_p0->mdref);
	printf("mdrwd       0x%x\n",	mmdc_p0->mdrwd);
	printf("mdor        0x%x\n",	mmdc_p0->mdor);
	printf("mdasp       0x%x\n",	mmdc_p0->mdasp);
	printf("mapsr       0x%x\n",	mmdc_p0->mapsr);
	printf("mpzqhwctrl  0x%x\n",	mmdc_p0->mpzqhwctrl);
	printf("mpwldectrl0 0x%x\n",	mmdc_p0->mpwldectrl0);
	printf("mpwldectrl1 0x%x\n",	mmdc_p0->mpwldectrl1);
	printf("mpodtctrl   0x%x\n",	mmdc_p0->mpodtctrl);
	printf("mprddqby0dl 0x%x\n",	mmdc_p0->mprddqby0dl);
	printf("mprddqby1dl 0x%x\n",	mmdc_p0->mprddqby1dl);
	printf("mprddqby2dl 0x%x\n",	mmdc_p0->mprddqby2dl);
	printf("mprddqby3dl 0x%x\n",	mmdc_p0->mprddqby3dl);
	printf("mpdgctrl0   0x%x\n",	mmdc_p0->mpdgctrl0);
	printf("mpdgctrl1   0x%x\n",	mmdc_p0->mpdgctrl1);
	printf("mprddlctl   0x%x\n",	mmdc_p0->mprddlctl);
	printf("mpwrdlctl   0x%x\n",	mmdc_p0->mpwrdlctl);
	printf("mpmur0      0x%x\n",	mmdc_p0->mpmur0);

	printf("MMDC P1\n");
	printf("mdctl       0x%x\n",	mmdc_p1->mdctl);
	printf("mdpdc       0x%x\n",	mmdc_p1->mdpdc);
	printf("mdotc       0x%x\n",	mmdc_p1->mdotc);
	printf("mdcfg0      0x%x\n",	mmdc_p1->mdcfg0);
	printf("mdcfg1      0x%x\n",	mmdc_p1->mdcfg1);
	printf("mdcfg2      0x%x\n",	mmdc_p1->mdcfg2);
	printf("mdmisc      0x%x\n",	mmdc_p1->mdmisc);
	printf("mdscr       0x%x\n",	mmdc_p1->mdscr);
	printf("mdref       0x%x\n",	mmdc_p1->mdref);
	printf("mdrwd       0x%x\n",	mmdc_p1->mdrwd);
	printf("mdor        0x%x\n",	mmdc_p1->mdor);
	printf("mdasp       0x%x\n",	mmdc_p1->mdasp);
	printf("mapsr       0x%x\n",	mmdc_p1->mapsr);
	printf("mpzqhwctrl  0x%x\n",	mmdc_p1->mpzqhwctrl);
	printf("mpwldectrl0 0x%x\n",	mmdc_p1->mpwldectrl0);
	printf("mpwldectrl1 0x%x\n",	mmdc_p1->mpwldectrl1);
	printf("mpodtctrl   0x%x\n",	mmdc_p1->mpodtctrl);
	printf("mprddqby0dl 0x%x\n",	mmdc_p1->mprddqby0dl);
	printf("mprddqby1dl 0x%x\n",	mmdc_p1->mprddqby1dl);
	printf("mprddqby2dl 0x%x\n",	mmdc_p1->mprddqby2dl);
	printf("mprddqby3dl 0x%x\n",	mmdc_p1->mprddqby3dl);
	printf("mpdgctrl0   0x%x\n",	mmdc_p1->mpdgctrl0);
	printf("mpdgctrl1   0x%x\n",	mmdc_p1->mpdgctrl1);
	printf("mprddlctl   0x%x\n",	mmdc_p1->mprddlctl);
	printf("mpwrdlctl   0x%x\n",	mmdc_p1->mpwrdlctl);
	printf("mpmur0      0x%x\n",	mmdc_p1->mpmur0);
}
