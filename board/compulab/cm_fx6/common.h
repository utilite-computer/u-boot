/*
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/mx6-pins.h>
#include <asm/arch/clock.h>

#define MX6QDL_SET_PAD(p, q) \
	if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D)) \
		imx_iomux_v3_setup_pad(MX6Q_##p | q);\
	else \
		imx_iomux_v3_setup_pad(MX6DL_##p | q)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |	\
			PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	\
			PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define CM_FX6_ECSPI_BUS0_CS0	IMX_GPIO_NR(2, 30)
#define CM_FX6_GREEN_LED	IMX_GPIO_NR(2, 31)

#if defined(CONFIG_FSL_ESDHC)
#include <fsl_esdhc.h>

static __maybe_unused struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

static __maybe_unused enum mxc_clock usdhc_clk[3] = {
	MXC_ESDHC_CLK,
	MXC_ESDHC2_CLK,
	MXC_ESDHC3_CLK,
};
#endif

void cm_fx6_set_usdhc_iomux(void);
void cm_fx6_set_ecspi_iomux(void);
void dump_mmdc_registers(void);
