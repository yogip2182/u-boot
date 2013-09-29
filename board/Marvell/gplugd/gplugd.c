/*
 * (C) Copyright 2011
 * eInfochips Ltd. <www.einfochips.com>
 * Written-by: Ajay Bhargav <ajay.bhargav@einfochips.com>
 *
 * Based on Aspenite:
 * (C) Copyright 2010
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
 * Contributor: Mahavir Jain <mjain@marvell.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <mvmfp.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mfp.h>
#include <asm/arch/armada100.h>
#include <asm/gpio.h>
#include <miiphy.h>

#ifdef CONFIG_ARMADA100_FEC
#include <net.h>
#include <netdev.h>
#endif /* CONFIG_ARMADA100_FEC */

DECLARE_GLOBAL_DATA_PTR;

int board_early_init_f(void)
{
	u32 mfp_cfg[] = {
		/* I2C */
		MFP105_CI2C_SDA,
		MFP106_CI2C_SCL,

		/* Enable Console on UART3 */
		MFPO8_UART3_TXD,
		MFPO9_UART3_RXD,

		/* Ethernet PHY Interface */
		MFP086_ETH_TXCLK,
		MFP087_ETH_TXEN,
		MFP088_ETH_TXDQ3,
		MFP089_ETH_TXDQ2,
		MFP090_ETH_TXDQ1,
		MFP091_ETH_TXDQ0,
		MFP092_ETH_CRS,
		MFP093_ETH_COL,
		MFP094_ETH_RXCLK,
		MFP095_ETH_RXER,
		MFP096_ETH_RXDQ3,
		MFP097_ETH_RXDQ2,
		MFP098_ETH_RXDQ1,
		MFP099_ETH_RXDQ0,
		MFP100_ETH_MDC,
		MFP101_ETH_MDIO,
		MFP103_ETH_RXDV,

		/* SSP2 */
		MFP107_SSP2_RXD,
		MFP108_SSP2_TXD,
		MFP110_SSP2_CS,
		MFP111_SSP2_CLK,

		/* MMC1 */
		MFP040_MMC1_D1,
		MFP041_MMC1_D0,
		MFP043_MMC1_CLK,
		MFP049_MMC1_CMD,
		MFP051_MMC1_D3,
		MFP052_MMC1_D2,
		MFP053_MMC1_CD,

		/* MMC2 */
		MFP028_MMC2_CMD,
		MFP029_MMC2_CLK,
		MFP030_MMC2_D0,
		MFP031_MMC2_D1,
		MFP032_MMC2_D2,
		MFP033_MMC2_D3,

		MFP_EOC		/*End of configuration*/
	};
	/* configure MFP's */
	mfp_config(mfp_cfg);
	return 0;
}

int misc_init_r(void)
{
	char *env;

	/* primary network interface */
        env = getenv("ethprime");
        if (!env)
		setenv("ethprime", "eth0");

        /* linux boot arguments */
        env = getenv("default_load_addr");
        if (!env)
                setenv("default_load_addr", "0x00500000");

	env = getenv("baudrate");
	if (!env)
		setenv("baudrate", 115200);

	env = getenv("ethaddr");
	if (!env)
		setenv("ethaddr", "00:00:5A:9F:6D:82");

	env = getenv("mmc_bootargs");
	if (!env)
		setenv("mmc_bootargs", "root=/dev/mmcblk0p2 "\
		       "console=ttyS2,115200 uart_dma rootwait");

	env = getenv("mmc_bootcmd");
	if (!env)
		setenv("mmc_bootcmd", "setenv bootargs $(mmc_bootargs);"\
			"mmc sw_dev 1; mmc rescan;"\
		       "fatls mmc 1:1; fatload mmc 1:1 0x0500000 zImage;"\
		       "bootz 0x0500000;");

	env = getenv("usb_bootargs");
	if (!env)
		setenv("usb_bootargs", "root=/dev/sda1 console=ttyS2,115200 "\
		       "uart_dma rootwait");

	env = getenv("usb_bootcmd");
	if (!env)
		setenv("usb_bootcmd", "setenv bootargs $(usb_bootargs);"\
		       "usb start;"\
		       "ext2load usb 0 0x0500000 boot/uImage;"\
		       "bootm 0x0500000;");

	env = getenv("bootcmd");
	if (!env)
		setenv("bootcmd", "run usb_bootcmd");

	env = getenv("ipaddr");
	if (!env)
		setenv("ipaddr", "192.168.7.200");

	env = getenv("serverip");
	if (!env)
		setenv("serverip", "192.168.7.103");

	env = getenv("ddr-test-disable");
	if (!env)
		setenv("ddr-test-disable", "sf erase 0x07BC00 0x100;"\
		       "mw.b 0x500000 0xff 4;"\
		       "mw.b 0x500000 0 4;"\
		       "sf write 0x500000 0x07BC00 4;");

}

int board_init(void)
{
	struct armd1apb2_registers *apb2_regs =
		(struct armd1apb2_registers *)ARMD1_APBC2_BASE;

	/* arch number of Board */
	gd->bd->bi_arch_number = MACH_TYPE_SHEEVAD;
	/* adress of boot parameters */
	gd->bd->bi_boot_params = armd1_sdram_base(0) + 0x100;
	/* Assert PHY_RST# */
	gpio_direction_output(CONFIG_SYS_GPIO_PHY_RST, GPIO_LOW);
	udelay(10);
	/* Deassert PHY_RST# */
	gpio_set_value(CONFIG_SYS_GPIO_PHY_RST, GPIO_HIGH);

	/* Enable SSP2 clock */
	writel(SSP2_APBCLK | SSP2_FNCLK, &apb2_regs->ssp2_clkrst);
	return 0;
}
#ifdef CONFIG_PXASDH
int pxa_sdh_init(bd_t *);
int board_mmc_init(bd_t *bd)
{
	 return pxa_sdh_init(bd);
}
#endif

#ifdef CONFIG_ARMADA100_FEC
int board_eth_init(bd_t *bis)
{
	struct armd1apmu_registers *apmu_regs =
		(struct armd1apmu_registers *)ARMD1_APMU_BASE;

	/* Enable clock of ethernet controller */
	writel(FE_CLK_RST | FE_CLK_ENA, &apmu_regs->fecrc);

	return armada100_fec_register(ARMD1_FEC_BASE);
}

#ifdef CONFIG_RESET_PHY_R
/* Configure and initialize PHY chip 88E3015 */
void reset_phy(void)
{
	u16 phy_adr;
	const char *name = "armd-fec0";

	if (miiphy_set_current_dev(name))
		return;

	/* command to read PHY dev address */
	if (miiphy_read(name, 0xff, 0xff, &phy_adr)) {
		printf("Err..%s could not read PHY dev address\n", __func__);
		return;
	}

	/* Set Ethernet LED in TX blink mode */
	miiphy_write(name, phy_adr, PHY_LED_MAN_REG, 0x00);
	miiphy_write(name, phy_adr, PHY_LED_PAR_SEL_REG, PHY_LED_VAL);

	/* reset the phy */
	miiphy_reset(name, phy_adr);
	debug("88E3015 Initialized on %s\n", name);
}
#endif /* CONFIG_RESET_PHY_R */
#endif /* CONFIG_ARMADA100_FEC */
