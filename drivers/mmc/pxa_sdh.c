/**************************************************************************
 *
 * Copyright (c) 2009, 2010 Marvell International Ltd.
 *
 * This file is part of GNU program.
 *
 * GNU program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or (at your
 * option) any later version.
 *
 * GNU program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 *
 *************************************************************************/

/*
 *  Copyright (C) Marvell International Ltd. (kvedere@marvell.com)
 *  Code heavily based on Linux driver
 *  	/driver/mmc/host/pxa_sdh.c
 *  Copyright (C) 2008-2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <common.h>
#include <part.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/io.h>
#include <asm/errno.h>
//#include <linux/compat.h>

#include "pxa_sdh.h"

#define CLKRT_OFF			(~0)
#define GET_REG(host, reg) 		readw(host->regbase + reg)
#define SET_REG(host, val, reg)		writew(val, host->regbase + reg)
#define DATA_DIRECTION_READ(data) 	(data->flags & MMC_DATA_READ)
#define SET_REG_BIT(host, bit_mask, reg) \
		SET_REG(host, \
			GET_REG(host, reg) | (bit_mask), reg)
#define CLEAR_REG_BIT(host, bit_mask, reg) \
		SET_REG(host, \
			GET_REG(host, reg) & ~(bit_mask), reg)
#define SET_REG_BITS(host, bits_pos, bits_mask, val, reg) \
		SET_REG(host, \
			GET_REG(host, reg) & ~(bits_mask << bits_pos), reg); \
		SET_REG(host, \
			GET_REG(host, reg) | (val << bits_pos), reg)
#define GET_REG_BITS(host, bit_pos, bits_mask, reg) \
		((GET_REG(host, reg) >> bit_pos) & bits_mask)
#define mmc_resp_type(cmd) ((cmd)->resp_type & (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC|MMC_RSP_BUSY|MMC_RSP_OPCODE))
#define MMC_BUS_WIDTH_1		1
#define MMC_BUS_WIDTH_4		4
#define MMC_BUS_WIDTH_8		8
#define CONFIG_MMC_SDMA		1

//#define CONFIG_MMC_DEBUG	1

#ifdef CONFIG_MMC_DEBUG
#define mmc_dbg printf
#else
#define mmc_dbg(arg...)
#endif

struct pxa_sdh_host {
	struct mmc_cmd	*cmd;
	struct mmc_data *data;
	u32		regbase;
	u32		bytes_xfered;
	u32		clkrate;
	u32		clkrt;
	u32		data_len;
	int		error;
	int		port_num;
};

static int pxa_sdh_cmd_done(struct pxa_sdh_host *host);
static void pxa_sdh_dma_data_done(struct pxa_sdh_host *host);
static void pxa_sdh_data_done(struct pxa_sdh_host *host);

#ifdef CONFIG_MMC_DEBUG
static void dump_registers(struct pxa_sdh_host *host)
{
	unsigned int val;
	int offset;

	for (offset = 0; offset < 0x60; offset += 4) {
		if (offset == 0x20)
			continue;
		val = readl(host->regbase + offset);
		printf("%08x: %08x\n", (unsigned int)host->regbase + offset, val);
	}
	for (offset = 0xE0; offset < 0xF0; offset += 4) {
		val = readl(host->regbase + offset);
		printf("%08x: %08x\n", (unsigned int)host->regbase + offset, val);
	}
	val = readl(host->regbase + 0xFC);
	printf("%08x: %08x\n", (unsigned int)host->regbase + 0xFC, val);
}
#endif

static inline int pxa_sdh_fls(int x)
{
	int ret;

	asm("clz\t%0, %1" : "=r" (ret) : "r" (x) : "cc");
	ret = 32 - ret;
	return ret;
}

static int pxa_sdh_wait_reset(struct pxa_sdh_host *host)
{
	u32 timeout = 1000;
	u16 val;

	do {
		val = GET_REG(host, SD_TO_CTRL_SW_RST);
		if (!(val & (SW_RST_DAT | SW_RST_CMD | SW_RST_ALL)))
			break;
		udelay(1);
	} while (timeout--);
	if (timeout)
		return 0;

	printf("%s: Fatal: Wait RESET timeout.\n",__func__);

	return 1;
}

static void pxa_sdh_stop_clock(struct pxa_sdh_host *host)
{
	CLEAR_REG_BIT(host, EXT_CLK_EN, SD_CLOCK_CNTL);
}

static void pxa_sdh_start_clock(struct pxa_sdh_host *host)
{
	u32 timeout = 1000;

	SET_REG_BIT(host, INT_CLK_EN, SD_CLOCK_CNTL);
	do {
		if (GET_REG(host, SD_CLOCK_CNTL) & INT_CLK_STABLE)
			break;
		udelay(1);
	} while (timeout--);
	if (!timeout)
		printf("%s: unable to start clock\n",__func__);

	SET_REG_BITS(host, SD_FREQ_SEL_OFFSET, SD_FREQ_SEL_MASK,
		host->clkrt, SD_CLOCK_CNTL);

	/* set as maximum value for data line timeout*/
	SET_REG_BITS(host, DAT_TO_VAL_OFFSET, DAT_TO_MASK,
		(DAT_TO_MASK - 1), SD_TO_CTRL_SW_RST);

	SET_REG_BIT(host, EXT_CLK_EN, SD_CLOCK_CNTL);
}

static void pxa_sdh_setup_sdma(struct pxa_sdh_host *host)
{
	struct mmc_data* data = host->data;
	if (DATA_DIRECTION_READ(data)) {
		char* dest = data->dest + host->bytes_xfered;
		SET_REG(host, (u32) dest & 0xffff, SD_SYS_ADDR_LOW);
		SET_REG(host, (u32) dest >> 16, SD_SYS_ADDR_HIGH);
	}
	else {
		char* src = (char*) data->src + host->bytes_xfered;
		SET_REG(host, (u32) src & 0xffff, SD_SYS_ADDR_LOW);
		SET_REG(host, (u32) src >> 16, SD_SYS_ADDR_HIGH);
	}

}

static void pxa_sdh_setup_data(struct pxa_sdh_host *host)
{
#ifdef CONFIG_MMC_SDMA
	pxa_sdh_setup_sdma(host);
	SET_REG_BITS(host, DMA_SEL_OFFSET, DMA_SEL_MASK, DMA_SEL_SDMA, SD_HOST_CTRL);
#endif
}

static void pxa_sdh_finish_request(struct pxa_sdh_host *host)
{

#ifdef CONFIG_MMC_DEBUG
	struct mmc_cmd* cmd = host->cmd;
	dump_registers(host);
#endif
	if (host->data && host->error)
		SET_REG_BIT(host, SW_RST_DAT, SD_TO_CTRL_SW_RST);

	mmc_dbg("%s: Finishing CMD%d(%s)\n",__func__, cmd->cmdidx,
		(host->error ? "failed" : "done"));

	host->data = NULL;
	host->cmd = NULL;
	host->error = 0;
	host->data_len = 0;
	host->bytes_xfered = 0;
}

static int pxa_sdh_process_irq(struct pxa_sdh_host *host,u32 intr_type)
{
	ulong hz = 3250000; /* 3.25 MHz Timer Clock */
	ushort done = GET_REG(host, SD_NOR_I_STAT) & intr_type;
        ulong start, curr, diff;
#ifdef CONFIG_MMC_DEBUG
	struct mmc_cmd* cmd = host->cmd;
#endif

	start = get_ticks();
        while(!done)
        {
                done = GET_REG(host, SD_NOR_I_STAT) & intr_type;
                if( GET_REG(host, SD_NOR_I_STAT) & 0x8000 )
                {
                        mmc_dbg("Error! cmd : %d, err : %04x\n", cmd->cmdidx, GET_REG(host, SD_ERR_I_STAT));
			host->error = 1;
			pxa_sdh_finish_request(host);
			if (GET_REG(host, SD_ERR_I_STAT) & 0x1)
				return TIMEOUT;      /* error happened */
			else
				return COMM_ERR;
                }
                curr = get_ticks();
                diff = (long) curr - (long) start;
                if (diff > (3*hz))
                {
                        printf("cmd timeout, status : %04x\n", GET_REG(host, SD_NOR_I_STAT));
                        printf("xfer mode : %04x\n", GET_REG(host, SD_TRANS_MODE));
			host->error = 1;
			pxa_sdh_finish_request(host);
                        return -ETIMEDOUT;
                }
        }

	if (intr_type & CMD_COMP)
		pxa_sdh_cmd_done(host);

#ifdef CONFIG_MMC_SDMA
	if (intr_type & DMA_INT)
		pxa_sdh_dma_data_done(host);
#else
	if ((cmdtype & TX_RDY) || (cmdtype & RX_RDY))
		pxa_sdh_pio_data_done(host);
#endif

	if (intr_type & XFER_COMP)
		pxa_sdh_data_done(host);

	/* Clear Status Bits */
	SET_REG(host, intr_type, SD_NOR_I_STAT);
	return 0;
}

static int pxa_sdh_start_cmd(struct pxa_sdh_host *host)
{
	u16 resp = 0;
	u16 xfrmd_val = 0;
	u16 cmd_val = 0;
	u16 val, mask;
        struct mmc_data *data = host->data;
        struct mmc_cmd *cmd = host->cmd;
	int ret;

	BUG_ON(!cmd);

	/*Set Response Type*/
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;

	case MMC_RSP_R1: /* r1, r5, r6, r7 */
		resp = CMD_RESP_48BIT;
		cmd_val |= CMD_CRC_CHK_EN | CMD_IDX_CHK_EN;
		break;

	case MMC_RSP_R2: /* r2 */
		resp = CMD_RESP_136BIT;
		cmd_val |= CMD_CRC_CHK_EN;
		break;

	case MMC_RSP_R3: /* r3, r4*/
		resp = CMD_RESP_48BIT;
		break;

	case MMC_RSP_R1b: /* r1b */
		resp = CMD_RESP_48BITB;
		cmd_val |= CMD_CRC_CHK_EN | CMD_IDX_CHK_EN;
		break;

	default:
		break;
	}

	/*Set Transfer mode regarding to data flag*/
	if (data)
	{
		cmd_val |= DATA_PRESENT;
		xfrmd_val |= BLK_CNT_EN;
		if (data->blocks > 1)
			xfrmd_val |= MULTI_BLK_SEL;
#ifdef CONFIG_MMC_SDMA
		xfrmd_val |= DMA_EN;
#else
		xfrmd_val &= ~DMA_EN;
#endif
		if (DATA_DIRECTION_READ(data))
			xfrmd_val |= TO_HOST_DIR;
		else
			xfrmd_val &= ~TO_HOST_DIR;
	}

	//if (cmd->opcode == 12)
	//	cmd_val |= host, CMD_TYPE_OFFSET, CMD_TYPE_MASK, CMD_TYPE_ABORT, SD_COMMAND);
	SET_REG(host, cmd->cmdarg & 0xffff, SD_ARG_LOW);
	SET_REG(host, cmd->cmdarg >> 16, SD_ARG_HIGH);
	SET_REG(host, xfrmd_val, SD_TRANS_MODE);
	cmd_val |= cmd->cmdidx << CMD_IDX_OFFSET | resp << RESP_TYPE_OFFSET;
	mmc_dbg("%s:Starting CMD%d with ARGUMENT 0x%x\n",__func__, cmd->cmdidx, cmd->cmdarg);

	val = GET_REG(host, SD_PRESENT_STAT_2);
	mask = CMD_LINE_LEVEL_MASK | DATA_LINE_LEVEL_MASK;
	if ((val & mask) != mask)
		mmc_dbg("%s:WARN: CMD/DATA pins are not all high, PRE_STAT=0x%04x\n",__func__,
			GET_REG(host, SD_PRESENT_STAT_2));

	SET_REG(host, cmd_val, SD_COMMAND);

	ret = pxa_sdh_process_irq(host, CMD_COMP);
	if (!ret && data) {
		do {
			ret = pxa_sdh_process_irq(host, (DMA_INT | XFER_COMP));
			/* If error or xfer completed (in which case
			 * bytes_xfered is reset to 0) break */
		} while (host->bytes_xfered);
	}
	return ret;
}



static int pxa_sdh_cmd_done(struct pxa_sdh_host *host)
{
	struct mmc_cmd *cmd = host->cmd;
	u32 resp[8];

	BUG_ON(!cmd);

	/* get cmd response */
	resp[0] = GET_REG(host, SD_RESP_0);
	resp[1] = GET_REG(host, SD_RESP_1);
	resp[2] = GET_REG(host, SD_RESP_2);
	resp[3] = GET_REG(host, SD_RESP_3);
	resp[4] = GET_REG(host, SD_RESP_4);
	resp[5] = GET_REG(host, SD_RESP_5);
	resp[6] = GET_REG(host, SD_RESP_6);
	resp[7] = readb(host->regbase + SD_RESP_7);

	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = resp[5] >> 8 | resp[6] << 8 | resp[7] << 24;
		cmd->response[1] = resp[3] >> 8 | resp[4] << 8 | resp[5] << 24;
		cmd->response[2] = resp[1] >> 8 | resp[2] << 8 | resp[3] << 24;
		cmd->response[3] = resp[0] << 8 | resp[1] << 24;
	} else {
		cmd->response[0] = resp[1] << 16 | resp[0];
		cmd->response[1] = resp[3] << 16 | resp[2];
		cmd->response[2] = resp[5] << 16 | resp[4];
		cmd->response[3] = resp[7] << 16 | resp[6];
	}


	mmc_dbg("%s: resp[0]=0x%x resp[1]=0x%x resp[2]=0x%x resp[3]=0x%x\n",__func__,
                cmd->response[0], cmd->response[1], cmd->response[2], cmd->response[3]);
	if (host->error || !host->data) {
		pxa_sdh_finish_request(host);
	}

	return 1;
}

#ifndef CONFIG_MMC_SDMA
static void pxa_sdh_pio_data_done(struct pxa_sdh_host *host)
{
	struct mmc_data *data = host->data;
	u16 blk_size = data->blocksize;
	u16 i = 0;

	if (DATA_DIRECTION_READ(data)) {
		char* dest = data->dest + host->bytes_xfered;
		for (i = 0; i < blk_size; i += sizeof(u32)) {
			*(u32*)(dest + i) = readl(host->regbase + SD_BUF_DPORT_0);
		}
	} else {
		char* src = (char*) data->src + host->bytes_xfered;
		for (i = 0; i < blk_size; i += sizeof(u32)) {
			writel(*(u32*)(src +  i), host->regbase + SD_BUF_DPORT_0);
		}
	}

	if (host->bytes_xfered < host->data_len) {
		host->bytes_xfered = host->bytes_xfered + blk_size;
		pxa_sdh_setup_data(host);
	}
}
#endif

static void pxa_sdh_dma_data_done(struct pxa_sdh_host *host)
{
	struct mmc_data *data = host->data;
	int host_dma_bdry_size;

	if (host->bytes_xfered < host->data_len) {
		host_dma_bdry_size = 0x1000 << (((u16) GET_REG(host, SD_BLOCK_SIZE)) >> HOST_DMA_BDRY_OFFSET);
		if (host_dma_bdry_size < data->blocksize)
			host->bytes_xfered = host->bytes_xfered + host_dma_bdry_size;
		else
			host->bytes_xfered = host->bytes_xfered + data->blocksize;
		pxa_sdh_setup_data(host);
	}
}

static void pxa_sdh_data_done(struct pxa_sdh_host *host)
{
	pxa_sdh_finish_request(host);
}

static int pxa_sdh_request(struct mmc *mmc, struct mmc_cmd *cmd,
                struct mmc_data *data)
{
	struct pxa_sdh_host *host = mmc->priv;
	u16 val;
	u32 timeout = 1000;
	int ret = 0;

	host->data = data;
	host->cmd  = cmd;

	if (pxa_sdh_wait_reset(host)) {
		host->error = 1;
		pxa_sdh_finish_request(host);
		return -ETIMEDOUT;
	}
	do {
		val = GET_REG(host, SD_PRESENT_STAT_1);
		if (!(val & CMD_INHBT_DAT || val & CMD_INHBT_CMD))
			break;
		udelay(1);
	} while (timeout--);
	if (!timeout) {
		printf("%s: In busy, unable to start the request.\n",__func__);
		host->error = 1;
                pxa_sdh_finish_request(host);
		return -EBUSY;
	}

	/* Clear Interrupt/Error status */
	SET_REG(host, 0xffff, SD_NOR_I_STAT);
	SET_REG(host, 0xffff, SD_ERR_I_STAT);

	if (data) {
		mmc_dbg("%s: setup data, blk_sz=%d, blk_cnt=0x%x\n",__func__,
			data->blocksize, data->blocks);
		/* Set the Host DMA Boundary Buffer to Maximum allowed (512 KB) */
		SET_REG(host, ((u16)((HOST_DMA_BDRY_MASK & 0x7) << HOST_DMA_BDRY_OFFSET) | data->blocksize), SD_BLOCK_SIZE);
		SET_REG(host, data->blocks, SD_BLOCK_COUNT);

		pxa_sdh_setup_data(host);
	}

	ret = pxa_sdh_start_cmd(host);
	return ret;
}

static void pxa_sdh_set_ios(struct mmc *mmc)
{
	struct pxa_sdh_host *host = mmc->priv;

	if (pxa_sdh_wait_reset(host))
		return;

	if (mmc->clock) {
		unsigned long rate = host->clkrate;
		unsigned int clk = rate / mmc->clock;
		unsigned int shift;

		BUG_ON((mmc->clock > mmc->f_max) || (mmc->clock < mmc->f_min));
		if (mmc->clock >= host->clkrate) {
			host->clkrt = 0x00;
		} else {
			shift = pxa_sdh_fls(clk);
			if (rate / clk > mmc->clock)
				shift++;
			host->clkrt = 1 << (shift - 2);
		}

		mmc_dbg("%s: set clkrt = %08x\n",__func__,host->clkrt);
		pxa_sdh_stop_clock(host);
		pxa_sdh_start_clock(host);

		if((host->clkrt == 0 && host->clkrate > 25000000)
			|| (host->clkrt && (host->clkrate/(host->clkrt*2)) > 25000000)) {
			SET_REG_BIT(host, HI_SPEED_EN, SD_HOST_CTRL);
			mmc_dbg("%s: set as HIGH_SPEED.\n",__func__);
		} else
			CLEAR_REG_BIT(host, HI_SPEED_EN, SD_HOST_CTRL);

	} else {
		pxa_sdh_stop_clock(host);
		if (host->clkrt != CLKRT_OFF) {
			host->clkrt = CLKRT_OFF;
		}
	}

	SET_REG_BITS(host, SDCLK_SEL_OFFSET, SDCLK_SEL_MASK, SDCLK_SEL_INIT_VAL, SD_CLK_BURST_SET);
	SET_REG_BITS(host, SD_BUS_VLT_OFFSET, SD_BUS_VLT_MASK,
			SD_BUS_VLT_18V, SD_HOST_CTRL);
	SET_REG_BIT(host, SD_BUS_POWER, SD_HOST_CTRL);

	if (mmc->bus_width == MMC_BUS_WIDTH_8) {
		SET_REG_BIT(host, MMC_CARD, SD_CE_ATA_2);
		SET_REG_BIT(host, DATA_WIDTH_8BIT, SD_CE_ATA_2);
			mmc_dbg("%s: set as 8_BIT_MODE.\n",__func__);
	} else {
		CLEAR_REG_BIT(host, MMC_CARD, SD_CE_ATA_2);
		CLEAR_REG_BIT(host, DATA_WIDTH_8BIT, SD_CE_ATA_2);
		if (mmc->bus_width == MMC_BUS_WIDTH_4) {
			SET_REG_BIT(host, DATA_WIDTH_4BIT, SD_HOST_CTRL);
				mmc_dbg("%s: set as 4_BIT_MODE.\n",__func__);
		} else {
			CLEAR_REG_BIT(host, DATA_WIDTH_4BIT, SD_HOST_CTRL);
		}
	}
}

static int sdh_init(struct mmc *mmc)
{
	struct pxa_sdh_host* host = mmc->priv;

	switch (host->port_num) {
	case 0:
		/* Setup the MMC/SD(1) Host Controller Clock */
		*(volatile unsigned short*)(0xd4282854) = 0x19;
		udelay(10);
		*(volatile unsigned short*)(0xd4282854) = 0x1b;
		break;
	case 1:
		/* Setup the MMC/SD(2) Host Controller Clock */
		*(volatile unsigned short*)(0xd4282858) = 0x10;
		udelay(10);
		*(volatile unsigned short*)(0xd4282858) = 0x12;
		break;
	case 2:
		/* Setup the MMC/SD(3) Host Controller Clock */
		*(volatile unsigned short*)(0xd42828e0) = 0x19;
		udelay(10);
		*(volatile unsigned short*)(0xd42828e0) = 0x1b;
		break;
	case 3:
		/* Setup the MMC/SD(4) Host Controller Clock */
		*(volatile unsigned short*)(0xd42828e4) = 0x10;
		udelay(10);
		*(volatile unsigned short*)(0xd42828e4) = 0x12;
		break;
	}

	/* Enable Interrupt status */
	SET_REG(host, 0xffff, SD_NOR_I_STAT_EN);
	SET_REG(host, 0xffff, SD_ERR_I_STAT_EN);

	/* Disable interrupt generation */
	SET_REG(host, 0, SD_NOR_INT_EN);
	SET_REG(host, 0, SD_ERR_INT_EN);
	return 0;
}

int pxa_sdh_init(bd_t *bis)
{
	struct mmc *mmc;
	struct pxa_sdh_host *host;
	unsigned int i;
	unsigned int mmc_base[] = {
		CONFIG_SYS_MMC_BASE,
		CONFIG_SYS_MMC1_BASE,
		CONFIG_SYS_MMC2_BASE,
		CONFIG_SYS_MMC3_BASE,
		0
	};

	/* Initialize the controllers clock for SD1 and SD2 */
	if (mmc_base[0] != 0xFF || mmc_base[1] != 0xFF) {
		*(volatile unsigned short*)(0xd4282854) = 0xa;
		udelay(10);
		*(volatile unsigned short*)(0xd4282854) = 0xb;
	}

	/* Initialize the controllers clock for SD3 and SD4 */
	if (mmc_base[2] != 0xFF || mmc_base[3] != 0xFF) {
		*(volatile unsigned short*)(0xd42828e0) = 0xa;
		udelay(10);
		*(volatile unsigned short*)(0xd42828e0) = 0xb;
	}

	for (i = 0; mmc_base[i]; i++) {
		if (mmc_base[i] == 0xFF)
			continue;

		mmc = malloc(sizeof(struct mmc));
		if (!mmc) {
			printf("mmc malloc fail!!\n");
			return -1;
		}

		host = malloc(sizeof(struct pxa_sdh_host));
		if (!host) {
			printf("host malloc fail!!!\n");
			return -1;
		}

		host->regbase = mmc_base[i];
		host->clkrate = 48000000;
		host->clkrt = CLKRT_OFF;
		host->port_num = i;

		strncpy(mmc->name, "pxa-sdh", 7);
		mmc->priv = host;
		mmc->send_cmd = pxa_sdh_request;
		mmc->set_ios = pxa_sdh_set_ios;
		mmc->init = sdh_init;

		/*
		 * Calculate minimum clock rate, rounding up.
		 */
		mmc->f_min = (host->clkrate + SD_FREQ_SEL_MASK) /
				 (SD_FREQ_SEL_MASK + 1);
		mmc->f_max = host->clkrate / 2;
		mmc->ocr = 0xffffffff;
		mmc->voltages = MMC_VDD_27_28 | MMC_VDD_28_29;
#ifndef CONFIG_MMC3
		mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_HS;
#else
		mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_8BIT | MMC_MODE_HS;
#endif

		mmc_register(mmc);
	}

	return 0;
}
