/*
 * (C) Copyright 2008-2010 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <spi.h>
#include <asm/errno.h>
#include <linux/types.h>
#include <asm/io.h>
#include <malloc.h>
#include <asm/arch/clock.h>
#include <imx_spi.h>

static inline struct imx_spi_dev_t *to_imx_spi_slave(struct spi_slave *slave)
{
	return container_of(slave, struct imx_spi_dev_t, slave);
}

static s32 spi_reset(struct spi_slave *slave)
{
	u32 clk_src = mxc_get_clock(MXC_CSPI_CLK);
	u32 pre_div = 0, post_div = 0, reg_ctrl, reg_config;
	struct imx_spi_dev_t *dev = to_imx_spi_slave(slave);
	struct spi_reg_t *reg = &(dev->reg);

	if (dev->freq == 0) {
		printf("Error: desired clock is 0\n");
		return 1;
	}

	reg_ctrl = readl(dev->base + SPI_CON_REG);
	reg_config = readl(dev->base + SPI_CFG_REG);
	/* Reset spi */
	writel(0, dev->base + SPI_CON_REG);
	writel((reg_ctrl | 0x1), dev->base + SPI_CON_REG);

	/* Control register setup */
	pre_div = (clk_src + dev->freq - 1) / dev->freq;
	while (pre_div > 16) {
		pre_div = (pre_div + 1) >> 1;
		post_div++;
	}
	if (post_div > 0x0f) {
		printf("Error: no divider can meet the freq: %d\n", dev->freq);
		return -1;
	}
	if (pre_div)
		pre_div--;

	debug("pre_div = %d, post_div=%d, clk_src=%d, spi_freq=%d\n", pre_div,
			post_div, clk_src, (clk_src/(pre_div + 1)) >> post_div);
	reg_ctrl &= ~((3 << 18) | (0xF << 12) | (0xF << 8));
	reg_ctrl |= (dev->ss << 18);
	reg_ctrl |= (pre_div << 12);
	reg_ctrl |= (post_div << 8);
	reg_ctrl |= (1 << (dev->ss + 4));	/* always set to master mode */
	reg_ctrl |= 1;

	/* configuration register setup */
	reg_config &= ~(0x111111 << dev->ss);
	reg_config |= (dev->in_sctl << (dev->ss + 20));
	reg_config |= (dev->in_dctl << (dev->ss + 16));
	reg_config |= (dev->ss_pol << (dev->ss + 12));
	reg_config |= (dev->ssctl << (dev->ss + 8));
	reg_config |= (dev->sclkpol << (dev->ss + 4));
	reg_config |= (dev->sclkpha << (dev->ss));

	reg_config &= 0x0f << 12 ;
	reg_config |= (dev->ss_pol)<<(12+dev->ss);
	debug("ss%x, reg_config = 0x%x\n", dev->ss, reg_config);
	writel(reg_config, dev->base + SPI_CFG_REG);
	debug("ss%x, reg_ctrl = 0x%x\n", dev->ss, reg_ctrl);
	writel(reg_ctrl & ~1, dev->base + SPI_CON_REG);

	/* save config register and control register */
	reg->cfg_reg  = reg_config;
	reg->ctrl_reg = reg_ctrl;

	/* clear interrupt reg */
	writel(0, dev->base + SPI_INT_REG);
	writel(3 << 6, dev->base + SPI_STAT_REG);
	return 0;
}

void spi_init(void)
{
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct imx_spi_dev_t *imx_spi_slave = NULL;

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

	imx_spi_slave = (struct imx_spi_dev_t *)
			calloc(sizeof(struct imx_spi_dev_t), 1);
	if (!imx_spi_slave)
		return NULL;

	imx_spi_slave->slave.bus = bus;
	imx_spi_slave->slave.cs = cs;

	spi_get_cfg(imx_spi_slave);

	if (max_hz < imx_spi_slave->freq)
		imx_spi_slave->freq = max_hz ;
	spi_io_init(imx_spi_slave, 0);

	spi_reset(&(imx_spi_slave->slave));

	return &(imx_spi_slave->slave);
}

void spi_free_slave(struct spi_slave *slave)
{
	struct imx_spi_dev_t *imx_spi_slave;

	if (slave) {
		imx_spi_slave = to_imx_spi_slave(slave);
		free(imx_spi_slave);
	}
}

int spi_claim_bus(struct spi_slave *slave)
{
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{

}

/*
 * SPI xchg
 */

int spi_xchg_single(struct spi_slave *slave, unsigned int bitlen,
	const u8 *dout, u8 *din, unsigned long flags)
{
	int nbytes = (bitlen + 7) / 8;
	struct imx_spi_dev_t *dev = to_imx_spi_slave(slave);
	struct spi_reg_t *spi_reg = &(dev->reg);
	u32 loop_cnt ;
	if (!slave)
		return -1;

	if (spi_reg->ctrl_reg == 0) {
		printf("Error: spi(base=0x%x) has not been initialized yet\n",
				dev->base);
		return -1;
	}
	spi_reg->ctrl_reg = (spi_reg->ctrl_reg & ~0xFFF00000) | \
			((bitlen - 1) << 20);

	writel(spi_reg->ctrl_reg, dev->base + SPI_CON_REG);
	writel(spi_reg->cfg_reg, dev->base + SPI_CFG_REG);

	/* move data to the tx fifo */
	debug("dout=0x%p, bitlen=%x\n", dout, bitlen);

	/*
	 * The SPI controller works only with words,
	 * check if less than a word is sent.
	 * Access to the FIFO is only 32 bit
	 */
	if (bitlen % 32) {
		u32 data = 0;
		u32 cnt = (bitlen % 32) / 8;
		if (dout) {
			int i ;
			for (i = 0; i < cnt; i++)
				data = (data << 8) | (*dout++ & 0xFF);
		}
		debug("Sending SPI 0x%x\n", data);

		writel(data, dev->base + SPI_TX_DATA);
		nbytes -= cnt;
	}

	while (nbytes > 0) {
		u32 data = 0;
		if (dout) {
			/* Buffer is not 32-bit aligned */
			if ((unsigned long)dout & 0x03) {
				int i ;
				data = 0;
				for (i = 0; i < 4; i++)
					data = (data << 8) | (*dout++ & 0xFF);
			} else {
				data = *(u32 *)dout;
				data = cpu_to_be32(data);
			}
			dout += 4;
		}
		debug("Sending SPI 0x%x\n", data);
		writel(data, dev->base + SPI_TX_DATA);
		nbytes -= 4;
	}

	writel(spi_reg->ctrl_reg | (1 << 2), dev->base + SPI_CON_REG);

	loop_cnt = SPI_RETRY_TIMES ;
	/* poll on the TC bit (transfer complete) */
	while ((readl(dev->base + SPI_STAT_REG) & (1 << 7)) == 0) {
		loop_cnt--;
		if (loop_cnt <= 0) {
			printf("%s: Error: re-tried %d times\n",
				__func__, SPI_RETRY_TIMES);
			bitlen = 0 ;
			spi_cs_deactivate(slave);
			return -1;
		}
		udelay(100);
	}

	/* clear the TC bit */
	writel(3 << 6, dev->base + SPI_STAT_REG);

	nbytes = (bitlen + 7) / 8;

	if (bitlen % 32) {
		u32 data = readl(dev->base + SPI_RX_DATA);
		u32 cnt = (bitlen % 32) / 8;
		data = cpu_to_be32(data) >> ((sizeof(data) - cnt) * 8);
		debug("SPI Rx unaligned: 0x%x\n", data);
		if (din) {
			memcpy(din, &data, cnt);
			din += cnt;
		}
		nbytes -= cnt;
	}

	while (nbytes > 0) {
		u32 tmp = readl(dev->base + SPI_RX_DATA);
		u32 data = cpu_to_be32(tmp);
		u32 cnt = min(nbytes, sizeof(data));
		debug("SPI Rx: 0x%x 0x%x\n", tmp, data);
		if (din) {
			memcpy(din, &data, cnt);
			din += cnt;
		}
		nbytes -= cnt;
	}

	return 0;
}

/*
 * SPI transfer:
 *
 * See include/spi.h and http://www.altera.com/literature/ds/ds_nios_spi.pdf
 * for more informations.
 */
int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	int n_bytes = (bitlen + 7) / 8;
	int n_bits;
	int ret;
	u32 blk_size;
	u8 *p_outbuf = (u8 *)dout;
	u8 *p_inbuf = (u8 *)din;

	if (!slave)
		return -1;

	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(slave);

	while (n_bytes > 0) {
		if (n_bytes < MAX_SPI_BYTES)
			blk_size = n_bytes;
		else
			blk_size = MAX_SPI_BYTES;

		n_bits = blk_size * 8;

		ret = spi_xchg_single(slave, n_bits, p_outbuf, p_inbuf, 0);

		if (ret)
			return ret;
		if (dout)
			p_outbuf += blk_size;
		if (din)
			p_inbuf += blk_size;
		n_bytes -= blk_size;
	}

	if (flags & SPI_XFER_END)
		spi_cs_deactivate(slave);

	return 0;
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return 1;
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct imx_spi_dev_t *dev = to_imx_spi_slave(slave);

	spi_io_init(dev, 1);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct imx_spi_dev_t *dev = to_imx_spi_slave(slave);
	spi_io_init(dev, 0);
	writel(0, dev->base + SPI_CON_REG);
}
