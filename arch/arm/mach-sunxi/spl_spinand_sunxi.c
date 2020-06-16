// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Benedikt-Alexander Mokroß <bam@icognize.de>
 */

#define DEBUG

#include <common.h>
#include <spl.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
#error CONFIG_SPL_OS_BOOT is not supported yet
#endif

/*
 * This is a very simple U-Boot image loading implementation, trying to
 * replicate what the boot ROM is doing when loading the SPL. Because we
 * know the exact pins where the SPI Flash is connected and also know
 * that the Read Data Bytes (03h) command is supported, the hardware
 * configuration is very simple and we don't need the extra flexibility
 * of the SPI framework. Moreover, we rely on the default settings of
 * the SPI controler hardware registers and only adjust what needs to
 * be changed. This is good for the code size and this implementation
 * adds less than 400 bytes to the SPL.
 *
 * There are two variants of the SPI controller in Allwinner SoCs:
 * A10/A13/A20 (sun4i variant) and everything else (sun6i variant).
 * Both of them are supported.
 *
 * The pin mixing part is SoC specific and only A10/A13/A20/H3/A64 are
 * supported at the moment.
 */

/*****************************************************************************/
/* SUN4I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN4I_SPI0_CCTL             (0x01C05000 + 0x1C)
#define SUN4I_SPI0_CTL              (0x01C05000 + 0x08)
#define SUN4I_SPI0_RX               (0x01C05000 + 0x00)
#define SUN4I_SPI0_TX               (0x01C05000 + 0x04)
#define SUN4I_SPI0_FIFO_STA         (0x01C05000 + 0x28)
#define SUN4I_SPI0_BC               (0x01C05000 + 0x20)
#define SUN4I_SPI0_TC               (0x01C05000 + 0x24)

#define SUN4I_CTL_ENABLE            BIT(0)
#define SUN4I_CTL_MASTER            BIT(1)
#define SUN4I_CTL_TF_RST            BIT(8)
#define SUN4I_CTL_RF_RST            BIT(9)
#define SUN4I_CTL_XCH               BIT(10)

/*****************************************************************************/
/* SUN6I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN6I_SPI0_CCTL             (0x01C68000 + 0x24)
#define SUN6I_SPI0_GCR              (0x01C68000 + 0x04)
#define SUN6I_SPI0_TCR              (0x01C68000 + 0x08)
#define SUN6I_SPI0_FIFO_STA         (0x01C68000 + 0x1C)
#define SUN6I_SPI0_WCR				(0x01C68000 + 0x20)
#define SUN6I_SPI0_MBC              (0x01C68000 + 0x30)
#define SUN6I_SPI0_MTC              (0x01C68000 + 0x34)
#define SUN6I_SPI0_BCC              (0x01C68000 + 0x38)
#define SUN6I_SPI0_TXD              (0x01C68000 + 0x200)
#define SUN6I_SPI0_RXD              (0x01C68000 + 0x300)

#define SUN6I_CTL_ENABLE            BIT(0)
#define SUN6I_CTL_MASTER            BIT(1)
#define SUN6I_CTL_SRST              BIT(31)
#define SUN6I_TCR_XCH               BIT(31)

/*****************************************************************************/

#define CCM_AHB_GATING0             (0x01C20000 + 0x60)
#define CCM_SPI0_CLK                (0x01C20000 + 0xA0)
#define SUN6I_BUS_SOFT_RST_REG0     (0x01C20000 + 0x2C0)

#define AHB_RESET_SPI0_SHIFT        20
#define AHB_GATE_OFFSET_SPI0        20

#define SPI0_CLK_DIV_NONE			0x0000
#define SPI0_CLK_DIV_BY_2           0x1000
#define SPI0_CLK_DIV_BY_4           0x1001

#define DUMMY_BURST_BYTE 			0x00

#ifndef CONFIG_SPL_SPINAND_SUNXI_SPL_SIZE
#define CONFIG_SPL_SPINAND_SUNXI_SPL_SIZE 		0x6000
#endif
#ifndef CONFIG_SPL_SPINAND_SUNXI_UBOOT_PADDING
#define CONFIG_SPL_SPINAND_SUNXI_UBOOT_PADDING 	0x2000
#endif
#ifndef CONFIG_SPL_SPINAND_SUNXI_PAGESIZE
#define CONFIG_SPL_SPINAND_SUNXI_PAGESIZE 		2048
#endif
#ifndef CONFIG_SYS_SPI_U_BOOT_OFFS
#define CONFIG_SYS_SPI_U_BOOT_OFFS (CONFIG_SPL_SPINAND_SUNXI_SPL_SIZE * (CONFIG_SPL_SPINAND_SUNXI_PAGESIZE / 1024)) + CONFIG_SPL_SPINAND_SUNXI_UBOOT_PADDING
#endif

#undef CONFIG_SYS_SPI_U_BOOT_OFFS
#define CONFIG_SYS_SPI_U_BOOT_OFFS 0x20000

/*****************************************************************************/

/*
 * Allwinner A10/A20 SoCs were using pins PC0,PC1,PC2,PC23 for booting
 * from SPI Flash, everything else is using pins PC0,PC1,PC2,PC3.
 */
static void spi0_pinmux_setup(unsigned int pin_function)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(2); pin++)
		sunxi_gpio_set_cfgpin(pin, pin_function);

	if (IS_ENABLED(CONFIG_MACH_SUN4I) || IS_ENABLED(CONFIG_MACH_SUN7I))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(23), pin_function);
	else
		sunxi_gpio_set_cfgpin(SUNXI_GPC(3), pin_function);
}

/*
 * Setup 24 MHz from OSC24M.
 */
static void spi0_enable_clock(void)
{
	/* Deassert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		setbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));

	/* Open the SPI0 gate */
	setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* No devide */
	writel(SPI0_CLK_DIV_NONE, IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ?
				  SUN6I_SPI0_CCTL : SUN4I_SPI0_CCTL);
	/* 24MHz from OSC24M */
	writel((1 << 31), CCM_SPI0_CLK);

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		/* Enable SPI in the master mode and do a soft reset */
		setbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE |
					     SUN6I_CTL_SRST);
		/* Wait for completion */
		while (readl(SUN6I_SPI0_GCR) & SUN6I_CTL_SRST)
			;
	} else {
		/* Enable SPI in the master mode and reset FIFO */
		setbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE |
					     SUN4I_CTL_TF_RST |
					     SUN4I_CTL_RF_RST);
	}
}

static void spi0_disable_clock(void)
{
	/* Disable the SPI0 controller */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE);
	else
		clrbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE);

	/* Disable the SPI0 clock */
	writel(0, CCM_SPI0_CLK);

	/* Close the SPI0 gate */
	clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* Assert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));
}

static void spi0_init(void)
{
	unsigned int pin_function = SUNXI_GPC_SPI0;
	if (IS_ENABLED(CONFIG_MACH_SUN50I))
		pin_function = SUN50I_GPC_SPI0;

	spi0_pinmux_setup(pin_function);
	spi0_enable_clock();

	writel(0x01, SUN6I_SPI0_WCR);
}

static void spi0_deinit(void)
{
	/* New SoCs can disable pins, older could only set them as input */
	unsigned int pin_function = SUNXI_GPIO_INPUT;
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		pin_function = SUNXI_GPIO_DISABLE;

	spi0_disable_clock();
	spi0_pinmux_setup(pin_function);
}

/*****************************************************************************/

#define SPI_READ_MAX_SIZE 60 /* FIFO size, minus 4 bytes of the header */

static void sunxi_spi0_load_page(u32 addr, ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg) {

    /* Read Page in Cache */
	u8 status = 0x01;
	addr = addr >> 11;

	//printf("sunxi SPI-NAND: Load Page 0x%x\n", addr);
	writel(4, spi_bc_reg); /* Burst counter (total bytes) */
	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(4, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read Data Bytes (13h) command header */
	writeb(0x13, spi_tx_reg);
	writeb((u8)(addr >> 16), spi_tx_reg);
	writeb((u8)(addr >> 8), spi_tx_reg);
	writeb((u8)(addr), spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait till all bytes are send */
	while((readl(spi_fifo_reg) & 0x7F0000) > 0)
		;

    /* wait till all bytes are read */
	while ((readl(spi_fifo_reg) & 0x7F) < 4)
		;

	/* Discard the 4 empty bytes from our send */
	readl(spi_rx_reg);

	/* tCS = 100ns + tRD_ECC 70ns -> 200ns wait */
	ndelay(200);

	do {
		/* Poll */
		writel(2 + 1, spi_bc_reg);   /* Burst counter (total bytes) */
		writel(2, spi_tc_reg);       /* Transfer counter (bytes to send) */
		if (spi_bcc_reg)
			writel(2, spi_bcc_reg);  /* SUN6I also needs this */
		/* Send the Read Status Bytes (0FC0h) command header */
		writeb(0x0F, spi_tx_reg);
		writeb(0xC0, spi_tx_reg);

		/* Start the data transfer */
		setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

		while ((readl(spi_fifo_reg) & 0x7F) < 2 + 1)
			;

		/* skip 2 */
		// printf("Skip %x\n",readb(spi_rx_reg));
		// printf("Skip %x\n",readb(spi_rx_reg));
	    readb(spi_rx_reg);
		readb(spi_rx_reg);

		status = readb(spi_rx_reg);
		ndelay(200);
	} while ((status & 0x01) == 0x01);

}

static void spi0_load_page(u32 addr)
{
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		sunxi_spi0_load_page(addr,
				     SUN6I_SPI0_TCR,
				     SUN6I_TCR_XCH,
				     SUN6I_SPI0_FIFO_STA,
				     SUN6I_SPI0_TXD,
				     SUN6I_SPI0_RXD,
				     SUN6I_SPI0_MBC,
				     SUN6I_SPI0_MTC,
				     SUN6I_SPI0_BCC);
	} else {
		sunxi_spi0_load_page(addr,
				     SUN4I_SPI0_CTL,
				     SUN4I_CTL_XCH,
				     SUN4I_SPI0_FIFO_STA,
				     SUN4I_SPI0_TX,
				     SUN4I_SPI0_RX,
				     SUN4I_SPI0_BC,
				     SUN4I_SPI0_TC,
				     0);
	}
}

static void sunxi_spi0_read_data(u8 *buf, u32 addr, u32 bufsize,
				 ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
{
	addr = addr & 0x07FF;
	//printf("sunxi SPI-NAND: Read %d bytes from cache at 0x%x\n", bufsize, addr);
	writel(4 + bufsize, spi_bc_reg); /* Burst counter (total bytes) */
	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(4, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read Data Bytes (0Bh) command header */
	writeb(0x0B, spi_tx_reg);
	writeb((u8)((addr >> 8)), spi_tx_reg);
	writeb((u8)(addr), spi_tx_reg);
	writeb(DUMMY_BURST_BYTE, spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < 4 + bufsize)
		;

	/* Skip 4 bytes since we send 4 */
	readl(spi_rx_reg);
	//readb(spi_rx_reg);
	//readb(spi_rx_reg);

	/* Read the data */
	while (bufsize-- > 0)
		*buf++ = readb(spi_rx_reg);

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	ndelay(100);
}

static void sunxi_spi0_read_cache(void *buf, u32 addr, u32 len) {

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
			sunxi_spi0_read_data(buf, addr, len,
					     SUN6I_SPI0_TCR,
					     SUN6I_TCR_XCH,
					     SUN6I_SPI0_FIFO_STA,
					     SUN6I_SPI0_TXD,
					     SUN6I_SPI0_RXD,
					     SUN6I_SPI0_MBC,
					     SUN6I_SPI0_MTC,
					     SUN6I_SPI0_BCC);
		} else {
			sunxi_spi0_read_data(buf, addr, len,
					     SUN4I_SPI0_CTL,
					     SUN4I_CTL_XCH,
					     SUN4I_SPI0_FIFO_STA,
					     SUN4I_SPI0_TX,
					     SUN4I_SPI0_RX,
					     SUN4I_SPI0_BC,
					     SUN4I_SPI0_TC,
					     0);
		}
}

static void spi0_read_data(void *buf, u32 addr, u32 len)
{
	u8 *buf8 = buf;
	u32 chunk_len;
	u32 last_page = addr >> 11;
	u32 curr_page;

	spi0_load_page(addr);

	while (len > 0) {
		curr_page = addr >> 11;
		if(curr_page > last_page) {
			spi0_load_page(addr);
			last_page = curr_page;
		}

		chunk_len = len;
		if (chunk_len > SPI_READ_MAX_SIZE) {
			chunk_len = SPI_READ_MAX_SIZE;
		}

		if(((addr + chunk_len) >> 11) > curr_page) {
			chunk_len = ((curr_page + 1) << 11) - addr;
		}

		sunxi_spi0_read_cache(buf8, addr, chunk_len);
		len  -= chunk_len;
		buf8 += chunk_len;
		addr += chunk_len;
	}
}

static int sunxi_spi0_read_id(ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
{
	u8 idbuf[2];
	writel(2 + 2, spi_bc_reg); /* Burst counter (total bytes) */
	writel(2, spi_tc_reg);     /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(2, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read ID Bytes (9Fh) command header */
	writeb(0x9F, spi_tx_reg);
	writeb(DUMMY_BURST_BYTE, spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < 2 + 2)
		;

	/* Skip 2 bytes */
	readb(spi_rx_reg);
	readb(spi_rx_reg);

	/* Read the data */
	//while (bufsize-- > 0)
	idbuf[0] = readb(spi_rx_reg);
	idbuf[1] = readb(spi_rx_reg);

	// printf("NAND ID: %x %x\n", idbuf[0],  idbuf[1]);

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	udelay(1);

	return idbuf[0] | (idbuf[1] << 8);
}

static int spi0_read_id(void) {
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		return sunxi_spi0_read_id(
				     SUN6I_SPI0_TCR,
				     SUN6I_TCR_XCH,
				     SUN6I_SPI0_FIFO_STA,
				     SUN6I_SPI0_TXD,
				     SUN6I_SPI0_RXD,
				     SUN6I_SPI0_MBC,
				     SUN6I_SPI0_MTC,
				     SUN6I_SPI0_BCC);
	} else {
		return sunxi_spi0_read_id(
				     SUN4I_SPI0_CTL,
				     SUN4I_CTL_XCH,
				     SUN4I_SPI0_FIFO_STA,
				     SUN4I_SPI0_TX,
				     SUN4I_SPI0_RX,
				     SUN4I_SPI0_BC,
				     SUN4I_SPI0_TC,
				     0);
	}
}

/*****************************************************************************/

static int spl_spi_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	int ret = 0;
	int id = 0;
	struct image_header *header;
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	spi0_init();
	id = spi0_read_id();

	switch(id) {
		default:
			printf("sunxi SPI-NAND: Unknown chip %x\n", id);
			return -1;
		case 0x12C2:
			printf("sunxi SPI-NAND: Found 1Gb Macronix MX35LF1GE4AB (%x)\n", id);
			break;
		case 0x22C2:
			printf("sunxi SPI-NAND: Found 2Gb Macronix MX35LF2GE4AB (%x)\n", id);
			break;
		case 0xAA21:
			printf("sunxi SPI-NAND: Found 1Gb Winbond W25N01GVxxIG (%x)\n", id);
			break;
	}

	spi0_read_data((void *)header, CONFIG_SYS_SPI_U_BOOT_OFFS, 0x40);
	ret = spl_parse_image_header(spl_image, header);
	if (ret) {
		printf("spl_parse_image_header: %x\n", ret);
		return ret;
	}

	spi0_read_data((void *)spl_image->load_addr, CONFIG_SYS_SPI_U_BOOT_OFFS, spl_image->size);

	spi0_deinit();
	return ret;
}
/* Use priorty 0 to override the default if it happens to be linked in */
SPL_LOAD_IMAGE_METHOD("sunxi SPI-NAND", 0, BOOT_DEVICE_SPI, spl_spi_load_image);
