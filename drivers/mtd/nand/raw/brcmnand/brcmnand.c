// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <asm/io.h>
#include <memalign.h>
#include <nand.h>
#include <clk.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/log2.h>
#include <asm/processor.h>
#include <dm.h>

#include "brcmnand.h"
#include "brcmnand_compat.h"

/***********************************************************************
 * Definitions
 ***********************************************************************/

#define DRV_NAME			"bcm-leg-nand"

/* NAND Registers */
#define BLN_REV				0x00
#define BLN_CMD_START			0x04
#define BLN_CMD_EXT_ADDR		0x08
#define BLN_CMD_ADDR			0x0c
#define BLN_CMD_END_ADDR		0x10
#define BLN_CS_SEL			0x14
#define BLN_CS_XOR			0x18
#define BLN_OOB_READ_BASE		0x20
#define BLN_OOB_WRITE_BASE		0x30
#define BLN_ACC_CTL			0x40
#define BLN_CFG				0x44
#define BLN_TIMING1			0x48
#define BLN_TIMING2			0x4c
#define BLN_SEM				0x50
#define BLN_DEV_ID			0x54
#define BLN_BLK_LOCK_STATUS		0x58
#define BLN_INTFC_STATUS		0x5c
#define BLN_CORR_EXT_ADDR		0x60
#define BLN_CORR_ADDR			0x64
#define BLN_UNCORR_EXT_ADDR		0x68
#define BLN_UNCORR_ADDR			0x6c
#define BLN_READ_EXT_ADDR		0x70
#define BLN_READ_ADDR			0x74
#define BLN_PRG_PAGE_EXT_ADDR		0x78
#define BLN_PRG_PAGE_ADDR		0x7c
#define BLN_COPY_BACK_EXT_ADDR		0x80
#define BLN_COPY_BACK_ADDR		0x84
#define BLN_BLK_ERASE_EXT_ADDR		0x88
#define BLN_BLK_ERASE_ADDR		0x8c
#define BLN_INV_READ_EXT_ADDR		0x90
#define BLN_INV_READ_ADDR		0x94
#define BLN_WRITE_PROTECT		0x98

/* NAND Revision */
#define NAND_REV_MASK			GENMASK(15, 0)

/* NAND Commands */
#define CMD_SHIFT			24
#define CMD_NULL			0
#define CMD_PAGE_READ			1
#define CMD_SPARE_READ			2
#define CMD_STATUS_READ			3
#define CMD_PROGRAM_PAGE		4
#define CMD_PROGRAM_SPARE		5
#define CMD_COPY_BACK			6
#define CMD_DEV_ID_READ			7
#define CMD_BLK_ERASE			8
#define CMD_FLASH_RESET			9
#define CMD_BLKS_LOCK			10
#define CMD_BLKS_LOCK_DOWN		11
#define CMD_BLKS_UNLOCK			12
#define CMD_READ_BLKS_LOCK_STATUS	13

/* NAND Acc Control */
#define ACC_CTL_RD_ECC			BIT(31)
#define ACC_CTL_WR_ECC			BIT(30)
#define ACC_CTL_RD_ECC_BLK0		BIT(29)
#define ACC_CTL_FAST_PGM_RDIN		BIT(28)
#define ACC_CTL_RD_ERASED_ECC		BIT(27)
#define ACC_CTL_WR_PREEMPT		BIT(25)
#define ACC_CTL_PAGE_HIT		BIT(24)

/* NAND CS Sel */
#define CS_SEL_LOCK			BIT(31)
#define CS_SEL_AUTO_DEV_ID		BIT(30)
#define CS_SEL_WR_PROT_BLK0		BIT(28)
#define CS_SEL_USE_SHIFT		8
#define CS_SEL_USE_MASK			GENMASK(15, CS_SEL_USE_SHIFT)
#define CS_SEL_SHIFT			0
#define CS_SEL_MASK			GENMASK(7, CS_SEL_SHIFT)

/* NAND CS XOR */
#define CS_XOR_SHIFT			0
#define CS_XOR_MASK			GENMASK(7, CS_XOR_SHIFT)

/* NAND Config */
#define CFG_LOCK			BIT(31)
#define CFG_BLK_SIZE_SHIFT		28
#define CFG_BLK_SIZE_MASK		GENMASK(30, CFG_BLK_SIZE_SHIFT)
#define CFG_DEV_SIZE_SHIFT		24
#define CFG_DEV_SIZE_MASK		GENMASK(27, CFG_DEV_SIZE_SHIFT)
#define CFG_DEV_WIDTH_16		BIT(23)
#define CFG_PAGE_SIZE_SHIFT		20
#define CFG_PAGE_SIZE_MASK		GENMASK(21, CFG_PAGE_SIZE_SHIFT)
#define CFG_FUL_ADDR_SHIFT		16
#define CFG_FUL_ADDR_MASK		GENMASK(18, CFG_FUL_ADDR_SHIFT)
#define CFG_COL_ADDR_SHIFT		12
#define CFG_COL_ADDR_MASK		GENMASK(14, CFG_COL_ADDR_SHIFT)
#define CFG_BLK_ADDR_SHIFT		8
#define CFG_BLK_ADDR_MASK		GENMASK(10, CFG_BLK_ADDR_SHIFT)

/* NAND INT FC */
#define INTFC_CTL_READY			BIT(31)
#define INTFC_FLASH_READY		BIT(30)
#define INTFC_CACHE_VALID		BIT(29)
#define INTFC_SPARE_VALID		BIT(28)
#define INTFC_FLASH_STATUS		GENMASK(7, 0)

#define NAND_CTL_READY			(INTFC_CTL_READY | INTFC_FLASH_READY)
#define NAND_POLL_TOUT_MS		100

/* NAND Flash Cache */
#define FC_SHIFT			9U
#define FC_BYTES			512U
#define FC_WORDS			(FC_BYTES >> 2)

/* NAND Minimum Values */
#define BLN_MIN_PAGESIZE		512
#define BLN_MIN_BLOCKSIZE		(8 * 1024)
#define BLN_MIN_DEVSIZE			(4ULL * 1024 * 1024)

/* Structures */
struct bln_controller {
	struct udevice		*dev;
	struct nand_hw_control	controller;
	void __iomem		*nand_base;
	void __iomem		*nand_fc;
	int			nand_version;

	/* Some SoCs provide custom interrupt status register(s) */
	struct bln_soc	*soc;

	/* Some SoCs have a gateable clock for the controller */
	struct clk		*clk;

	int			cmd_pending;

	/* List of NAND hosts (one for each chip-select) */
	struct list_head	host_list;

	/* in-memory cache of the FLASH_CACHE, used only for some commands */
	u8			flash_cache[FC_BYTES];

	/* Controller revision details */
	u32			max_block_size;
	const u16		*block_sizes;
	u32			max_page_size;
	const u16		*page_sizes;
	u32			max_oob;
};

struct bln_cfg {
	u64			device_size;
	unsigned int		block_size;
	unsigned int		page_size;
	unsigned int		spare_area_size;
	unsigned int		device_width;
	unsigned int		col_adr_bytes;
	unsigned int		blk_adr_bytes;
	unsigned int		ful_adr_bytes;
	unsigned int		ecc_level;
};

struct bln_host {
	struct list_head		node;

	struct nand_chip		chip;
	struct udevice			*pdev;
	int				cs;

	unsigned int			last_cmd;
	unsigned int			last_byte;
	u64				last_addr;
	struct bln_cfg		hwcfg;
	struct bln_controller	*ctrl;
};

/* NAND Controller I/O */
static inline u32 bln_read(struct bln_controller *ctrl, u32 offs)
{
	return brcmnand_readl(ctrl->nand_base + offs);
}

static inline void bln_write(struct bln_controller *ctrl, u32 offs, u32 val)
{
	brcmnand_writel(val, ctrl->nand_base + offs);
}

static inline void bln_rmw(struct bln_controller *ctrl, u32 offs, u32 mask,
			   u32 shift, u32 val)
{
	u32 tmp = bln_read(ctrl, offs);
	tmp &= ~mask;
	tmp |= val << shift;
	bln_write(ctrl, offs, tmp);
}

static inline u32 bln_read_fc(struct bln_controller *ctrl, int word)
{
	return __raw_readl(ctrl->nand_fc + word * 4);
}

static inline void bln_write_fc(struct bln_controller *ctrl, int word,
				   u32 val)
{
	__raw_writel(val, ctrl->nand_fc + word * 4);
}

/* NAND Controller Revision */
static int bln_revision_init(struct bln_controller *ctrl)
{
	static const u16 block_sizes_v2_2[] = { 16, 128, 8, 512, 256, 0 };
	static const u16 block_sizes_v2_1[] = { 16, 128, 8, 512, 0 };
	static const u16 page_sizes_v2_2[] = { 512, 2048, 4096, 0 };
	static const u16 page_sizes_v2_1[] = { 512, 2048, 0 };

	ctrl->nand_version = bln_read(ctrl, BLN_REV) & NAND_REV_MASK;

	/* Only support v2.1-v2.2 */
	if (ctrl->nand_version < 0x0201 ||
	    ctrl->nand_version > 0x0202) {
		dev_err(ctrl->dev, "version %#x not supported\n",
			ctrl->nand_version);
		return -ENODEV;
	}

	/* Page sizes */
	if (ctrl->nand_version >= 0x0202)
		ctrl->page_sizes = page_sizes_v2_2;
	else
		ctrl->page_sizes = page_sizes_v2_1;

	if (ctrl->nand_version < 0x0202)
		ctrl->max_page_size = 2048;
	else
		ctrl->max_page_size = 4096;

	/* Block sizes */
	if (ctrl->nand_version >= 0x0202)
		ctrl->block_sizes = block_sizes_v2_2;
	else
		ctrl->block_sizes = block_sizes_v2_1;

	ctrl->max_block_size = 512 * 1024;

	/* Maximum spare area sector size (per 512B) */
	ctrl->max_oob = 16;

	return 0;
}

static void bln_set_ecc_enabled(struct bln_host *host, int en)
{
	struct bln_controller *ctrl = host->ctrl;
	u32 acc_control = bln_read(ctrl, BLN_ACC_CTL);
	u32 ecc_flags = ACC_CTL_RD_ECC;

	if (en)
		acc_control |= ecc_flags; /* enable RD ECC */
	else
		acc_control &= ~ecc_flags; /* disable RD ECC */

	bln_write(ctrl, BLN_ACC_CTL, acc_control);
}

static int bln_ctrl_poll_status(struct bln_controller *ctrl,
				    u32 mask, u32 expected_val,
				    unsigned long timeout_ms)
{
	unsigned long base, limit;
	u32 val;

	if (!timeout_ms)
		timeout_ms = NAND_POLL_TOUT_MS;

	base = get_timer(0);
	limit = CONFIG_SYS_HZ * timeout_ms / 1000;
	do {
		val = bln_read(ctrl, BLN_INTFC_STATUS);
		if ((val & mask) == expected_val)
			return 0;

		cpu_relax();
	} while (get_timer(base) < limit);

	dev_warn(ctrl->dev, "timeout on status poll (expected %x got %x)\n",
		 expected_val, val & mask);

	return -ETIMEDOUT;
}

/***********************************************************************
 * Internal support functions
 ***********************************************************************/

static inline bool is_hamming_ecc(struct bln_controller *ctrl,
				  struct bln_cfg *cfg)
{
	return cfg->spare_area_size == 16 && cfg->ecc_level == 15;
}

static struct nand_ecclayout bln_oob_16 = {
	.eccbytes = 3,
	.eccpos = {
		6, 7, 8
	},
	.oobfree = {
		{ .offset = 0, .length = 5 },
		{ .offset = 9, .length = 7 },
		{ .offset = 0, .length = 0 },
	}
};

static struct nand_ecclayout bln_oob_64 = {
	.eccbytes = 12,
	.eccpos = {
		6, 7, 8,
		22, 23, 24,
		38, 39, 40,
		54, 55, 56
	},
	.oobfree = {
		{ .offset = 2, .length = 4 },
		{ .offset = 9, .length = 13 },
		{ .offset = 25, .length = 13 },
		{ .offset = 41, .length = 13 },
		{ .offset = 57, .length = 7 },
		{ .offset = 0, .length = 0 },
	}
};

static struct nand_ecclayout *bln_ecc_layout(struct bln_host *host)
{
	struct mtd_info *mtd = nand_to_mtd(&host->chip);
	struct nand_ecclayout *layout = NULL;

	switch (mtd->oobsize) {
		case 16:
			layout = &bln_oob_16;
			break;
		case 64:
			layout = &bln_oob_64;
			break;
		default:
			dev_err(&host->pdev->dev, "OOB size invalid\n");
			break;
	}

	return layout;
}

/* Helper functions for reading and writing OOB registers */
static inline u8 oob_reg_read(struct bln_controller *ctrl, u32 offs)
{
	u16 reg_offs;

	if (offs >= ctrl->max_oob)
		return 0x77;

	reg_offs = BLN_OOB_READ_BASE + (offs & ~0x03);

	return bln_read(ctrl, reg_offs) >> (24 - ((offs & 0x03) << 3));
}

static inline void oob_reg_write(struct bln_controller *ctrl, u32 offs,
				 u32 data)
{
	u16 reg_offs;

	if (offs >= ctrl->max_oob)
		return;

	reg_offs = BLN_OOB_WRITE_BASE + (offs & ~0x03);

	bln_write(ctrl, reg_offs, data);
}

/*
 * read_oob_from_regs - read data from OOB registers
 * @ctrl: NAND controller
 * @i: sub-page sector index
 * @oob: buffer to read to
 * @sas: spare area sector size (i.e., OOB size per FLASH_CACHE)
 */
static int read_oob_from_regs(struct bln_controller *ctrl, u8 *oob,
			      int sas)
{
	int tbytes = sas;
	int i;

	tbytes = min_t(int, tbytes, ctrl->max_oob);
	for (i = 0; i < tbytes; i++)
		oob[i] = oob_reg_read(ctrl, i);

	return tbytes;
}

/*
 * write_oob_to_regs - write data to OOB registers
 * @i: sub-page sector index
 * @oob: buffer to write from
 * @sas: spare area sector size (i.e., OOB size per FLASH_CACHE)
 */
static int write_oob_to_regs(struct bln_controller *ctrl,
			     const u8 *oob, int sas)
{
	int tbytes = sas;
	int i;

	tbytes = min_t(int, tbytes, ctrl->max_oob);
	for (i = 0; i < tbytes; i += 4)
		oob_reg_write(ctrl, i,
			      (oob[i + 0] << 24) |
			      (oob[i + 1] << 16) |
			      (oob[i + 2] <<  8) |
			      (oob[i + 3] <<  0));

	return tbytes;
}

static void bln_send_cmd(struct bln_host *host, int cmd)
{
	struct bln_controller *ctrl = host->ctrl;
	int ret;

	dev_dbg(ctrl->dev, "send native cmd %d addr_lo 0x%x\n", cmd,
		bln_read(ctrl, BLN_CMD_ADDR));
	BUG_ON(ctrl->cmd_pending != 0);
	ctrl->cmd_pending = cmd;

	ret = bln_ctrl_poll_status(ctrl, NAND_CTL_READY, NAND_CTL_READY, 0);
	WARN_ON(ret);

	mb(); /* flush previous writes */
	bln_write(ctrl, BLN_CMD_START, cmd << CMD_SHIFT);
}

/***********************************************************************
 * NAND MTD API: read/program/erase
 ***********************************************************************/

static void bln_cmd_ctrl(struct mtd_info *mtd, int dat,
			      unsigned int ctrl)
{
	/* intentionally left blank */
}

static int bln_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct bln_host *host = nand_get_controller_data(chip);
	struct bln_controller *ctrl = host->ctrl;
	int ret;

	dev_dbg(ctrl->dev, "wait on native cmd %d\n", ctrl->cmd_pending);

	ret = bln_ctrl_poll_status(ctrl, NAND_CTL_READY, NAND_CTL_READY,
				   NAND_POLL_TOUT_MS);
	WARN_ON(ret);

	ctrl->cmd_pending = 0;
	return bln_read(ctrl, BLN_INTFC_STATUS) &
			INTFC_FLASH_STATUS;
}

static void bln_cmdfunc(struct mtd_info *mtd, unsigned command,
			int column, int page_addr)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct bln_host *host = nand_get_controller_data(chip);
	struct bln_controller *ctrl = host->ctrl;
	u64 addr = (u64)page_addr << chip->page_shift;
	int native_cmd = 0;

	if (command == NAND_CMD_READID)
		addr = (u64)column;
	/* Avoid propagating a negative, don't-care address */
	else if (page_addr < 0)
		addr = 0;

	dev_dbg(ctrl->dev, "cmd 0x%x addr 0x%llx\n", command,
		(unsigned long long)addr);

	host->last_cmd = command;
	host->last_byte = 0;
	host->last_addr = addr;

	switch (command) {
	case NAND_CMD_RESET:
		native_cmd = CMD_FLASH_RESET;
		break;
	case NAND_CMD_STATUS:
		native_cmd = CMD_STATUS_READ;
		break;
	case NAND_CMD_READID:
		native_cmd = CMD_DEV_ID_READ;
		break;
	case NAND_CMD_READOOB:
		native_cmd = CMD_SPARE_READ;
		break;
	case NAND_CMD_ERASE1:
		native_cmd = CMD_BLK_ERASE;
		break;
	}

	if (!native_cmd) {
		dev_err(ctrl->dev, "cmd 0x%x not supported\n", command);
		return;
	}

	bln_write(ctrl, BLN_CMD_EXT_ADDR,
		  (host->cs << 16) | ((addr >> 32) & 0xffff));
	(void)bln_read(ctrl, BLN_CMD_EXT_ADDR);
	bln_write(ctrl, BLN_CMD_ADDR, lower_32_bits(addr));
	(void)bln_read(ctrl, BLN_CMD_ADDR);

	bln_send_cmd(host, native_cmd);
	bln_waitfunc(mtd, chip);
}

static uint8_t bln_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct bln_host *host = nand_get_controller_data(chip);
	struct bln_controller *ctrl = host->ctrl;
	uint8_t ret = 0;

	switch (host->last_cmd) {
	case NAND_CMD_READID:
		if (host->last_byte < 4)
			ret = bln_read(ctrl, BLN_DEV_ID) >>
				(24 - (host->last_byte << 3));
		break;

	case NAND_CMD_READOOB:
		ret = oob_reg_read(ctrl, host->last_byte);
		break;

	case NAND_CMD_STATUS:
		ret = bln_read(ctrl, BLN_INTFC_STATUS) &
					INTFC_FLASH_STATUS;
		break;
	}

	dev_dbg(ctrl->dev, "read byte = 0x%02x\n", ret);
	host->last_byte++;

	return ret;
}

static void bln_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;

	printk("%s\n", __func__);

	for (i = 0; i < len; i++, buf++)
		*buf = bln_read_byte(mtd);
}

static void bln_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	BUG();
}

static int brcmstb_nand_verify_erased_page(struct mtd_info *mtd,
		  struct nand_chip *chip, void *buf, u64 addr)
{
	int i, sas;
	void *oob = chip->oob_poi;
	int bitflips = 0;
	int page = addr >> chip->page_shift;
	int ret;

	if (!buf) {
		buf = chip->buffers->databuf;
		/* Invalidate page cache */
		chip->pagebuf = -1;
	}

	sas = mtd->oobsize / chip->ecc.steps;

	/* read without ecc for verification */
	ret = chip->ecc.read_page_raw(mtd, chip, buf, true, page);
	if (ret)
		return ret;

	for (i = 0; i < chip->ecc.steps; i++, oob += sas) {
		ret = nand_check_erased_ecc_chunk(buf, chip->ecc.size,
						  oob, sas, NULL, 0,
						  chip->ecc.strength);
		if (ret < 0)
			return ret;

		bitflips = max(bitflips, ret);
	}

	return bitflips;
}

static int bln_read_by_pio(struct mtd_info *mtd, struct nand_chip *chip,
			   u64 addr, unsigned int trans, u32 *buf,
			   u8 *oob, u64 *err_addr)
{
	struct bln_host *host = nand_get_controller_data(chip);
	struct bln_controller *ctrl = host->ctrl;
	int i, j, ret = 0;

	/* Clear error addresses */
	bln_write(ctrl, BLN_UNCORR_ADDR, 0);
	bln_write(ctrl, BLN_CORR_ADDR, 0);
	bln_write(ctrl, BLN_UNCORR_EXT_ADDR, 0);
	bln_write(ctrl, BLN_CORR_EXT_ADDR, 0);

	bln_write(ctrl, BLN_CMD_EXT_ADDR,
			(host->cs << 16) | ((addr >> 32) & 0xffff));
	(void)bln_read(ctrl, BLN_CMD_EXT_ADDR);

	for (i = 0; i < trans; i++, addr += FC_BYTES) {
		bln_write(ctrl, BLN_CMD_ADDR, lower_32_bits(addr));
		(void)bln_read(ctrl, BLN_CMD_ADDR);
		bln_send_cmd(host, CMD_PAGE_READ);
		bln_waitfunc(mtd, chip);

		if (likely(buf))
			for (j = 0; j < FC_WORDS; j++, buf++)
				*buf = bln_read_fc(ctrl, j);

		if (oob)
			oob += read_oob_from_regs(ctrl, oob,
					mtd->oobsize / trans);

		if (!ret) {
			*err_addr = bln_read(ctrl,
					BLN_UNCORR_ADDR) |
				((u64)(bln_read(ctrl,
						BLN_UNCORR_EXT_ADDR)
					& 0xffff) << 32);
			if (*err_addr)
				ret = -EBADMSG;
		}

		if (!ret) {
			*err_addr = bln_read(ctrl,
					BLN_CORR_ADDR) |
				((u64)(bln_read(ctrl,
						BLN_CORR_EXT_ADDR)
					& 0xffff) << 32);
			if (*err_addr)
				ret = -EUCLEAN;
		}
	}

	return ret;
}

static int bln_read_spare(struct mtd_info *mtd, struct nand_chip *chip,
			  u64 addr, unsigned int trans, u8 *oob)
{
	struct bln_host *host = nand_get_controller_data(chip);
	struct bln_controller *ctrl = host->ctrl;
	int i;

	bln_write(ctrl, BLN_CMD_EXT_ADDR,
		  (host->cs << 16) | ((addr >> 32) & 0xffff));
	(void)bln_read(ctrl, BLN_CMD_EXT_ADDR);

	for (i = 0; i < trans; i++, addr += FC_BYTES) {
		bln_write(ctrl, BLN_CMD_ADDR, lower_32_bits(addr));
		(void)bln_read(ctrl, BLN_CMD_ADDR);
		bln_send_cmd(host, CMD_SPARE_READ);
		bln_waitfunc(mtd, chip);

		oob += read_oob_from_regs(ctrl, oob, mtd->oobsize / trans);
	}

	return 0;
}

static int bln_read_cache(struct mtd_info *mtd, struct nand_chip *chip,
			  u64 addr, unsigned int trans, u32 *buf, u8 *oob)
{
	struct bln_host *host = nand_get_controller_data(chip);
	struct bln_controller *ctrl = host->ctrl;
	u64 err_addr = 0;
	int err;

	dev_err(ctrl->dev, "read %llx -> %p\n", (unsigned long long)addr, buf);

	if (oob)
		memset(oob, 0x99, mtd->oobsize);

	err = bln_read_by_pio(mtd, chip, addr, trans, buf, oob, &err_addr);
	if (mtd_is_eccerr(err)) {
		err = brcmstb_nand_verify_erased_page(mtd, chip, buf, addr);
		/* erased page bitflips corrected */
		if (err >= 0)
			return err;

		dev_err(ctrl->dev, "uncorrectable error at 0x%llx\n",
			(unsigned long long)err_addr);
		mtd->ecc_stats.failed++;

		/* NAND layer expects zero on ECC errors */
		return 0;
	}

	if (mtd_is_bitflip(err)) {
		dev_err(ctrl->dev, "corrected error at 0x%llx\n",
			(unsigned long long)err_addr);
		mtd->ecc_stats.corrected++;

		/* Always exceed the software-imposed threshold */
		return max(mtd->bitflip_threshold, mtd->ecc_stats.corrected);
	}

	return 0;
}

static int bln_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			 uint8_t *buf, int oob_required, int page)
{
	printk("%s\n", __func__);

	if (buf)
		memset(buf, 0, mtd->writesize);

	return 0;
}

static int bln_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			     uint8_t *buf, int oob_required, int page)
{
	struct bln_host *host = nand_get_controller_data(chip);
	int ret;

	printk("%s %llx\n", __func__, host->last_addr);

	nand_read_page_op(chip, page, 0, NULL, 0);

	bln_set_ecc_enabled(host, 0);
	ret = bln_read_cache(mtd, chip, host->last_addr,
			     mtd->writesize >> FC_SHIFT, (u32 *)buf, NULL);
	bln_set_ecc_enabled(host, 1);

	printk("%s %llx\n", __func__, host->last_addr);

	if (oob_required)
		bln_read_spare(mtd, chip, host->last_addr, mtd->writesize >> FC_SHIFT, chip->oob_poi);

	return ret;
}

static int bln_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
			int page)
{
	printk("%s\n", __func__);

	return 0;
}

static int bln_read_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
			    int page)
{
	printk("%s\n", __func__);

	return 0;
}

static int bln_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			  const uint8_t *buf, int oob_required, int page)
{
	printk("%s\n", __func__);

	return 0;
}

static int bln_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      const uint8_t *buf, int oob_required, int page)
{
	printk("%s\n", __func__);

	return 0;
}

static int bln_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			 int page)
{
	printk("%s\n", __func__);

	return 0;
}

static int bln_write_oob_raw(struct mtd_info *mtd, struct nand_chip *chip,
			     int page)
{
	printk("%s\n", __func__);

	return 0;
}

/***********************************************************************
 * Per-CS setup (1 NAND device)
 ***********************************************************************/

static int bln_set_cfg(struct bln_host *host, struct bln_cfg *cfg)
{
	struct bln_controller *ctrl = host->ctrl;
	u8 block_size = 0, page_size = 0, device_size = 0;
	u32 tmp;

	if (ctrl->block_sizes) {
		int i, found;

		for (i = 0, found = 0; ctrl->block_sizes[i]; i++)
			if (ctrl->block_sizes[i] * 1024 == cfg->block_size) {
				block_size = i;
				found = 1;
			}
		if (!found) {
			dev_warn(ctrl->dev, "invalid block size %u\n",
					cfg->block_size);
			return -EINVAL;
		}
	} else {
		block_size = ffs(cfg->block_size) - ffs(BLN_MIN_BLOCKSIZE);
	}

	if (cfg->block_size < BLN_MIN_BLOCKSIZE || (ctrl->max_block_size &&
				cfg->block_size > ctrl->max_block_size)) {
		dev_warn(ctrl->dev, "invalid block size %u\n",
				cfg->block_size);
		block_size = 0;
	}

	if (ctrl->page_sizes) {
		int i, found;

		for (i = 0, found = 0; ctrl->page_sizes[i]; i++)
			if (ctrl->page_sizes[i] == cfg->page_size) {
				page_size = i;
				found = 1;
			}
		if (!found) {
			dev_warn(ctrl->dev, "invalid page size %u\n",
					cfg->page_size);
			return -EINVAL;
		}
	} else {
		page_size = ffs(cfg->page_size) - ffs(BLN_MIN_PAGESIZE);
	}

	if (cfg->page_size < BLN_MIN_PAGESIZE || (ctrl->max_page_size &&
				cfg->page_size > ctrl->max_page_size)) {
		dev_warn(ctrl->dev, "invalid page size %u\n", cfg->page_size);
		return -EINVAL;
	}

	if (fls64(cfg->device_size) < fls64(BLN_MIN_DEVSIZE)) {
		dev_warn(ctrl->dev, "invalid device size 0x%llx\n",
			(unsigned long long)cfg->device_size);
		return -EINVAL;
	}
	device_size = fls64(cfg->device_size) - fls64(BLN_MIN_DEVSIZE);

	tmp = (cfg->blk_adr_bytes << CFG_BLK_ADDR_SHIFT) |
	      (cfg->col_adr_bytes << CFG_COL_ADDR_SHIFT) |
	      (cfg->ful_adr_bytes << CFG_FUL_ADDR_SHIFT) |
	      (device_size << CFG_DEV_SIZE_SHIFT) |
	      (page_size << CFG_PAGE_SIZE_SHIFT) |
	      (block_size << CFG_BLK_SIZE_SHIFT);
	if (cfg->device_width == 16)
		tmp |= CFG_DEV_WIDTH_16;
	bln_write(ctrl, BLN_CFG, tmp);

	return 0;
}

static void bln_print_cfg(struct bln_host *host,
			       char *buf, struct bln_cfg *cfg)
{
	buf += sprintf(buf,
		"%lluMiB total, %uKiB blocks, %u%s pages, %uB OOB, %u-bit",
		(unsigned long long)cfg->device_size >> 20,
		cfg->block_size >> 10,
		cfg->page_size >= 1024 ? cfg->page_size >> 10 : cfg->page_size,
		cfg->page_size >= 1024 ? "KiB" : "B",
		cfg->spare_area_size, cfg->device_width);

	/* Account for Hamming ECC and for BCH 512B vs 1KiB sectors */
	if (is_hamming_ecc(host->ctrl, cfg))
		sprintf(buf, ", Hamming ECC");
	else
		sprintf(buf, ", BCH-%u", cfg->ecc_level);
}

/*
 * Minimum number of bytes to address a page. Calculated as:
 *     roundup(log2(size / page-size) / 8)
 *
 * NB: the following does not "round up" for non-power-of-2 'size'; but this is
 *     OK because many other things will break if 'size' is irregular...
 */
static inline int get_blk_adr_bytes(u64 size, u32 writesize)
{
	return ALIGN(ilog2(size) - ilog2(writesize), 8) >> 3;
}

static int bln_setup_dev(struct bln_host *host)
{
	struct mtd_info *mtd = nand_to_mtd(&host->chip);
	struct nand_chip *chip = &host->chip;
	struct bln_controller *ctrl = host->ctrl;
	struct bln_cfg *cfg = &host->hwcfg;
	char msg[128];
	u32 tmp, oob_sector;
	int ret;

	memset(cfg, 0, sizeof(*cfg));

	ret = ofnode_read_u32(nand_get_flash_node(chip),
			      "brcm,nand-oob-sector-size",
			      &oob_sector);
	if (ret) {
		/* Use detected size */
		cfg->spare_area_size = mtd->oobsize /
					(mtd->writesize >> FC_SHIFT);
	} else {
		cfg->spare_area_size = oob_sector;
	}
	if (cfg->spare_area_size > ctrl->max_oob)
		cfg->spare_area_size = ctrl->max_oob;
	/*
	 * Set oobsize to be consistent with controller's spare_area_size, as
	 * the rest is inaccessible.
	 */
	mtd->oobsize = cfg->spare_area_size * (mtd->writesize >> FC_SHIFT);

	cfg->device_size = mtd->size;
	cfg->block_size = mtd->erasesize;
	cfg->page_size = mtd->writesize;
	cfg->device_width = (chip->options & NAND_BUSWIDTH_16) ? 16 : 8;
	cfg->col_adr_bytes = 2;
	cfg->blk_adr_bytes = get_blk_adr_bytes(mtd->size, mtd->writesize);

	if (chip->ecc.mode != NAND_ECC_HW) {
		dev_err(ctrl->dev, "only HW ECC supported; selected: %d\n",
			chip->ecc.mode);
		return -EINVAL;
	}

	if (chip->ecc.algo == NAND_ECC_UNKNOWN) {
		if (chip->ecc.strength == 1 && chip->ecc.size == 512)
			/* Default to Hamming for 1-bit ECC, if unspecified */
			chip->ecc.algo = NAND_ECC_HAMMING;
		else
			/* Otherwise, BCH */
			chip->ecc.algo = NAND_ECC_BCH;
	}

	if (chip->ecc.algo == NAND_ECC_HAMMING && (chip->ecc.strength != 1 ||
						   chip->ecc.size != 512)) {
		dev_err(ctrl->dev, "invalid Hamming params: %d bits per %d bytes\n",
			chip->ecc.strength, chip->ecc.size);
		return -EINVAL;
	}

	switch (chip->ecc.size) {
	case 512:
		if (chip->ecc.algo == NAND_ECC_HAMMING)
			cfg->ecc_level = 15;
		else
			cfg->ecc_level = chip->ecc.strength;
		break;
	default:
		dev_err(ctrl->dev, "unsupported ECC size: %d\n",
			chip->ecc.size);
		return -EINVAL;
	}

	cfg->ful_adr_bytes = cfg->blk_adr_bytes;
	if (mtd->writesize > 512)
		cfg->ful_adr_bytes += cfg->col_adr_bytes;
	else
		cfg->ful_adr_bytes += 1;

	ret = bln_set_cfg(host, cfg);
	if (ret)
		return ret;

#if 0
	bln_set_ecc_enabled(host, 1);
#endif

	bln_print_cfg(host, msg, cfg);
#if 0
	dev_info(ctrl->dev, "detected %s\n", msg);
#else
	dev_err(ctrl->dev, "detected %s\n", msg);
#endif

	/* Configure ACC_CONTROL */
	tmp = bln_read(ctrl, BLN_ACC_CTL);
	tmp &= ~ACC_CTL_RD_ERASED_ECC;
	tmp &= ~ACC_CTL_FAST_PGM_RDIN;
	bln_write(ctrl, BLN_ACC_CTL, tmp);

	return 0;
}

static int bln_init_cs(struct bln_host *host, ofnode dn)
{
	struct bln_controller *ctrl = host->ctrl;
	struct udevice *pdev = host->pdev;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	int ret;

	ret = ofnode_read_s32(dn, "reg", &host->cs);
	if (ret) {
		dev_err(&pdev->dev, "can't get chip-select\n");
		return -ENXIO;
	}

	mtd = nand_to_mtd(&host->chip);
	chip = &host->chip;

	nand_set_flash_node(chip, dn);
	nand_set_controller_data(chip, host);

	mtd->name = devm_kasprintf(pdev, GFP_KERNEL, "bcm-leg-nand.%d",
				   host->cs);
	if (!mtd->name)
		return -ENOMEM;

	mtd->owner = THIS_MODULE;
	mtd->dev->parent = pdev;

	chip->IO_ADDR_R = (void __iomem *)0xdeadbeef;
	chip->IO_ADDR_W = (void __iomem *)0xdeadbeef;

	chip->cmd_ctrl = bln_cmd_ctrl;
	chip->cmdfunc = bln_cmdfunc;
	chip->waitfunc = bln_waitfunc;
	chip->read_byte = bln_read_byte;
	chip->read_buf = bln_read_buf;
	chip->write_buf = bln_write_buf;

	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.read_page = bln_read_page;
	chip->ecc.write_page = bln_write_page;
	chip->ecc.read_page_raw = bln_read_page_raw;
	chip->ecc.write_page_raw = bln_write_page_raw;
	chip->ecc.write_oob_raw = bln_write_oob_raw;
	chip->ecc.read_oob_raw = bln_read_oob_raw;
	chip->ecc.read_oob = bln_read_oob;
	chip->ecc.write_oob = bln_write_oob;

	chip->controller = &ctrl->controller;

	/*
	 * The bootloader might have configured 16bit mode but
	 * NAND READID command only works in 8bit mode. We force
	 * 8bit mode here to ensure that NAND READID commands works.
	 */
	bln_rmw(ctrl, BLN_CFG, CFG_DEV_WIDTH_16, 0, 0);

	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret)
		return ret;

	chip->options |= NAND_NO_SUBPAGE_WRITE;
	/*
	 * Avoid (for instance) kmap()'d buffers from JFFS2, which we can't DMA
	 * to/from, and have nand_base pass us a bounce buffer instead, as
	 * needed.
	 */
	chip->options |= NAND_USE_BOUNCE_BUFFER;

	if (chip->bbt_options & NAND_BBT_USE_FLASH)
		chip->bbt_options |= NAND_BBT_NO_OOB;

	if (bln_setup_dev(host))
		return -ENXIO;

	chip->ecc.size = 512;
	/* only use our internal HW threshold */
	mtd->bitflip_threshold = 1;

	chip->ecc.layout = bln_ecc_layout(host);
	if (!chip->ecc.layout)
		return -ENXIO;

	ret = nand_scan_tail(mtd);
	if (ret)
		return ret;

	ret = nand_register(0, mtd);

	return ret;
}

/***********************************************************************
 * Platform driver setup (per controller)
 ***********************************************************************/

int bln_probe(struct udevice *dev, struct bln_soc *soc)
{
	ofnode child;
	struct udevice *pdev = dev;
	struct bln_controller *ctrl;
	struct resource res;
	int ret;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	soc->ctrl = ctrl;
	ctrl->dev = dev;

	nand_hw_control_init(&ctrl->controller);
	INIT_LIST_HEAD(&ctrl->host_list);

	/* NAND register range */
	dev_read_resource(pdev, 0, &res);
	ctrl->nand_base = devm_ioremap(pdev, res.start, resource_size(&res));
	if (IS_ERR(ctrl->nand_base))
		return PTR_ERR(ctrl->nand_base);

	/* Enable clock before using NAND registers */
	ctrl->clk = devm_clk_get(dev, "nand");
	if (!IS_ERR(ctrl->clk)) {
		ret = clk_prepare_enable(ctrl->clk);
		if (ret)
			return ret;
	} else {
		ret = PTR_ERR(ctrl->clk);
		if (ret == -EPROBE_DEFER)
			return ret;

		ctrl->clk = NULL;
	}

	/* Initialize NAND revision */
	ret = bln_revision_init(ctrl);
	if (ret)
		goto err;

	/*
	 * Most chips have this cache at a fixed offset within 'nand' block.
	 * Some must specify this region separately.
	 */
	dev_read_resource_byname(pdev, "nand-cache", &res);
	ctrl->nand_fc = devm_ioremap(dev, res.start, resource_size(&res));
	if (IS_ERR(ctrl->nand_fc)) {
		ret = PTR_ERR(ctrl->nand_fc);
		goto err;
	}

	/* Disable automatic device ID config, direct addressing */
	bln_rmw(ctrl, BLN_CS_SEL, CS_SEL_AUTO_DEV_ID | CS_SEL_MASK, 0, 0);

	/* Disable XOR addressing */
	bln_rmw(ctrl, BLN_CS_XOR, CS_XOR_MASK, 0, 0);

	ofnode_for_each_subnode(child, dev_ofnode(dev)) {
		if (ofnode_device_is_compatible(child, "brcm,nandcs")) {
			struct bln_host *host;

			host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
			if (!host) {
				ret = -ENOMEM;
				goto err;
			}

			host->pdev = pdev;
			host->ctrl = ctrl;

			ret = bln_init_cs(host, child);
			if (ret) {
				devm_kfree(dev, host);
				continue; /* Try all chip-selects */
			}

			list_add_tail(&host->node, &ctrl->host_list);
		}
	}

	/* No chip-selects could initialize properly */
	if (list_empty(&ctrl->host_list)) {
		ret = -ENODEV;
		goto err;
	}

	return 0;

err:
	if (ctrl->clk)
		clk_disable(ctrl->clk);

	return ret;
}
