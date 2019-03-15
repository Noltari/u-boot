/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __BRCMNAND_H__
#define __BRCMNAND_H__

#include <linux/types.h>
#include <linux/io.h>

struct bln_soc {
	bool (*ctlrdy_ack)(struct bln_soc *soc);
	void (*ctlrdy_set_enabled)(struct bln_soc *soc, bool en);
	void *ctrl;
};

static inline u32 brcmnand_readl(void __iomem *addr)
{
	/*
	 * MIPS endianness is configured by boot strap, which also reverses all
	 * bus endianness (i.e., big-endian CPU + big endian bus ==> native
	 * endian I/O).
	 *
	 * Other architectures (e.g., ARM) either do not support big endian, or
	 * else leave I/O in little endian mode.
	 */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(CONFIG_SYS_BIG_ENDIAN))
		return __raw_readl(addr);
	else
		return readl_relaxed(addr);
}

static inline void brcmnand_writel(u32 val, void __iomem *addr)
{
	/* See brcmnand_readl() comments */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(CONFIG_SYS_BIG_ENDIAN))
		__raw_writel(val, addr);
	else
		writel_relaxed(val, addr);
}

int bln_probe(struct udevice *dev, struct bln_soc *soc);
int bln_remove(struct udevice *dev);

#ifndef __UBOOT__
extern const struct dev_pm_ops brcmnand_pm_ops;
#endif /* __UBOOT__ */

#endif /* __BRCMNAND_H__ */
