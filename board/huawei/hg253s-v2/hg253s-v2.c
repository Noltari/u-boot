// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <common.h>
#include <asm/io.h>

#define GPIO_BASE_6362			0x10000080

#define GPIO_BASE_MODE_6362_REG		0x38
#define GPIO_BASE_MODE_NAND_OVERRIDE	BIT(2)

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
	void __iomem *gpio_regs = map_physmem(GPIO_BASE_6362, 0, MAP_NOCACHE);

	/* Enable NAND */
	setbits_be32(gpio_regs + GPIO_BASE_MODE_6362_REG,
		     GPIO_BASE_MODE_NAND_OVERRIDE);

	return 0;
}
#endif
