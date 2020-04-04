// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <common.h>
#include <asm/io.h>

#define GPIO_BASE_63268			0x100000c0

#define GPIO_MODE_63268_REG		0x18
#define GPIO_MODE_63268_SERIAL_LED_CLK	BIT(0)
#define GPIO_MODE_63268_SERIAL_LED_DATA	BIT(1)

#define ROBOSW_LED_63268_REG		0x30
#define ROBOSW_LED_63268_BICOLOR_SPD	BIT(30)

#define ROBOSW_EPHY_63268_REG		0x3c
#define ROBOSW_EPHY_63268_RST_1		BIT(0)
#define ROBOSW_EPHY_63268_RST_2		BIT(1)
#define ROBOSW_EPHY_63268_RST_3		BIT(2)

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
	void __iomem *gpio_regs = map_physmem(GPIO_BASE_63268, 0, MAP_NOCACHE);

	/* Enable Serial LEDs */
	setbits_be32(gpio_regs + GPIO_MODE_63268_REG,
		     GPIO_MODE_63268_SERIAL_LED_CLK |
		     GPIO_MODE_63268_SERIAL_LED_DATA);

	/* Init ROBOSW LED Controller */
	setbits_be32(gpio_regs + ROBOSW_LED_63268_REG,
		     ROBOSW_LED_63268_BICOLOR_SPD);

	/* Init ROBOSW EPHY Controller */
	setbits_be32(gpio_regs + ROBOSW_EPHY_63268_REG,
		     ROBOSW_EPHY_63268_RST_1 |
		     ROBOSW_EPHY_63268_RST_2 |
		     ROBOSW_EPHY_63268_RST_3);

	return 0;
}
#endif
