// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/G2L Pin Control and GPIO driver core
 *
 * Copyright (C) 2021 Renesas Electronics Corporation.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/spinlock.h>

#include <dt-bindings/pinctrl/rzg2l-pinctrl.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinmux.h"

#define DRV_NAME	"pinctrl-rzg2l"

/*
 * Use 16 lower bits [15:0] for pin identifier
 * Use 16 higher bits [31:16] for pin mux function
 */
#define MUX_PIN_ID_MASK		GENMASK(15, 0)
#define MUX_FUNC_MASK		GENMASK(31, 16)
#define MUX_FUNC_OFFS		16
#define MUX_FUNC(pinconf)	(((pinconf) & MUX_FUNC_MASK) >> MUX_FUNC_OFFS)

/* PIN capabilities */
#define PIN_CFG_IOLH_A			BIT(0)
#define PIN_CFG_IOLH_B			BIT(1)
#define PIN_CFG_SR			BIT(2)
#define PIN_CFG_IEN			BIT(3)
#define PIN_CFG_PUPD			BIT(4)
#define PIN_CFG_IO_VMC_SD0		BIT(5)
#define PIN_CFG_IO_VMC_SD1		BIT(6)
#define PIN_CFG_IO_VMC_QSPI		BIT(7)
#define PIN_CFG_IO_VMC_ETH0		BIT(8)
#define PIN_CFG_IO_VMC_ETH1		BIT(9)
#define PIN_CFG_IO_VMC_ETH		(PIN_CFG_IO_VMC_ETH0 | PIN_CFG_IO_VMC_ETH1)
#define PIN_CFG_FILONOFF		BIT(10)
#define PIN_CFG_FILNUM			BIT(11)
#define PIN_CFG_FILCLKSEL		BIT(12)

#define RZG2L_MPXED_PIN_FUNCS		(PIN_CFG_IOLH_A | \
					 PIN_CFG_SR | \
					 PIN_CFG_PUPD | \
					 PIN_CFG_FILONOFF | \
					 PIN_CFG_FILNUM | \
					 PIN_CFG_FILCLKSEL)

#define RZG2L_MPXED_ETH_PIN_FUNCS(x)	((x) | \
					 PIN_CFG_FILONOFF | \
					 PIN_CFG_FILNUM | \
					 PIN_CFG_FILCLKSEL)

/*
 * n indicates number of pins in the port, a is the register index
 * and f is pin configuration capabilities supported.
 */
#define RZG2L_GPIO_PORT_PACK(n, a, f)	(((n) << 27) | ((a) << 20) | (f))
#define RZG2L_GPIO_PORT_GET_PINCNT(x)	(((x) & GENMASK(30, 27)) >> 27)
#define RZG2L_GPIO_PORT_GET_INDEX(x)	(((x) & GENMASK(26, 20)) >> 20)
#define RZG2L_GPIO_PORT_GET_CFGS(x)	((x) & GENMASK(19, 0))

/*
 * BIT(31) indicates dedicated pin, p is the register index while
 * referencing to SR/IEN/IOLH/FILxx registers, b is the register bits
 * (b * 8) and f is the pin configuration capabilities supported.
 */
#define RZG2L_SINGLE_PIN		BIT(31)
#define RZG2L_SINGLE_PIN_PACK(p, b, f)	(RZG2L_SINGLE_PIN | \
					 ((p) << 24) | ((b) << 20) | (f))
#define RZG2L_SINGLE_PIN_GET_PORT_OFFSET(x)	(((x) & GENMASK(30, 24)) >> 24)
#define RZG2L_SINGLE_PIN_GET_BIT(x)	(((x) & GENMASK(22, 20)) >> 20)
#define RZG2L_SINGLE_PIN_GET_CFGS(x)	((x) & GENMASK(19, 0))

#define P(n)			(0x0000 + 0x10 + (n))
#define PM(n)			(0x0100 + 0x20 + (n) * 2)
#define PMC(n)			(0x0200 + 0x10 + (n))
#define PFC(n)			(0x0400 + 0x40 + (n) * 4)
#define PIN(n)			(0x0800 + 0x10 + (n))
#define IOLH(n)			(0x1000 + (n) * 8)
#define SR(n)			(0x1400 + (n) * 8)
#define IEN(n)			(0x1800 + (n) * 8)
#define ISEL(n)			(0x2C00 + 0x80 + (n) * 8)
#define NOD(n)			(0x3000 + 0x80 + (n) * 8)
#define PWPR			(0x3014)
#define PWPR_RZ_V2H		(0x3C04)
#define SD_CH(n)		(0x3000 + (n) * 4)
#define QSPI			(0x3008)
#define ETH_CH(n)		(0x300C + (n) * 4)

#define PVDD_1800		1	/* I/O domain voltage <= 1.8V */
#define PVDD_3300		0	/* I/O domain voltage >= 3.3V */
#define ETH_PVDD_2500		BIT(1)	/* Ether I/O voltage 2.5V */
#define ETH_PVDD_1800		BIT(0)	/* Ether I/O voltage 1.8V */
#define ETH_PVDD_3300		0	/* Ether I/O voltage 3.3V */

#define PWPR_B0WI		BIT(7)	/* Bit Write Disable */
#define PWPR_PFCWE		BIT(6)	/* PFC Register Write Enable */
#define PWPR_REGWE_A		BIT(6) /* Controls writing to PFC and PMC */
#define PWPR_REGWE_B		BIT(5) /* Controls writing to OEN */

#define PM_MASK			0x03
#define PVDD_MASK		0x01
#define ETH_PVDD_MASK		0x03
#define PFC_MASK		0x0F
#define IEN_MASK		0x01
#define SR_MASK			0x01
#define IOLH_MASK		0x03
#define NOD_MASK		0x01

#define PM_INPUT		0x1
#define PM_OUTPUT		0x2

#define RZG2L_PIN_ID_TO_PORT(id)	((id) / RZG2L_PINS_PER_PORT)
#define RZG2L_PIN_ID_TO_PORT_OFFSET(id)	(RZG2L_PIN_ID_TO_PORT(id) + 0x10)
#define RZG2L_PIN_ID_TO_PIN(id)		((id) % RZG2L_PINS_PER_PORT)

/* Hardware Registers support GPIO interrupt in IA55 Module */
#define TSCR		0x0 /* TINT Interrupt Status Control Register */
#define TITSR0(n)	(0x4 + (n) * 4) /* TINT detection method selection register 0 */
#define TITSR1(n)	(0x8 + (n) * 4) /* TINT detection method selection register 1 */
#define TSSR(n)		(0x10 + (n) * 4) /* TINT source selection register */
/* Only for RZ/V2H */
#define TSCLR		0x4 /* TINT Interrupt Clear Status Register */

#define TMSK		0x0 /* TINT interrupt mask control register */

#define RISING_EDGE	0
#define FALLING_EDGE	1
#define HIGH_LEVEL	2
#define LOW_LEVEL	3
#define IRQ_MASK	0x3

#define TINT_MAX       32	/* Maximum 32 Interrupts can be supported */

#define RZG2L_PIN_INFO(p, b)	(((p) << 16) | (b))
static const int rzg2l_pin_info[] = {
	RZG2L_PIN_INFO(0,  0), RZG2L_PIN_INFO(0,  1),
	RZG2L_PIN_INFO(1,  0), RZG2L_PIN_INFO(1,  1),
	RZG2L_PIN_INFO(2,  0), RZG2L_PIN_INFO(2,  1),
	RZG2L_PIN_INFO(3,  0), RZG2L_PIN_INFO(3,  1),
	RZG2L_PIN_INFO(4,  0), RZG2L_PIN_INFO(4,  1),
	RZG2L_PIN_INFO(5,  0), RZG2L_PIN_INFO(5,  1), RZG2L_PIN_INFO(5,  2),
	RZG2L_PIN_INFO(6,  0), RZG2L_PIN_INFO(6,  1),
	RZG2L_PIN_INFO(7,  0), RZG2L_PIN_INFO(7,  1), RZG2L_PIN_INFO(7,  2),
	RZG2L_PIN_INFO(8,  0), RZG2L_PIN_INFO(8,  1), RZG2L_PIN_INFO(8,  2),
	RZG2L_PIN_INFO(9,  0), RZG2L_PIN_INFO(9,  1),
	RZG2L_PIN_INFO(10, 0), RZG2L_PIN_INFO(10, 1),
	RZG2L_PIN_INFO(11, 0), RZG2L_PIN_INFO(11, 1),
	RZG2L_PIN_INFO(12, 0), RZG2L_PIN_INFO(12, 1),
	RZG2L_PIN_INFO(13, 0), RZG2L_PIN_INFO(13, 1), RZG2L_PIN_INFO(13, 2),
	RZG2L_PIN_INFO(14, 0), RZG2L_PIN_INFO(14, 1),
	RZG2L_PIN_INFO(15, 0), RZG2L_PIN_INFO(15, 1),
	RZG2L_PIN_INFO(16, 0), RZG2L_PIN_INFO(16, 1),
	RZG2L_PIN_INFO(17, 0), RZG2L_PIN_INFO(17, 1), RZG2L_PIN_INFO(17, 2),
	RZG2L_PIN_INFO(18, 0), RZG2L_PIN_INFO(18, 1),
	RZG2L_PIN_INFO(19, 0), RZG2L_PIN_INFO(19, 1),
	RZG2L_PIN_INFO(20, 0), RZG2L_PIN_INFO(20, 1), RZG2L_PIN_INFO(20, 2),
	RZG2L_PIN_INFO(21, 0), RZG2L_PIN_INFO(21, 1),
	RZG2L_PIN_INFO(22, 0), RZG2L_PIN_INFO(22, 1),
	RZG2L_PIN_INFO(23, 0), RZG2L_PIN_INFO(23, 1),
	RZG2L_PIN_INFO(24, 0), RZG2L_PIN_INFO(24, 1),
	RZG2L_PIN_INFO(25, 0), RZG2L_PIN_INFO(25, 1),
	RZG2L_PIN_INFO(26, 0), RZG2L_PIN_INFO(26, 1),
	RZG2L_PIN_INFO(27, 0), RZG2L_PIN_INFO(27, 1),
	RZG2L_PIN_INFO(28, 0), RZG2L_PIN_INFO(28, 1),
	RZG2L_PIN_INFO(29, 0), RZG2L_PIN_INFO(29, 1),
	RZG2L_PIN_INFO(30, 0), RZG2L_PIN_INFO(30, 1),
	RZG2L_PIN_INFO(31, 0), RZG2L_PIN_INFO(31, 1),
	RZG2L_PIN_INFO(32, 0), RZG2L_PIN_INFO(32, 1),
	RZG2L_PIN_INFO(33, 0), RZG2L_PIN_INFO(33, 1),
	RZG2L_PIN_INFO(34, 0), RZG2L_PIN_INFO(34, 1),
	RZG2L_PIN_INFO(35, 0), RZG2L_PIN_INFO(35, 1),
	RZG2L_PIN_INFO(36, 0), RZG2L_PIN_INFO(36, 1),
	RZG2L_PIN_INFO(37, 0), RZG2L_PIN_INFO(37, 1), RZG2L_PIN_INFO(37, 2),
	RZG2L_PIN_INFO(38, 0), RZG2L_PIN_INFO(38, 1),
	RZG2L_PIN_INFO(39, 0), RZG2L_PIN_INFO(39, 1), RZG2L_PIN_INFO(39, 2),
	RZG2L_PIN_INFO(40, 0), RZG2L_PIN_INFO(40, 1), RZG2L_PIN_INFO(40, 2),
	RZG2L_PIN_INFO(41, 0), RZG2L_PIN_INFO(41, 1),
	RZG2L_PIN_INFO(42, 0), RZG2L_PIN_INFO(42, 1), RZG2L_PIN_INFO(42, 2),
	RZG2L_PIN_INFO(42, 3), RZG2L_PIN_INFO(42, 4),
	RZG2L_PIN_INFO(43, 0), RZG2L_PIN_INFO(43, 1), RZG2L_PIN_INFO(43, 2),
	RZG2L_PIN_INFO(43, 3),
	RZG2L_PIN_INFO(44, 0), RZG2L_PIN_INFO(44, 1), RZG2L_PIN_INFO(44, 2),
	RZG2L_PIN_INFO(44, 3),
	RZG2L_PIN_INFO(45, 0), RZG2L_PIN_INFO(45, 1), RZG2L_PIN_INFO(45, 2),
	RZG2L_PIN_INFO(45, 3),
	RZG2L_PIN_INFO(46, 0), RZG2L_PIN_INFO(46, 1), RZG2L_PIN_INFO(46, 2),
	RZG2L_PIN_INFO(46, 3),
	RZG2L_PIN_INFO(47, 0), RZG2L_PIN_INFO(47, 1), RZG2L_PIN_INFO(47, 2),
	RZG2L_PIN_INFO(47, 3),
	RZG2L_PIN_INFO(48, 0), RZG2L_PIN_INFO(48, 1), RZG2L_PIN_INFO(48, 2),
	RZG2L_PIN_INFO(48, 3), RZG2L_PIN_INFO(48, 4),
};

static const int rzg2ul_pin_info[] = {
	RZG2L_PIN_INFO(0,  0), RZG2L_PIN_INFO(0,  1), RZG2L_PIN_INFO(0,  2),
	RZG2L_PIN_INFO(0,  3),
	RZG2L_PIN_INFO(1,  0), RZG2L_PIN_INFO(1,  1), RZG2L_PIN_INFO(1,  2),
	RZG2L_PIN_INFO(1,  3), RZG2L_PIN_INFO(1,  4),
	RZG2L_PIN_INFO(2,  0), RZG2L_PIN_INFO(2,  1), RZG2L_PIN_INFO(2,  2),
	RZG2L_PIN_INFO(2,  3),
	RZG2L_PIN_INFO(3,  0), RZG2L_PIN_INFO(3,  1), RZG2L_PIN_INFO(3,  2),
	RZG2L_PIN_INFO(3,  3),
	RZG2L_PIN_INFO(4,  0), RZG2L_PIN_INFO(4,  1), RZG2L_PIN_INFO(4,  2),
	RZG2L_PIN_INFO(4,  3), RZG2L_PIN_INFO(4,  4), RZG2L_PIN_INFO(4,  5),
	RZG2L_PIN_INFO(5,  0), RZG2L_PIN_INFO(5,  1), RZG2L_PIN_INFO(5,  2),
	RZG2L_PIN_INFO(5,  3), RZG2L_PIN_INFO(5,  4),
	RZG2L_PIN_INFO(6,  0), RZG2L_PIN_INFO(6,  1), RZG2L_PIN_INFO(6,  2),
	RZG2L_PIN_INFO(6,  3), RZG2L_PIN_INFO(6,  4),
	RZG2L_PIN_INFO(7,  0), RZG2L_PIN_INFO(7,  1), RZG2L_PIN_INFO(7,  2),
	RZG2L_PIN_INFO(7,  3), RZG2L_PIN_INFO(7,  4),
	RZG2L_PIN_INFO(8,  0), RZG2L_PIN_INFO(8,  1), RZG2L_PIN_INFO(8,  2),
	RZG2L_PIN_INFO(8,  3), RZG2L_PIN_INFO(8,  4),
	RZG2L_PIN_INFO(9,  0), RZG2L_PIN_INFO(9,  1), RZG2L_PIN_INFO(9,  2),
	RZG2L_PIN_INFO(9,  3),
	RZG2L_PIN_INFO(10, 0), RZG2L_PIN_INFO(10, 1), RZG2L_PIN_INFO(10, 2),
	RZG2L_PIN_INFO(10, 3), RZG2L_PIN_INFO(10, 4),
	RZG2L_PIN_INFO(11, 0), RZG2L_PIN_INFO(11, 1), RZG2L_PIN_INFO(11, 2),
	RZG2L_PIN_INFO(11, 3),
	RZG2L_PIN_INFO(12, 0), RZG2L_PIN_INFO(12, 1),
	RZG2L_PIN_INFO(13, 0), RZG2L_PIN_INFO(13, 1), RZG2L_PIN_INFO(13, 2),
	RZG2L_PIN_INFO(13, 3), RZG2L_PIN_INFO(13, 4),
	RZG2L_PIN_INFO(14, 0), RZG2L_PIN_INFO(14, 1), RZG2L_PIN_INFO(14, 2),
	RZG2L_PIN_INFO(15, 0), RZG2L_PIN_INFO(15, 1), RZG2L_PIN_INFO(15, 2),
	RZG2L_PIN_INFO(15, 3),
	RZG2L_PIN_INFO(16, 0), RZG2L_PIN_INFO(16, 1),
	RZG2L_PIN_INFO(17, 0), RZG2L_PIN_INFO(17, 1), RZG2L_PIN_INFO(17, 2),
	RZG2L_PIN_INFO(17, 3),
	RZG2L_PIN_INFO(18, 0), RZG2L_PIN_INFO(18, 1), RZG2L_PIN_INFO(18, 2),
	RZG2L_PIN_INFO(18, 3), RZG2L_PIN_INFO(18, 4), RZG2L_PIN_INFO(18, 5),
};

static const int rzv2h_pin_info[] = {
	RZG2L_PIN_INFO(0,  0), RZG2L_PIN_INFO(0,  1), RZG2L_PIN_INFO(0,  2),
	RZG2L_PIN_INFO(0,  3), RZG2L_PIN_INFO(0,  4), RZG2L_PIN_INFO(0,  5),
	RZG2L_PIN_INFO(0,  6), RZG2L_PIN_INFO(0,  7),
	RZG2L_PIN_INFO(1,  0), RZG2L_PIN_INFO(1,  1), RZG2L_PIN_INFO(1,  2),
	RZG2L_PIN_INFO(1,  3), RZG2L_PIN_INFO(1,  4), RZG2L_PIN_INFO(1,  5),
	RZG2L_PIN_INFO(2,  0), RZG2L_PIN_INFO(2,  1),
	RZG2L_PIN_INFO(3,  0), RZG2L_PIN_INFO(3,  1), RZG2L_PIN_INFO(3,  2),
	RZG2L_PIN_INFO(3,  3), RZG2L_PIN_INFO(3,  4), RZG2L_PIN_INFO(3,  5),
	RZG2L_PIN_INFO(3,  6), RZG2L_PIN_INFO(3,  7),
	RZG2L_PIN_INFO(4,  0), RZG2L_PIN_INFO(4,  1), RZG2L_PIN_INFO(4,  2),
	RZG2L_PIN_INFO(4,  3), RZG2L_PIN_INFO(4,  4), RZG2L_PIN_INFO(4,  5),
	RZG2L_PIN_INFO(4,  6), RZG2L_PIN_INFO(4,  7),
	RZG2L_PIN_INFO(5,  0), RZG2L_PIN_INFO(5,  1), RZG2L_PIN_INFO(5,  2),
	RZG2L_PIN_INFO(5,  3), RZG2L_PIN_INFO(5,  4), RZG2L_PIN_INFO(5,  5),
	RZG2L_PIN_INFO(5,  6), RZG2L_PIN_INFO(5,  7),
	RZG2L_PIN_INFO(6,  0), RZG2L_PIN_INFO(6,  1), RZG2L_PIN_INFO(6,  2),
	RZG2L_PIN_INFO(6,  3), RZG2L_PIN_INFO(6,  4), RZG2L_PIN_INFO(6,  5),
	RZG2L_PIN_INFO(6,  6), RZG2L_PIN_INFO(6,  7),
	RZG2L_PIN_INFO(7,  0), RZG2L_PIN_INFO(7,  1), RZG2L_PIN_INFO(7,  2),
	RZG2L_PIN_INFO(7,  3), RZG2L_PIN_INFO(7,  4), RZG2L_PIN_INFO(7,  5),
	RZG2L_PIN_INFO(7,  6), RZG2L_PIN_INFO(7,  7),
	RZG2L_PIN_INFO(8,  0), RZG2L_PIN_INFO(8,  1), RZG2L_PIN_INFO(8,  2),
	RZG2L_PIN_INFO(8,  3), RZG2L_PIN_INFO(8,  4), RZG2L_PIN_INFO(8,  5),
	RZG2L_PIN_INFO(8,  6), RZG2L_PIN_INFO(8,  7),
	RZG2L_PIN_INFO(9,  0), RZG2L_PIN_INFO(9,  1), RZG2L_PIN_INFO(9,  2),
	RZG2L_PIN_INFO(9,  3), RZG2L_PIN_INFO(9,  4), RZG2L_PIN_INFO(9,  5),
	RZG2L_PIN_INFO(9,  6), RZG2L_PIN_INFO(9,  7),
	RZG2L_PIN_INFO(10, 0), RZG2L_PIN_INFO(10, 1), RZG2L_PIN_INFO(10, 2),
	RZG2L_PIN_INFO(10, 3), RZG2L_PIN_INFO(10, 4), RZG2L_PIN_INFO(10, 5),
	RZG2L_PIN_INFO(10, 6), RZG2L_PIN_INFO(10, 7),
	RZG2L_PIN_INFO(11, 0), RZG2L_PIN_INFO(11, 1), RZG2L_PIN_INFO(11, 2),
	RZG2L_PIN_INFO(11, 3), RZG2L_PIN_INFO(11, 4), RZG2L_PIN_INFO(11, 5),
};

struct rzg2l_dedicated_configs {
	const char *name;
	u32 config;
};

struct rzg2l_backup_data {
	uint8_t  *p_reg;
	uint8_t  *pmc_reg;
	uint16_t *pm_reg;
	uint32_t *pfc_reg;
	uint32_t *isel_reg;
	u8 oen_reg;
};

struct rzg2l_pinctrl_data {
	const char * const *port_pins;
	const u32 *port_pin_configs;
	struct rzg2l_dedicated_configs *dedicated_pins;
	unsigned int n_port_pins;
	unsigned int n_dedicated_pins;
	const unsigned int *pin_info;
	unsigned int ngpioints;
	bool irq_mask;
	u32 extended_reg_offset;
	bool have_clrirq_reg;
	u32 pwpr;
	u32 oen_reg_offset;
	unsigned int n_ports;
};

struct rzg2l_pinctrl {
	struct pinctrl_dev		*pctl;
	struct pinctrl_desc		desc;
	struct pinctrl_pin_desc		*pins;

	const struct rzg2l_pinctrl_data	*data;
	void __iomem			*base;
	void __iomem			*base_tint;
	void __iomem			*tint_irq_mask;
	struct device			*dev;
	struct clk			*clk;

	struct gpio_chip		gpio_chip;
	struct pinctrl_gpio_range	gpio_range;

	struct irq_chip			irq_chip;
	unsigned int			irq_start;
	atomic_t			wakeup_path;

	/* This array will store GPIO IDs for TINT[0-32] with value:
	 * - [15-0] bits: store GPIO IDs (ID = port * 8 + bit).
	 * - [16] bit: store active status (1 for enabled, 0 for disabled).
	 */
	u32				tint[TINT_MAX];

	spinlock_t			lock; /* lock read/write registers */
	struct mutex			mutex; /* serialize adding groups and functions */
	void				*backup;
};

static const unsigned int iolh_groupa_mA[] = { 2, 4, 8, 12 };
static const unsigned int iolh_groupb_oi[] = { 100, 66, 50, 33 };

static void rzg2l_pinctrl_set_pfc_mode(struct rzg2l_pinctrl *pctrl,
				       u8 port, u8 pin, u8 func)
{
	unsigned long flags;
	u32 reg;
	u32 pwpr = pctrl->data->pwpr;

	spin_lock_irqsave(&pctrl->lock, flags);

	port += pctrl->data->extended_reg_offset;

	/* Set pin to 'Non-use (Hi-Z input protection)'  */
	reg = readw(pctrl->base + PM(port));
	reg &= ~(PM_MASK << (pin * 2));
	writew(reg, pctrl->base + PM(port));

	/* Set the PWPR register to allow PFC/PMC register to write */
	if (pwpr == PWPR) {
		writel(0x0, pctrl->base + pwpr);        /* B0WI=0, PFCWE=0 */
		writel(PWPR_PFCWE, pctrl->base + pwpr);  /* B0WI=0, PFCWE=1 */
	} else {
		writel(readl(pctrl->base + pwpr) | PWPR_REGWE_A,
		       pctrl->base + pwpr);
	}

	/* Temporarily switch to GPIO mode with PMC register */
	reg = readb(pctrl->base + PMC(port));
	writeb(reg & ~BIT(pin), pctrl->base + PMC(port));

	/* Select Pin function mode with PFC register */
	reg = readl(pctrl->base + PFC(port));
	reg &= ~(PFC_MASK << (pin * 4));
	writel(reg | (func << (pin * 4)), pctrl->base + PFC(port));


	/* Switch to Peripheral pin function with PMC register */
	reg = readb(pctrl->base + PMC(port));
	writeb(reg | BIT(pin), pctrl->base + PMC(port));

	/* Set the PWPR register to be write-protected */
	if (pwpr == PWPR) {
		writel(0x0, pctrl->base + PWPR);        /* B0WI=0, PFCWE=0 */
		writel(PWPR_B0WI, pctrl->base + PWPR);  /* B0WI=1, PFCWE=0 */
	} else {
		writel(readl(pctrl->base + pwpr) & ~PWPR_REGWE_A,
		       pctrl->base + pwpr);
	}

	spin_unlock_irqrestore(&pctrl->lock, flags);
};

static int rzg2l_pinctrl_set_mux(struct pinctrl_dev *pctldev,
				 unsigned int func_selector,
				 unsigned int group_selector)
{
	struct rzg2l_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct function_desc *func;
	unsigned int i, *psel_val;
	struct group_desc *group;
	int *pins;

	func = pinmux_generic_get_function(pctldev, func_selector);
	if (!func)
		return -EINVAL;
	group = pinctrl_generic_get_group(pctldev, group_selector);
	if (!group)
		return -EINVAL;

	psel_val = func->data;
	pins = group->pins;

	for (i = 0; i < group->num_pins; i++) {
		dev_dbg(pctrl->dev, "port:%u pin: %u PSEL:%u\n",
			RZG2L_PIN_ID_TO_PORT(pins[i]), RZG2L_PIN_ID_TO_PIN(pins[i]),
			psel_val[i]);
		rzg2l_pinctrl_set_pfc_mode(pctrl, RZG2L_PIN_ID_TO_PORT(pins[i]),
					   RZG2L_PIN_ID_TO_PIN(pins[i]), psel_val[i]);
	}

	return 0;
};

static int rzg2l_map_add_config(struct pinctrl_map *map,
				const char *group_or_pin,
				enum pinctrl_map_type type,
				unsigned long *configs,
				unsigned int num_configs)
{
	unsigned long *cfgs;

	cfgs = kmemdup(configs, num_configs * sizeof(*cfgs),
		       GFP_KERNEL);
	if (!cfgs)
		return -ENOMEM;

	map->type = type;
	map->data.configs.group_or_pin = group_or_pin;
	map->data.configs.configs = cfgs;
	map->data.configs.num_configs = num_configs;

	return 0;
}

static int rzg2l_dt_subnode_to_map(struct pinctrl_dev *pctldev,
				   struct device_node *np,
				   struct pinctrl_map **map,
				   unsigned int *num_maps,
				   unsigned int *index)
{
	struct rzg2l_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pinctrl_map *maps = *map;
	unsigned int nmaps = *num_maps;
	unsigned long *configs = NULL;
	unsigned int *pins, *psel_val;
	unsigned int num_pinmux = 0;
	unsigned int idx = *index;
	unsigned int num_pins, i;
	unsigned int num_configs;
	struct property *pinmux;
	struct property *prop;
	int ret, gsel, fsel;
	const char **pin_fn;
	const char *pin;

	pinmux = of_find_property(np, "pinmux", NULL);
	if (pinmux)
		num_pinmux = pinmux->length / sizeof(u32);

	ret = of_property_count_strings(np, "pins");
	if (ret == -EINVAL) {
		num_pins = 0;
	} else if (ret < 0) {
		dev_err(pctrl->dev, "Invalid pins list in DT\n");
		return ret;
	} else {
		num_pins = ret;
	}

	if (!num_pinmux && !num_pins)
		return 0;

	if (num_pinmux && num_pins) {
		dev_err(pctrl->dev,
			"DT node must contain either a pinmux or pins and not both\n");
		return -EINVAL;
	}

	ret = pinconf_generic_parse_dt_config(np, NULL, &configs, &num_configs);
	if (ret < 0)
		return ret;

	if (num_pins && !num_configs) {
		dev_err(pctrl->dev, "DT node must contain a config\n");
		ret = -ENODEV;
		goto done;
	}

	if (num_pinmux)
		nmaps += 1;

	if (num_pins)
		nmaps += num_pins;

	if (configs && num_pinmux)
		nmaps += 1;

	maps = krealloc(maps, nmaps * sizeof(*maps), GFP_KERNEL);
	if (!maps) {
		ret = -ENOMEM;
		goto done;
	}

	*map = maps;
	*num_maps = nmaps;
	if (num_pins) {
		of_property_for_each_string(np, "pins", prop, pin) {
			ret = rzg2l_map_add_config(&maps[idx], pin,
						   PIN_MAP_TYPE_CONFIGS_PIN,
						   configs, num_configs);
			if (ret < 0)
				goto done;

			idx++;
		}
		ret = 0;
		goto done;
	}

	pins = devm_kcalloc(pctrl->dev, num_pinmux, sizeof(*pins), GFP_KERNEL);
	psel_val = devm_kcalloc(pctrl->dev, num_pinmux, sizeof(*psel_val),
				GFP_KERNEL);
	pin_fn = devm_kzalloc(pctrl->dev, sizeof(*pin_fn), GFP_KERNEL);
	if (!pins || !psel_val || !pin_fn) {
		ret = -ENOMEM;
		goto done;
	}

	/* Collect pin locations and mux settings from DT properties */
	for (i = 0; i < num_pinmux; ++i) {
		u32 value;

		ret = of_property_read_u32_index(np, "pinmux", i, &value);
		if (ret)
			goto done;
		pins[i] = value & MUX_PIN_ID_MASK;
		psel_val[i] = MUX_FUNC(value);
	}

	mutex_lock(&pctrl->mutex);

	/* Register a single pin group listing all the pins we read from DT */
	gsel = pinctrl_generic_add_group(pctldev, np->name, pins, num_pinmux, NULL);
	if (gsel < 0) {
		ret = gsel;
		goto unlock;
	}

	/*
	 * Register a single group function where the 'data' is an array PSEL
	 * register values read from DT.
	 */
	pin_fn[0] = np->name;
	fsel = pinmux_generic_add_function(pctldev, np->name, pin_fn, 1,
					   psel_val);
	if (fsel < 0) {
		ret = fsel;
		goto remove_group;
	}

	mutex_unlock(&pctrl->mutex);

	maps[idx].data.mux.group = np->name;
	maps[idx].data.mux.function = np->name;
	maps[idx].type = PIN_MAP_TYPE_MUX_GROUP;
	idx++;

	if (num_configs) {
		ret = rzg2l_map_add_config(&maps[idx], np->name,
					   PIN_MAP_TYPE_CONFIGS_GROUP,
					   configs, num_configs);
		if (ret < 0)
			goto remove_group;

		idx++;
	};

	dev_dbg(pctrl->dev, "Parsed %pOF with %d pins\n", np, num_pinmux);
	ret = 0;
	goto done;

remove_group:
	pinctrl_generic_remove_group(pctldev, gsel);
unlock:
	mutex_unlock(&pctrl->mutex);
done:
	*index = idx;
	kfree(configs);
	return ret;
}

static void rzg2l_dt_free_map(struct pinctrl_dev *pctldev,
			      struct pinctrl_map *map,
			      unsigned int num_maps)
{
	unsigned int i;

	if (!map)
		return;

	for (i = 0; i < num_maps; ++i) {
		if (map[i].type == PIN_MAP_TYPE_CONFIGS_GROUP ||
		    map[i].type == PIN_MAP_TYPE_CONFIGS_PIN)
			kfree(map[i].data.configs.configs);
	}
	kfree(map);
}

static int rzg2l_dt_node_to_map(struct pinctrl_dev *pctldev,
				struct device_node *np,
				struct pinctrl_map **map,
				unsigned int *num_maps)
{
	struct rzg2l_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct device_node *child;
	unsigned int index;
	int ret;

	*map = NULL;
	*num_maps = 0;
	index = 0;

	for_each_child_of_node(np, child) {
		ret = rzg2l_dt_subnode_to_map(pctldev, child, map,
					      num_maps, &index);
		if (ret < 0) {
			of_node_put(child);
			goto done;
		}
	}

	if (*num_maps == 0) {
		ret = rzg2l_dt_subnode_to_map(pctldev, np, map,
					      num_maps, &index);
		if (ret < 0)
			goto done;
	}

	if (*num_maps)
		return 0;

	dev_err(pctrl->dev, "no mapping found in node %pOF\n", np);
	ret = -EINVAL;

done:
	if (ret < 0)
		rzg2l_dt_free_map(pctldev, *map, *num_maps);

	return ret;
}

static int rzg2l_validate_gpio_pin(struct rzg2l_pinctrl *pctrl,
				   u32 cfg, u32 port, u8 bit)
{
	u8 pincount = RZG2L_GPIO_PORT_GET_PINCNT(cfg);
	u32 port_index = RZG2L_GPIO_PORT_GET_INDEX(cfg);
	u32 data;

	if (bit >= pincount || port >= pctrl->data->n_port_pins)
		return -EINVAL;

	data = pctrl->data->port_pin_configs[port];
	if (port_index != RZG2L_GPIO_PORT_GET_INDEX(data))
		return -EINVAL;

	return 0;
}

static u32 rzg2l_read_pin_config(struct rzg2l_pinctrl *pctrl, u32 offset,
				 u8 bit, u32 mask)
{
	void __iomem *addr = pctrl->base + offset;

	/* handle _L/_H for 32-bit register read/write */
	if (bit >= 4) {
		bit -= 4;
		addr += 4;
	}

	return (readl(addr) >> (bit * 8)) & mask;
}

static void rzg2l_rmw_pin_config(struct rzg2l_pinctrl *pctrl, u32 offset,
				 u8 bit, u32 mask, u32 val)
{
	void __iomem *addr = pctrl->base + offset;
	unsigned long flags;
	u32 reg;

	/* handle _L/_H for 32-bit register read/write */
	if (bit >= 4) {
		bit -= 4;
		addr += 4;
	}

	spin_lock_irqsave(&pctrl->lock, flags);
	reg = readl(addr) & ~(mask << (bit * 8));
	writel(reg | (val << (bit * 8)), addr);
	spin_unlock_irqrestore(&pctrl->lock, flags);
}

static int rzg2l_pinctrl_pinconf_get(struct pinctrl_dev *pctldev,
				     unsigned int _pin,
				     unsigned long *config)
{
	struct rzg2l_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	const struct pinctrl_pin_desc *pin = &pctrl->desc.pins[_pin];
	unsigned int *pin_data = pin->drv_data;
	unsigned int arg = 0;
	unsigned long flags;
	void __iomem *addr;
	u32 port_offset;
	u32 cfg = 0;
	u8 bit = 0;

	if (!pin_data)
		return -EINVAL;

	if (*pin_data & RZG2L_SINGLE_PIN) {
		port_offset = RZG2L_SINGLE_PIN_GET_PORT_OFFSET(*pin_data);
		cfg = RZG2L_SINGLE_PIN_GET_CFGS(*pin_data);
		bit = RZG2L_SINGLE_PIN_GET_BIT(*pin_data);
	} else {
		cfg = RZG2L_GPIO_PORT_GET_CFGS(*pin_data);
		port_offset = RZG2L_PIN_ID_TO_PORT_OFFSET(_pin);
		bit = RZG2L_PIN_ID_TO_PIN(_pin);

		if (rzg2l_validate_gpio_pin(pctrl, *pin_data, RZG2L_PIN_ID_TO_PORT(_pin), bit))
			return -EINVAL;
	}

	switch (param) {
	case PIN_CONFIG_INPUT_ENABLE:
		if (!(cfg & PIN_CFG_IEN))
			return -EINVAL;
		arg = rzg2l_read_pin_config(pctrl, IEN(port_offset), bit, IEN_MASK);
		if (!arg)
			return -EINVAL;
		break;

	case PIN_CONFIG_POWER_SOURCE: {
		u32 pwr_reg = 0x0;

		if (cfg & PIN_CFG_IO_VMC_SD0)
			pwr_reg = SD_CH(0);
		else if (cfg & PIN_CFG_IO_VMC_SD1)
			pwr_reg = SD_CH(1);
		else if (cfg & PIN_CFG_IO_VMC_QSPI)
			pwr_reg = QSPI;
		else if (cfg & PIN_CFG_IO_VMC_ETH0)
			pwr_reg = ETH_CH(0);
		else if (cfg & PIN_CFG_IO_VMC_ETH1)
			pwr_reg = ETH_CH(1);
		else
			return -EINVAL;

		spin_lock_irqsave(&pctrl->lock, flags);
		addr = pctrl->base + pwr_reg;
		if (cfg & PIN_CFG_IO_VMC_ETH) {
			arg = (readl(addr) & ETH_PVDD_MASK);
			arg = arg ?
			      ((arg == ETH_PVDD_1800) ? 1800 : 2500) :
			      3300;
		} else {
			arg = (readl(addr) & PVDD_MASK) ? 1800 : 3300;
		}
		spin_unlock_irqrestore(&pctrl->lock, flags);
		break;
	}

	case PIN_CONFIG_DRIVE_STRENGTH: {
		unsigned int index;

		if (!(cfg & PIN_CFG_IOLH_A))
			return -EINVAL;

		index = rzg2l_read_pin_config(pctrl, IOLH(port_offset), bit, IOLH_MASK);
		arg = iolh_groupa_mA[index];
		break;
	}

	case PIN_CONFIG_OUTPUT_IMPEDANCE_OHMS: {
		unsigned int index;

		if (!(cfg & PIN_CFG_IOLH_B))
			return -EINVAL;

		index = rzg2l_read_pin_config(pctrl, IOLH(port_offset), bit, IOLH_MASK);
		arg = iolh_groupb_oi[index];
		break;
	}

	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		arg = rzg2l_read_pin_config(pctrl, NOD(port_offset), bit, NOD_MASK);
		if (!arg && param != PIN_CONFIG_DRIVE_PUSH_PULL)
		return -EINVAL;
		if (arg && param != PIN_CONFIG_DRIVE_OPEN_DRAIN)
			return -EINVAL;
		break;

	case PIN_CONFIG_SLEW_RATE: {
		if (!(cfg & PIN_CFG_SR))
			return -EINVAL;

		arg = rzg2l_read_pin_config(pctrl, SR(port_offset), bit, SR_MASK);
		break;
	}

	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
};

static int rzg2l_pinctrl_pinconf_set(struct pinctrl_dev *pctldev,
				     unsigned int _pin,
				     unsigned long *_configs,
				     unsigned int num_configs)
{
	struct rzg2l_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct pinctrl_pin_desc *pin = &pctrl->desc.pins[_pin];
	unsigned int *pin_data = pin->drv_data;
	enum pin_config_param param;
	unsigned long flags;
	void __iomem *addr;
	u32 port_offset;
	unsigned int i;
	u32 cfg = 0;
	u8 bit = 0;

	if (!pin_data)
		return -EINVAL;

	if (*pin_data & RZG2L_SINGLE_PIN) {
		port_offset = RZG2L_SINGLE_PIN_GET_PORT_OFFSET(*pin_data);
		cfg = RZG2L_SINGLE_PIN_GET_CFGS(*pin_data);
		bit = RZG2L_SINGLE_PIN_GET_BIT(*pin_data);
	} else {
		cfg = RZG2L_GPIO_PORT_GET_CFGS(*pin_data);
		port_offset = RZG2L_PIN_ID_TO_PORT_OFFSET(_pin);
		bit = RZG2L_PIN_ID_TO_PIN(_pin);

		if (rzg2l_validate_gpio_pin(pctrl, *pin_data, RZG2L_PIN_ID_TO_PORT(_pin), bit))
			return -EINVAL;
	}

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(_configs[i]);
		switch (param) {
		case PIN_CONFIG_INPUT_ENABLE: {
			unsigned int arg =
					pinconf_to_config_argument(_configs[i]);

			if (!(cfg & PIN_CFG_IEN))
				return -EINVAL;

			rzg2l_rmw_pin_config(pctrl, IEN(port_offset), bit, IEN_MASK, !!arg);
			break;
		}

		case PIN_CONFIG_POWER_SOURCE: {
			unsigned int mV = pinconf_to_config_argument(_configs[i]);
			u32 pwr_reg = 0x0;

			if (mV != 1800 && mV != 3300)
				if (!((mV == 2500) && (cfg & PIN_CFG_IO_VMC_ETH)))
					return -EINVAL;

			if (cfg & PIN_CFG_IO_VMC_SD0)
				pwr_reg = SD_CH(0);
			else if (cfg & PIN_CFG_IO_VMC_SD1)
				pwr_reg = SD_CH(1);
			else if (cfg & PIN_CFG_IO_VMC_QSPI)
				pwr_reg = QSPI;
			else if (cfg & PIN_CFG_IO_VMC_ETH0)
				pwr_reg = ETH_CH(0);
			else if (cfg & PIN_CFG_IO_VMC_ETH1)
				pwr_reg = ETH_CH(1);
			else
				return -EINVAL;

			addr = pctrl->base + pwr_reg;
			spin_lock_irqsave(&pctrl->lock, flags);
			if (cfg & PIN_CFG_IO_VMC_ETH)
				writel((mV == 3300) ? ETH_PVDD_3300 :
				      ((mV == 2500) ? ETH_PVDD_2500 :
						      ETH_PVDD_1800), addr);
			else
				writel((mV == 1800) ? PVDD_1800 :
						      PVDD_3300, addr);
			spin_unlock_irqrestore(&pctrl->lock, flags);
			break;
		}

		case PIN_CONFIG_DRIVE_STRENGTH: {
			unsigned int arg = pinconf_to_config_argument(_configs[i]);
			unsigned int index;

			if (!(cfg & PIN_CFG_IOLH_A))
				return -EINVAL;

			for (index = 0; index < ARRAY_SIZE(iolh_groupa_mA); index++) {
				if (arg == iolh_groupa_mA[index])
					break;
			}
			if (index >= ARRAY_SIZE(iolh_groupa_mA))
				return -EINVAL;

			rzg2l_rmw_pin_config(pctrl, IOLH(port_offset), bit, IOLH_MASK, index);
			break;
		}

		case PIN_CONFIG_OUTPUT_IMPEDANCE_OHMS: {
			unsigned int arg = pinconf_to_config_argument(_configs[i]);
			unsigned int index;

			if (!(cfg & PIN_CFG_IOLH_B))
				return -EINVAL;

			for (index = 0; index < ARRAY_SIZE(iolh_groupb_oi); index++) {
				if (arg == iolh_groupb_oi[index])
					break;
			}
			if (index >= ARRAY_SIZE(iolh_groupb_oi))
				return -EINVAL;

			rzg2l_rmw_pin_config(pctrl, IOLH(port_offset), bit, IOLH_MASK, index);
			break;
		}

		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			rzg2l_rmw_pin_config(pctrl, NOD(port_offset), bit, NOD_MASK,
			param == PIN_CONFIG_DRIVE_OPEN_DRAIN ? 1 : 0);
			break;

		case PIN_CONFIG_SLEW_RATE: {
			unsigned int arg =
					pinconf_to_config_argument(_configs[i]);

			if (!(cfg & PIN_CFG_SR))
				return -EINVAL;

			rzg2l_rmw_pin_config(pctrl, SR(port_offset),
					     bit, SR_MASK, !!arg);
			break;
		}

		default:
			return -EOPNOTSUPP;
		}
	}

	return 0;
}

static int rzg2l_pinctrl_pinconf_group_set(struct pinctrl_dev *pctldev,
					   unsigned int group,
					   unsigned long *configs,
					   unsigned int num_configs)
{
	const unsigned int *pins;
	unsigned int i, npins;
	int ret;

	ret = pinctrl_generic_get_group_pins(pctldev, group, &pins, &npins);
	if (ret)
		return ret;

	for (i = 0; i < npins; i++) {
		ret = rzg2l_pinctrl_pinconf_set(pctldev, pins[i], configs,
						num_configs);
		if (ret)
			return ret;
	}

	return 0;
};

static int rzg2l_pinctrl_pinconf_group_get(struct pinctrl_dev *pctldev,
					   unsigned int group,
					   unsigned long *config)
{
	const unsigned int *pins;
	unsigned int i, npins, prev_config = 0;
	int ret;

	ret = pinctrl_generic_get_group_pins(pctldev, group, &pins, &npins);
	if (ret)
		return ret;

	for (i = 0; i < npins; i++) {
		ret = rzg2l_pinctrl_pinconf_get(pctldev, pins[i], config);
		if (ret)
			return ret;

		/* Check config matching between to pin  */
		if (i && prev_config != *config)
			return -EOPNOTSUPP;

		prev_config = *config;
	}

	return 0;
};

static const struct pinctrl_ops rzg2l_pinctrl_pctlops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = rzg2l_dt_node_to_map,
	.dt_free_map = rzg2l_dt_free_map,
};

static const struct pinmux_ops rzg2l_pinctrl_pmxops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = rzg2l_pinctrl_set_mux,
	.strict = true,
};

static const struct pinconf_ops rzg2l_pinctrl_confops = {
	.is_generic = true,
	.pin_config_get = rzg2l_pinctrl_pinconf_get,
	.pin_config_set = rzg2l_pinctrl_pinconf_set,
	.pin_config_group_set = rzg2l_pinctrl_pinconf_group_set,
	.pin_config_group_get = rzg2l_pinctrl_pinconf_group_get,
	.pin_config_config_dbg_show = pinconf_generic_dump_config,
};

static int rzg2l_gpio_irq_validate_id(struct rzg2l_pinctrl *pctrl, u32 port,
				      u32 bit)
{
	int i;
	u32 pin_info = (port << 16) | bit;

	for (i = 0; i < pctrl->data->ngpioints ; i++) {
		if (pin_info == pctrl->data->pin_info[i])
			break;
	}

	return i;
}

static int rzg2l_gpio_irq_request_tint_slot(struct rzg2l_pinctrl *pctrl)
{
	int i;

	for (i = 0; i < TINT_MAX; i++) {
		if (pctrl->tint[i] == 0)
			break;
	}

	return i;
}

static int rzg2l_gpio_irq_check_tint_slot(struct rzg2l_pinctrl *pctrl,
					  u32 gpio_id)
{
	int i;

	for (i = 0; i < TINT_MAX; i++) {
		if (pctrl->tint[i] == (BIT(16) | gpio_id))
			break;
	}

	return i;
}

static void rzg2l_gpio_irq_shutdown(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	int hw_irq = irqd_to_hwirq(d);
	u32 port = RZG2L_PIN_ID_TO_PORT(hw_irq);
	u8 bit = RZG2L_PIN_ID_TO_PIN(hw_irq);
	u32 gpioint;
	u32 tint_slot;
	unsigned long flags;
	u64 reg64;
	u32 reg32;

	gpioint = rzg2l_gpio_irq_validate_id(pctrl, port, bit);
	if (gpioint == pctrl->data->ngpioints)
		return;

	tint_slot = rzg2l_gpio_irq_check_tint_slot(pctrl, hw_irq);
	if (tint_slot ==  TINT_MAX)
		return;

	spin_lock_irqsave(&pctrl->lock, flags);

	port += pctrl->data->extended_reg_offset;

	reg64 = readq(pctrl->base + ISEL(port));
	reg64 &= ~BIT(bit * 8);
	writeq(reg64, pctrl->base + ISEL(port));

	reg32 = readl(pctrl->base_tint + TSSR(tint_slot / 4));
	reg32 &= ~(GENMASK(7, 0) << ((tint_slot % 4) * 8));
	writel(reg32, pctrl->base_tint + TSSR(tint_slot / 4));

	if (pctrl->data->irq_mask) {
		reg32 = readl(pctrl->tint_irq_mask + TMSK);
		reg32 |= BIT(tint_slot);
		writel(reg32, pctrl->tint_irq_mask + TMSK);
	}

	spin_unlock_irqrestore(&pctrl->lock, flags);

	pctrl->tint[tint_slot] = 0;
}

static void rzg2l_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	int hw_irq = irqd_to_hwirq(d);
	u32 port = RZG2L_PIN_ID_TO_PORT(hw_irq);
	u8 bit = RZG2L_PIN_ID_TO_PIN(hw_irq);
	u32 gpioint;
	u32 tint_slot;
	unsigned long flags;
	u32 reg32;

	gpioint = rzg2l_gpio_irq_validate_id(pctrl, port, bit);
	if (gpioint == pctrl->data->ngpioints)
		return;

	tint_slot = rzg2l_gpio_irq_check_tint_slot(pctrl, hw_irq);
	if (tint_slot ==  TINT_MAX)
		return;

	spin_lock_irqsave(&pctrl->lock, flags);

	if (pctrl->data->irq_mask) {
		reg32 = readl(pctrl->tint_irq_mask + TMSK);
		reg32 |= BIT(tint_slot);
		writel(reg32, pctrl->tint_irq_mask + TMSK);
	} else {
		reg32 = readl(pctrl->base_tint + TSSR(tint_slot / 4));
		reg32 &= ~(BIT(7) << ((tint_slot % 4) * 8));
		writel(reg32, pctrl->base_tint + TSSR(tint_slot / 4));
	}

	spin_unlock_irqrestore(&pctrl->lock, flags);
}

static void rzg2l_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	int hw_irq = irqd_to_hwirq(d);
	u32 port = RZG2L_PIN_ID_TO_PORT(hw_irq);
	u8 bit = RZG2L_PIN_ID_TO_PIN(hw_irq);
	u32 gpioint;
	u32 tint_slot;
	unsigned long flags;
	u32 reg32;

	gpioint = rzg2l_gpio_irq_validate_id(pctrl, port, bit);
	if (gpioint == pctrl->data->ngpioints)
		return;

	tint_slot = rzg2l_gpio_irq_check_tint_slot(pctrl, hw_irq);
	if (tint_slot ==  TINT_MAX)
		return;

	spin_lock_irqsave(&pctrl->lock, flags);

	if (pctrl->data->irq_mask) {
		reg32 = readl(pctrl->tint_irq_mask + TMSK);
		reg32 &= ~BIT(tint_slot);
		writel(reg32, pctrl->tint_irq_mask + TMSK);
	} else {
		u32 irq_type;

		if (tint_slot > 15) {
			reg32 = readl(pctrl->base_tint +
				      TITSR1(pctrl->data->have_clrirq_reg));
			reg32 = reg32 >> ((tint_slot - 16) * 2);
			irq_type = reg32 & IRQ_MASK;
		} else {
			reg32 = readl(pctrl->base_tint +
				      TITSR0(pctrl->data->have_clrirq_reg));
			reg32 = reg32 >> (tint_slot * 2);
			irq_type = reg32 & IRQ_MASK;
		}

		reg32 = readl(pctrl->base_tint + TSSR(tint_slot / 4));
		reg32 |= BIT(7) << (8 * (tint_slot % 4));
		writel(reg32, pctrl->base_tint + TSSR(tint_slot / 4));

		/* Clear Interrupt status bit to avoid unexpected triggering */
		if ((irq_type == RISING_EDGE) || (irq_type == FALLING_EDGE)) {
			if(pctrl->data->have_clrirq_reg) {
				writel(BIT(tint_slot),
				       pctrl->base_tint + TSCLR);
			} else {
				reg32 = readl(pctrl->base_tint + TSCR);
				writel(reg32 & ~BIT(tint_slot),
				       pctrl->base_tint + TSCR);
			}
		}
	}

	spin_unlock_irqrestore(&pctrl->lock, flags);
}

static int rzg2l_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	int hw_irq = irqd_to_hwirq(d);
	u32 port = RZG2L_PIN_ID_TO_PORT(hw_irq);
	u8 bit = RZG2L_PIN_ID_TO_PIN(hw_irq);
	u32 gpioint;
	u32 tint_slot;
	unsigned long flags;
	u32 irq_type;
	u64 reg64;
	u32 reg32;
	u8 reg8;

	gpioint = rzg2l_gpio_irq_validate_id(pctrl, port, bit);
	if (gpioint == pctrl->data->ngpioints)
		return -EINVAL;

	tint_slot = rzg2l_gpio_irq_check_tint_slot(pctrl, hw_irq);
	if (tint_slot ==  TINT_MAX) {
		tint_slot = rzg2l_gpio_irq_request_tint_slot(pctrl);
		if (tint_slot ==  TINT_MAX)
			return -EINVAL;
	}

	switch (type & IRQ_TYPE_SENSE_MASK) {
	/*
	 * Currently we just support interrupt edge type.
	 * About level type, we do not support because we can not clear
	 * after triggering.
	 */
	case IRQ_TYPE_EDGE_RISING:
		irq_type = RISING_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		irq_type = FALLING_EDGE;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		irq_type = LOW_LEVEL;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		irq_type = HIGH_LEVEL;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&pctrl->lock, flags);

	port += pctrl->data->extended_reg_offset;

        /* Set the PWPR register to allow PMC register to write */
        if (pctrl->data->pwpr == PWPR_RZ_V2H)
                writel(readl(pctrl->base + pctrl->data->pwpr) | PWPR_REGWE_A,
		       pctrl->base + pctrl->data->pwpr);

	/* Select GPIO mode in PMC Register before enabling interrupt mode */
	reg8 = readb(pctrl->base + PMC(port));
	reg8 &= ~BIT(bit);
	writeb(reg8, pctrl->base + PMC(port));

        /* Set the PWPR register to disallow PMC register to write */
        if (pctrl->data->pwpr == PWPR_RZ_V2H)
                writel(readl(pctrl->base + pctrl->data->pwpr) & ~PWPR_REGWE_A,
		       pctrl->base + pctrl->data->pwpr);

	/* Select Interrupt Mode */
	reg64 = readq(pctrl->base + ISEL(port));
	reg64 |= BIT(bit * 8);
	writeq(reg64, pctrl->base + ISEL(port));

	pctrl->tint[tint_slot] = BIT(16) | hw_irq;

	if (tint_slot > 15) {
		reg32 = readl(pctrl->base_tint +
			      TITSR1(pctrl->data->have_clrirq_reg));
		reg32 &= ~(IRQ_MASK << ((tint_slot - 16) * 2));
		reg32 |= irq_type << ((tint_slot - 16) * 2);
		writel(reg32, pctrl->base_tint +
			      TITSR1(pctrl->data->have_clrirq_reg));
	} else {
		reg32 = readl(pctrl->base_tint +
			      TITSR0(pctrl->data->have_clrirq_reg));
		reg32 &= ~(IRQ_MASK << (tint_slot * 2));
		reg32 |= irq_type << (tint_slot * 2);
		writel(reg32, pctrl->base_tint +
			      TITSR0(pctrl->data->have_clrirq_reg));
	}

	reg32 = readl(pctrl->base_tint + TSSR(tint_slot / 4));
	if (pctrl->data->irq_mask)
		reg32 |= (BIT(7) | gpioint) << (8 * (tint_slot % 4));
	else
		reg32 |= gpioint << (8 * (tint_slot % 4));
	writel(reg32, pctrl->base_tint + TSSR(tint_slot / 4));

	/* Clear Interrupt status bit to avoid unexpected triggering */
	if ((pctrl->data->irq_mask) &&
	   ((irq_type == RISING_EDGE) || (irq_type == FALLING_EDGE))) {
		reg32 = readl(pctrl->base_tint + TSCR);
		writel(reg32 & ~BIT(tint_slot), pctrl->base_tint + TSCR);
	}

	spin_unlock_irqrestore(&pctrl->lock, flags);

	return 0;
}

static int rzg2l_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	int hw_irq = irqd_to_hwirq(d);
	u32 tint_slot;

	tint_slot = rzg2l_gpio_irq_check_tint_slot(pctrl, hw_irq);
	if (tint_slot ==  TINT_MAX)
		return -EINVAL;

	irq_set_irq_wake(pctrl->irq_start + tint_slot, on);
	if (on)
		atomic_inc(&pctrl->wakeup_path);
	else
		atomic_dec(&pctrl->wakeup_path);

	return 0;
}

static irqreturn_t rzg2l_pinctrl_irq_handler(int irq, void *dev_id)
{
	struct rzg2l_pinctrl *pctrl = dev_id;
	unsigned int offset = irq - pctrl->irq_start;
	u32 reg32;

	if (pctrl->data->have_clrirq_reg) {
		writel(BIT(offset), pctrl->base_tint + TSCLR);
	} else {
		reg32 = readl(pctrl->base_tint + TSCR);
		writel(reg32 & ~BIT(offset), pctrl->base_tint + TSCR);
	}

	generic_handle_irq(irq_find_mapping(pctrl->gpio_chip.irq.domain,
					    pctrl->tint[offset] & ~BIT(16)));

	return IRQ_HANDLED;
}

static int rzg2l_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	u32 port = RZG2L_PIN_ID_TO_PORT(offset);
	u8 bit = RZG2L_PIN_ID_TO_PIN(offset);
	unsigned long flags;
	u8 reg8;
	int ret;

	ret = pinctrl_gpio_request(chip->base + offset);
	if (ret)
		return ret;

	spin_lock_irqsave(&pctrl->lock, flags);

	port += pctrl->data->extended_reg_offset;

	/* Set the PWPR register to allow PMC register to write */
	if (pctrl->data->pwpr == PWPR_RZ_V2H)
		writel(readl(pctrl->base + pctrl->data->pwpr) | PWPR_REGWE_A,
		       pctrl->base + pctrl->data->pwpr);

	/* Select GPIO mode in PMC Register */
	reg8 = readb(pctrl->base + PMC(port));
	reg8 &= ~BIT(bit);
	writeb(reg8, pctrl->base + PMC(port));

	/* Set the PWPR register to disallow PMC register to write */
	if (pctrl->data->pwpr == PWPR_RZ_V2H)
		writel(readl(pctrl->base + pctrl->data->pwpr) & ~PWPR_REGWE_A,
		       pctrl->base + pctrl->data->pwpr);

	spin_unlock_irqrestore(&pctrl->lock, flags);

	return 0;
}

static void rzg2l_gpio_set_direction(struct rzg2l_pinctrl *pctrl, u32 port,
				     u8 bit, bool output)
{
	unsigned long flags;
	u16 reg16;

	spin_lock_irqsave(&pctrl->lock, flags);

	port += pctrl->data->extended_reg_offset;

	reg16 = readw(pctrl->base + PM(port));
	reg16 &= ~(PM_MASK << (bit * 2));

	reg16 |= (output ? PM_OUTPUT : PM_INPUT) << (bit * 2);
	writew(reg16, pctrl->base + PM(port));

	spin_unlock_irqrestore(&pctrl->lock, flags);
}

static int rzg2l_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	u32 port = RZG2L_PIN_ID_TO_PORT(offset);
	u8 bit = RZG2L_PIN_ID_TO_PIN(offset);

	port += pctrl->data->extended_reg_offset;

	if (!(readb(pctrl->base + PMC(port)) & BIT(bit))) {
		u16 reg16;

		reg16 = readw(pctrl->base + PM(port));
		reg16 = (reg16 >> (bit * 2)) & PM_MASK;
		if (reg16 == PM_OUTPUT)
			return GPIO_LINE_DIRECTION_OUT;
	}

	return GPIO_LINE_DIRECTION_IN;
}

static int rzg2l_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	u32 port = RZG2L_PIN_ID_TO_PORT(offset);
	u8 bit = RZG2L_PIN_ID_TO_PIN(offset);

	rzg2l_gpio_set_direction(pctrl, port, bit, false);

	return 0;
}

static void rzg2l_gpio_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	u32 port = RZG2L_PIN_ID_TO_PORT(offset);
	u8 bit = RZG2L_PIN_ID_TO_PIN(offset);
	unsigned long flags;
	u8 reg8;

	spin_lock_irqsave(&pctrl->lock, flags);

	port += pctrl->data->extended_reg_offset;

	reg8 = readb(pctrl->base + P(port));

	if (value)
		writeb(reg8 | BIT(bit), pctrl->base + P(port));
	else
		writeb(reg8 & ~BIT(bit), pctrl->base + P(port));

	spin_unlock_irqrestore(&pctrl->lock, flags);
}

static int rzg2l_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	u32 port = RZG2L_PIN_ID_TO_PORT(offset);
	u8 bit = RZG2L_PIN_ID_TO_PIN(offset);

	rzg2l_gpio_set(chip, offset, value);
	rzg2l_gpio_set_direction(pctrl, port, bit, true);

	return 0;
}

static int rzg2l_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct rzg2l_pinctrl *pctrl = gpiochip_get_data(chip);
	u32 port = RZG2L_PIN_ID_TO_PORT(offset);
	u8 bit = RZG2L_PIN_ID_TO_PIN(offset);
	u16 reg16;

	port += pctrl->data->extended_reg_offset;

	reg16 = readw(pctrl->base + PM(port));
	reg16 = (reg16 >> (bit * 2)) & PM_MASK;

	if (reg16 == PM_INPUT)
		return !!(readb(pctrl->base + PIN(port)) & BIT(bit));
	else if (reg16 == PM_OUTPUT)
		return !!(readb(pctrl->base + P(port)) & BIT(bit));
	else
		return -EINVAL;
}

static void rzg2l_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	pinctrl_gpio_free(chip->base + offset);

	/*
	 * Set the GPIO as an input to ensure that the next GPIO request won't
	 * drive the GPIO pin as an output.
	 */
	if (rzg2l_gpio_get_direction(chip, offset) == GPIO_LINE_DIRECTION_IN)
		rzg2l_gpio_direction_input(chip, offset);
}

static const char * const rzg2l_gpio_names[] = {
	"P0_0", "P0_1", "P0_2", "P0_3", "P0_4", "P0_5", "P0_6", "P0_7",
	"P1_0", "P1_1", "P1_2", "P1_3", "P1_4", "P1_5", "P1_6", "P1_7",
	"P2_0", "P2_1", "P2_2", "P2_3", "P2_4", "P2_5", "P2_6", "P2_7",
	"P3_0", "P3_1", "P3_2", "P3_3", "P3_4", "P3_5", "P3_6", "P3_7",
	"P4_0", "P4_1", "P4_2", "P4_3", "P4_4", "P4_5", "P4_6", "P4_7",
	"P5_0", "P5_1", "P5_2", "P5_3", "P5_4", "P5_5", "P5_6", "P5_7",
	"P6_0", "P6_1", "P6_2", "P6_3", "P6_4", "P6_5", "P6_6", "P6_7",
	"P7_0", "P7_1", "P7_2", "P7_3", "P7_4", "P7_5", "P7_6", "P7_7",
	"P8_0", "P8_1", "P8_2", "P8_3", "P8_4", "P8_5", "P8_6", "P8_7",
	"P9_0", "P9_1", "P9_2", "P9_3", "P9_4", "P9_5", "P9_6", "P9_7",
	"P10_0", "P10_1", "P10_2", "P10_3", "P10_4", "P10_5", "P10_6", "P10_7",
	"P11_0", "P11_1", "P11_2", "P11_3", "P11_4", "P11_5", "P11_6", "P11_7",
	"P12_0", "P12_1", "P12_2", "P12_3", "P12_4", "P12_5", "P12_6", "P12_7",
	"P13_0", "P13_1", "P13_2", "P13_3", "P13_4", "P13_5", "P13_6", "P13_7",
	"P14_0", "P14_1", "P14_2", "P14_3", "P14_4", "P14_5", "P14_6", "P14_7",
	"P15_0", "P15_1", "P15_2", "P15_3", "P15_4", "P15_5", "P15_6", "P15_7",
	"P16_0", "P16_1", "P16_2", "P16_3", "P16_4", "P16_5", "P16_6", "P16_7",
	"P17_0", "P17_1", "P17_2", "P17_3", "P17_4", "P17_5", "P17_6", "P17_7",
	"P18_0", "P18_1", "P18_2", "P18_3", "P18_4", "P18_5", "P18_6", "P18_7",
	"P19_0", "P19_1", "P19_2", "P19_3", "P19_4", "P19_5", "P19_6", "P19_7",
	"P20_0", "P20_1", "P20_2", "P20_3", "P20_4", "P20_5", "P20_6", "P20_7",
	"P21_0", "P21_1", "P21_2", "P21_3", "P21_4", "P21_5", "P21_6", "P21_7",
	"P22_0", "P22_1", "P22_2", "P22_3", "P22_4", "P22_5", "P22_6", "P22_7",
	"P23_0", "P23_1", "P23_2", "P23_3", "P23_4", "P23_5", "P23_6", "P23_7",
	"P24_0", "P24_1", "P24_2", "P24_3", "P24_4", "P24_5", "P24_6", "P24_7",
	"P25_0", "P25_1", "P25_2", "P25_3", "P25_4", "P25_5", "P25_6", "P25_7",
	"P26_0", "P26_1", "P26_2", "P26_3", "P26_4", "P26_5", "P26_6", "P26_7",
	"P27_0", "P27_1", "P27_2", "P27_3", "P27_4", "P27_5", "P27_6", "P27_7",
	"P28_0", "P28_1", "P28_2", "P28_3", "P28_4", "P28_5", "P28_6", "P28_7",
	"P29_0", "P29_1", "P29_2", "P29_3", "P29_4", "P29_5", "P29_6", "P29_7",
	"P30_0", "P30_1", "P30_2", "P30_3", "P30_4", "P30_5", "P30_6", "P30_7",
	"P31_0", "P31_1", "P31_2", "P31_3", "P31_4", "P31_5", "P31_6", "P31_7",
	"P32_0", "P32_1", "P32_2", "P32_3", "P32_4", "P32_5", "P32_6", "P32_7",
	"P33_0", "P33_1", "P33_2", "P33_3", "P33_4", "P33_5", "P33_6", "P33_7",
	"P34_0", "P34_1", "P34_2", "P34_3", "P34_4", "P34_5", "P34_6", "P34_7",
	"P35_0", "P35_1", "P35_2", "P35_3", "P35_4", "P35_5", "P35_6", "P35_7",
	"P36_0", "P36_1", "P36_2", "P36_3", "P36_4", "P36_5", "P36_6", "P36_7",
	"P37_0", "P37_1", "P37_2", "P37_3", "P37_4", "P37_5", "P37_6", "P37_7",
	"P38_0", "P38_1", "P38_2", "P38_3", "P38_4", "P38_5", "P38_6", "P38_7",
	"P39_0", "P39_1", "P39_2", "P39_3", "P39_4", "P39_5", "P39_6", "P39_7",
	"P40_0", "P40_1", "P40_2", "P40_3", "P40_4", "P40_5", "P40_6", "P40_7",
	"P41_0", "P41_1", "P41_2", "P41_3", "P41_4", "P41_5", "P41_6", "P41_7",
	"P42_0", "P42_1", "P42_2", "P42_3", "P42_4", "P42_5", "P42_6", "P42_7",
	"P43_0", "P43_1", "P43_2", "P43_3", "P43_4", "P43_5", "P43_6", "P43_7",
	"P44_0", "P44_1", "P44_2", "P44_3", "P44_4", "P44_5", "P44_6", "P44_7",
	"P45_0", "P45_1", "P45_2", "P45_3", "P45_4", "P45_5", "P45_6", "P45_7",
	"P46_0", "P46_1", "P46_2", "P46_3", "P46_4", "P46_5", "P46_6", "P46_7",
	"P47_0", "P47_1", "P47_2", "P47_3", "P47_4", "P47_5", "P47_6", "P47_7",
	"P48_0", "P48_1", "P48_2", "P48_3", "P48_4", "P48_5", "P48_6", "P48_7",
};

static const char * const rzv2h_gpio_names[] = {
	"P0_0", "P0_1", "P0_2", "P0_3", "P0_4", "P0_5", "P0_6", "P0_7",
	"P1_0", "P1_1", "P1_2", "P1_3", "P1_4", "P1_5", "P1_6", "P1_7",
	"P2_0", "P2_1", "P2_2", "P2_3", "P2_4", "P2_5", "P2_6", "P2_7",
	"P3_0", "P3_1", "P3_2", "P3_3", "P3_4", "P3_5", "P3_6", "P3_7",
	"P4_0", "P4_1", "P4_2", "P4_3", "P4_4", "P4_5", "P4_6", "P4_7",
	"P5_0", "P5_1", "P5_2", "P5_3", "P5_4", "P5_5", "P5_6", "P5_7",
	"P6_0", "P6_1", "P6_2", "P6_3", "P6_4", "P6_5", "P6_6", "P6_7",
	"P7_0", "P7_1", "P7_2", "P7_3", "P7_4", "P7_5", "P7_6", "P7_7",
	"P8_0", "P8_1", "P8_2", "P8_3", "P8_4", "P8_5", "P8_6", "P8_7",
	"P9_0", "P9_1", "P9_2", "P9_3", "P9_4", "P9_5", "P9_6", "P9_7",
	"PA_0", "PA_1", "PA_2", "PA_3", "PA_4", "PA_5", "PA_6", "PA_7",
	"PB_0", "PB_1", "PB_2", "PB_3", "PB_4", "PB_5", "PB_6", "PB_7",
};

static const u32 rzg2l_gpio_configs[] = {
	RZG2L_GPIO_PORT_PACK(2, 0x10, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x11, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x12, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x13, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x14, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x15, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x16, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x17, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x18, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x19, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x1a, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x1b, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x1c, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x1d, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x1e, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x1f, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x20, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x21, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x22, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x23, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x24, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x25, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x26, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x27, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x28, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x29, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x2a, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x2b, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x2c, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(2, 0x2d, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x2e, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x2f, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x30, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x31, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x32, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x33, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x34, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(3, 0x35, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(2, 0x36, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x37, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x38, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x39, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(5, 0x3a, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x3b, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x3c, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x3d, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x3e, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x3f, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(5, 0x40, RZG2L_MPXED_PIN_FUNCS),
};

static const u32 r9a07g043_gpio_configs[] = {
	RZG2L_GPIO_PORT_PACK(4, 0x10, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(5, 0x11, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(4, 0x12, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(4, 0x13, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(6, 0x14, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH0)),
	RZG2L_GPIO_PORT_PACK(5, 0x15, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(5, 0x16, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(5, 0x17, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(5, 0x18, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(4, 0x19, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(5, 0x1a, RZG2L_MPXED_ETH_PIN_FUNCS(PIN_CFG_IO_VMC_ETH1)),
	RZG2L_GPIO_PORT_PACK(4, 0x1b, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x1c, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(5, 0x1d, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(3, 0x1e, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x1f, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x20, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(4, 0x21, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(6, 0x22, RZG2L_MPXED_PIN_FUNCS),
};

static const u32 r9a09g057_gpio_configs[] = {
	RZG2L_GPIO_PORT_PACK(8, 0x0, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(6, 0x1, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(2, 0x2, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x3, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x4, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x5, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x6, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x7, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x8, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0x9, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(8, 0xa, RZG2L_MPXED_PIN_FUNCS),
	RZG2L_GPIO_PORT_PACK(6, 0xb, RZG2L_MPXED_PIN_FUNCS),
};

static struct {
	struct rzg2l_dedicated_configs common[35];
	struct rzg2l_dedicated_configs rzg2l_pins[7];
} rzg2l_dedicated_pins = {
	.common = {
		{ "NMI", RZG2L_SINGLE_PIN_PACK(0x1, 0,
		 (PIN_CFG_FILONOFF | PIN_CFG_FILNUM | PIN_CFG_FILCLKSEL)) },
		{ "TMS/SWDIO", RZG2L_SINGLE_PIN_PACK(0x2, 0,
		 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN)) },
		{ "TDO", RZG2L_SINGLE_PIN_PACK(0x3, 0,
		 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN)) },
		{ "AUDIO_CLK1", RZG2L_SINGLE_PIN_PACK(0x4, 0, PIN_CFG_IEN) },
		{ "AUDIO_CLK2", RZG2L_SINGLE_PIN_PACK(0x4, 1, PIN_CFG_IEN) },
		{ "SD0_CLK", RZG2L_SINGLE_PIN_PACK(0x6, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_CMD", RZG2L_SINGLE_PIN_PACK(0x6, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_RST#", RZG2L_SINGLE_PIN_PACK(0x6, 2,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA0", RZG2L_SINGLE_PIN_PACK(0x7, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA1", RZG2L_SINGLE_PIN_PACK(0x7, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA2", RZG2L_SINGLE_PIN_PACK(0x7, 2,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA3", RZG2L_SINGLE_PIN_PACK(0x7, 3,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA4", RZG2L_SINGLE_PIN_PACK(0x7, 4,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA5", RZG2L_SINGLE_PIN_PACK(0x7, 5,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA6", RZG2L_SINGLE_PIN_PACK(0x7, 6,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD0_DATA7", RZG2L_SINGLE_PIN_PACK(0x7, 7,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD0)) },
		{ "SD1_CLK", RZG2L_SINGLE_PIN_PACK(0x8, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_SD1)) },
		{ "SD1_CMD", RZG2L_SINGLE_PIN_PACK(0x8, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD1)) },
		{ "SD1_DATA0", RZG2L_SINGLE_PIN_PACK(0x9, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD1)) },
		{ "SD1_DATA1", RZG2L_SINGLE_PIN_PACK(0x9, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD1)) },
		{ "SD1_DATA2", RZG2L_SINGLE_PIN_PACK(0x9, 2,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD1)) },
		{ "SD1_DATA3", RZG2L_SINGLE_PIN_PACK(0x9, 3,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_IO_VMC_SD1)) },
		{ "QSPI0_SPCLK", RZG2L_SINGLE_PIN_PACK(0xa, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI0_IO0", RZG2L_SINGLE_PIN_PACK(0xa, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI0_IO1", RZG2L_SINGLE_PIN_PACK(0xa, 2,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI0_IO2", RZG2L_SINGLE_PIN_PACK(0xa, 3,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI0_IO3", RZG2L_SINGLE_PIN_PACK(0xa, 4,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI0_SSL", RZG2L_SINGLE_PIN_PACK(0xa, 5,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI_RESET#", RZG2L_SINGLE_PIN_PACK(0xc, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI_WP#", RZG2L_SINGLE_PIN_PACK(0xc, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "WDTOVF_PERROUT#", RZG2L_SINGLE_PIN_PACK(0xd, 0, (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
		{ "RIIC0_SDA", RZG2L_SINGLE_PIN_PACK(0xe, 0, PIN_CFG_IEN) },
		{ "RIIC0_SCL", RZG2L_SINGLE_PIN_PACK(0xe, 1, PIN_CFG_IEN) },
		{ "RIIC1_SDA", RZG2L_SINGLE_PIN_PACK(0xe, 2, PIN_CFG_IEN) },
		{ "RIIC1_SCL", RZG2L_SINGLE_PIN_PACK(0xe, 3, PIN_CFG_IEN) },
	},
	.rzg2l_pins = {
		{ "QSPI_INT#", RZG2L_SINGLE_PIN_PACK(0xc, 2, (PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI1_SPCLK", RZG2L_SINGLE_PIN_PACK(0xb, 0,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI1_IO0", RZG2L_SINGLE_PIN_PACK(0xb, 1,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI1_IO1", RZG2L_SINGLE_PIN_PACK(0xb, 2,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI1_IO2", RZG2L_SINGLE_PIN_PACK(0xb, 3,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI1_IO3", RZG2L_SINGLE_PIN_PACK(0xb, 4,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR  | PIN_CFG_IO_VMC_QSPI)) },
		{ "QSPI1_SSL", RZG2L_SINGLE_PIN_PACK(0xb, 5,
		 (PIN_CFG_IOLH_B | PIN_CFG_SR | PIN_CFG_IO_VMC_QSPI)) },
	}
};

static struct rzg2l_dedicated_configs rzv2h_pins[] = {
	{ "NMI", RZG2L_SINGLE_PIN_PACK(0x1, 0,
	 (PIN_CFG_FILONOFF | PIN_CFG_FILNUM | PIN_CFG_FILCLKSEL)) },
	{ "TMS/SWDIO", RZG2L_SINGLE_PIN_PACK(0x3, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN)) },
	{ "TDO", RZG2L_SINGLE_PIN_PACK(0x3, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
	{ "WDTUDF_CA", RZG2L_SINGLE_PIN_PACK(0x5, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "WDTUDF_CM", RZG2L_SINGLE_PIN_PACK(0x5, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "SCIF_RXD", RZG2L_SINGLE_PIN_PACK(0x6, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "SCIF_TXD", RZG2L_SINGLE_PIN_PACK(0x6, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_CKP", RZG2L_SINGLE_PIN_PACK(0x7, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_CKN", RZG2L_SINGLE_PIN_PACK(0x7, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_CS0_N", RZG2L_SINGLE_PIN_PACK(0x7, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_DS", RZG2L_SINGLE_PIN_PACK(0x7, 3,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_RESET0_N", RZG2L_SINGLE_PIN_PACK(0x7, 4,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_RSTO0_N", RZG2L_SINGLE_PIN_PACK(0x7, 5,
	 (PIN_CFG_PUPD)) },
	{ "XSPI0_INT0_N", RZG2L_SINGLE_PIN_PACK(0x7, 6,
	 (PIN_CFG_PUPD)) },
	{ "XSPI0_ECS0_N", RZG2L_SINGLE_PIN_PACK(0x7, 7,
	 (PIN_CFG_PUPD)) },
	{ "XSPI0_IO0", RZG2L_SINGLE_PIN_PACK(0x8, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO1", RZG2L_SINGLE_PIN_PACK(0x8, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO2", RZG2L_SINGLE_PIN_PACK(0x8, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO3", RZG2L_SINGLE_PIN_PACK(0x8, 3,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO4", RZG2L_SINGLE_PIN_PACK(0x8, 4,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO5", RZG2L_SINGLE_PIN_PACK(0x8, 5,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO6", RZG2L_SINGLE_PIN_PACK(0x8, 6,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "XSPI0_IO7", RZG2L_SINGLE_PIN_PACK(0x8, 7,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "QSD0_CLK", RZG2L_SINGLE_PIN_PACK(0x9, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
	{ "QSD0_CMD", RZG2L_SINGLE_PIN_PACK(0x9, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
	{ "QSD0_RSTN", RZG2L_SINGLE_PIN_PACK(0x9, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT0", RZG2L_SINGLE_PIN_PACK(0xa, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT1", RZG2L_SINGLE_PIN_PACK(0xa, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT2", RZG2L_SINGLE_PIN_PACK(0xa, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT3", RZG2L_SINGLE_PIN_PACK(0xa, 3,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT4", RZG2L_SINGLE_PIN_PACK(0xa, 4,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT5", RZG2L_SINGLE_PIN_PACK(0xa, 5,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT6", RZG2L_SINGLE_PIN_PACK(0xa, 6,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD0_DAT7", RZG2L_SINGLE_PIN_PACK(0xa, 7,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD1_CLK", RZG2L_SINGLE_PIN_PACK(0xb, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
	{ "QSD1_CMD", RZG2L_SINGLE_PIN_PACK(0xb, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD1_DAT0", RZG2L_SINGLE_PIN_PACK(0xc, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD1_DAT1", RZG2L_SINGLE_PIN_PACK(0xc, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD1_DAT2", RZG2L_SINGLE_PIN_PACK(0xc, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "QSD1_DAT3", RZG2L_SINGLE_PIN_PACK(0xc, 3,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "PCIE0_RSTOUTB", RZG2L_SINGLE_PIN_PACK(0xe, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
	{ "PCIE1_RSTOUTB", RZG2L_SINGLE_PIN_PACK(0xe, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR)) },
	{ "ET0_MDIO", RZG2L_SINGLE_PIN_PACK(0xf, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_IEN | PIN_CFG_PUPD)) },
	{ "ET0_MDC", RZG2L_SINGLE_PIN_PACK(0xf, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_PHY_INTR", RZG2L_SINGLE_PIN_PACK(0xf, 4, 0)},
	{ "ET0_RX_CTL_RX_DV", RZG2L_SINGLE_PIN_PACK(0x10, 0,
	 (PIN_CFG_PUPD)) },
	{ "ET0_TX_CTL_TX_EN", RZG2L_SINGLE_PIN_PACK(0x10, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_TX_ER", RZG2L_SINGLE_PIN_PACK(0x10, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_RX_ER", RZG2L_SINGLE_PIN_PACK(0x10, 3,
	 (PIN_CFG_PUPD)) },
	{ "ET0_RXC_RX_CLK", RZG2L_SINGLE_PIN_PACK(0x10, 4,
	 (PIN_CFG_PUPD)) },
	{ "ET0_TXC_TX_CLK", RZG2L_SINGLE_PIN_PACK(0x10, 5,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_CRS", RZG2L_SINGLE_PIN_PACK(0x10, 6,
	 (PIN_CFG_PUPD)) },
	{ "ET0_COL", RZG2L_SINGLE_PIN_PACK(0x10, 7,
	 (PIN_CFG_PUPD)) },
	{ "ET0_TXD0", RZG2L_SINGLE_PIN_PACK(0x11, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_TXD1", RZG2L_SINGLE_PIN_PACK(0x11, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_TXD2", RZG2L_SINGLE_PIN_PACK(0x11, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_TXD3", RZG2L_SINGLE_PIN_PACK(0x11, 3,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET0_RXD0", RZG2L_SINGLE_PIN_PACK(0x11, 4,
	 (PIN_CFG_PUPD)) },
	{ "ET0_RXD1", RZG2L_SINGLE_PIN_PACK(0x11, 5,
	 (PIN_CFG_PUPD)) },
	{ "ET0_RXD2", RZG2L_SINGLE_PIN_PACK(0x11, 6,
	 (PIN_CFG_PUPD)) },
	{ "ET0_RXD3", RZG2L_SINGLE_PIN_PACK(0x11, 7,
	 (PIN_CFG_PUPD)) },
	{ "ET1_MDIO", RZG2L_SINGLE_PIN_PACK(0x12, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_SR | PIN_CFG_IEN| PIN_CFG_PUPD)) },
	{ "ET1_MDC", RZG2L_SINGLE_PIN_PACK(0x12, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_PHY_INTR", RZG2L_SINGLE_PIN_PACK(0x12, 4, 0)},
	{ "ET1_RX_CTL_RX_DV", RZG2L_SINGLE_PIN_PACK(0x13, 0,
	 (PIN_CFG_PUPD)) },
	{ "ET1_TX_CTL_TX_EN", RZG2L_SINGLE_PIN_PACK(0x13, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_TX_ER", RZG2L_SINGLE_PIN_PACK(0x13, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_RX_ER", RZG2L_SINGLE_PIN_PACK(0x13, 3,
	 (PIN_CFG_PUPD)) },
	{ "ET1_RXC_RX_CLK", RZG2L_SINGLE_PIN_PACK(0x13, 4,
	 (PIN_CFG_PUPD)) },
	{ "ET1_TXC_TX_CLK", RZG2L_SINGLE_PIN_PACK(0x13, 5,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_CRS", RZG2L_SINGLE_PIN_PACK(0x13, 6,
	 (PIN_CFG_PUPD)) },
	{ "ET1_COL", RZG2L_SINGLE_PIN_PACK(0x13, 7,
	 (PIN_CFG_PUPD)) },
	{ "ET1_TXD0", RZG2L_SINGLE_PIN_PACK(0x14, 0,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_TXD1", RZG2L_SINGLE_PIN_PACK(0x14, 1,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_TXD2", RZG2L_SINGLE_PIN_PACK(0x14, 2,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_TXD3", RZG2L_SINGLE_PIN_PACK(0x14, 3,
	 (PIN_CFG_IOLH_A | PIN_CFG_SR | PIN_CFG_PUPD)) },
	{ "ET1_RXD0", RZG2L_SINGLE_PIN_PACK(0x14, 4,
	 (PIN_CFG_PUPD)) },
	{ "ET1_RXD1", RZG2L_SINGLE_PIN_PACK(0x14, 5,
	 (PIN_CFG_PUPD)) },
	{ "ET1_RXD2", RZG2L_SINGLE_PIN_PACK(0x14, 6,
	 (PIN_CFG_PUPD)) },
	{ "ET1_RXD3", RZG2L_SINGLE_PIN_PACK(0x14, 7,
	 (PIN_CFG_PUPD)) },
};

static int rzg2l_gpio_register(struct rzg2l_pinctrl *pctrl)
{
	struct device_node *np = pctrl->dev->of_node;
	struct gpio_chip *chip = &pctrl->gpio_chip;
	struct irq_chip *irq_chip = &pctrl->irq_chip;
	const char *name = dev_name(pctrl->dev);
	struct of_phandle_args of_args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3, 0, &of_args);
	if (ret) {
		dev_err(pctrl->dev, "Unable to parse gpio-ranges\n");
		return ret;
	}

	if (of_args.args[0] != 0 || of_args.args[1] != 0 ||
	    of_args.args[2] != pctrl->data->n_port_pins) {
		dev_err(pctrl->dev, "gpio-ranges does not match selected SOC\n");
		return -EINVAL;
	}

	chip->names = pctrl->data->port_pins;
	chip->request = rzg2l_gpio_request;
	chip->free = rzg2l_gpio_free;
	chip->get_direction = rzg2l_gpio_get_direction;
	chip->direction_input = rzg2l_gpio_direction_input;
	chip->direction_output = rzg2l_gpio_direction_output;
	chip->get = rzg2l_gpio_get;
	chip->set = rzg2l_gpio_set;
	chip->label = name;
	chip->parent = pctrl->dev;
	chip->owner = THIS_MODULE;
	chip->base = -1;
	chip->ngpio = of_args.args[2];

	pctrl->gpio_range.id = 0;
	pctrl->gpio_range.pin_base = 0;
	pctrl->gpio_range.base = 0;
	pctrl->gpio_range.npins = chip->ngpio;
	pctrl->gpio_range.name = chip->label;
	pctrl->gpio_range.gc = chip;

	irq_chip->name = dev_name(pctrl->dev);
	irq_chip->irq_shutdown = rzg2l_gpio_irq_shutdown;
	irq_chip->irq_mask = rzg2l_gpio_irq_mask;
	irq_chip->irq_unmask = rzg2l_gpio_irq_unmask;
	irq_chip->irq_set_type = rzg2l_gpio_irq_set_type;
	irq_chip->irq_set_wake = rzg2l_gpio_irq_set_wake;
	irq_chip->flags = IRQCHIP_SET_TYPE_MASKED | IRQCHIP_MASK_ON_SUSPEND;

	ret = gpiochip_irqchip_add(chip, irq_chip, 0, handle_level_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		dev_err(pctrl->dev, "cannot add irqchip\n");
		return ret;
	}

	dev_dbg(pctrl->dev, "Registered interrupt controller\n");

	ret = devm_gpiochip_add_data(pctrl->dev, chip, pctrl);
	if (ret) {
		dev_err(pctrl->dev, "failed to add GPIO controller\n");
		return ret;
	}

	dev_dbg(pctrl->dev, "Registered gpio controller\n");

	return 0;
}

static int rzg2l_pinctrl_register(struct rzg2l_pinctrl *pctrl)
{
	struct pinctrl_pin_desc *pins;
	unsigned int i, j;
	u32 *pin_data;
	int ret;

	pctrl->desc.name = DRV_NAME;
	pctrl->desc.npins = pctrl->data->n_port_pins + pctrl->data->n_dedicated_pins;
	pctrl->desc.pctlops = &rzg2l_pinctrl_pctlops;
	pctrl->desc.pmxops = &rzg2l_pinctrl_pmxops;
	pctrl->desc.confops = &rzg2l_pinctrl_confops;
	pctrl->desc.owner = THIS_MODULE;

	pins = devm_kcalloc(pctrl->dev, pctrl->desc.npins, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	pin_data = devm_kcalloc(pctrl->dev, pctrl->desc.npins,
				sizeof(*pin_data), GFP_KERNEL);
	if (!pin_data)
		return -ENOMEM;

	pctrl->pins = pins;
	pctrl->desc.pins = pins;

	for (i = 0, j = 0; i < pctrl->data->n_port_pins; i++) {
		pins[i].number = i;
		pins[i].name = pctrl->data->port_pins[i];
		if (i && !(i % RZG2L_PINS_PER_PORT))
			j++;
		pin_data[i] = pctrl->data->port_pin_configs[j];
		pins[i].drv_data = &pin_data[i];
	}

	for (i = 0; i < pctrl->data->n_dedicated_pins; i++) {
		unsigned int index = pctrl->data->n_port_pins + i;

		pins[index].number = index;
		pins[index].name = pctrl->data->dedicated_pins[i].name;
		pin_data[index] = pctrl->data->dedicated_pins[i].config;
		pins[index].drv_data = &pin_data[index];
	}

	ret = devm_pinctrl_register_and_init(pctrl->dev, &pctrl->desc, pctrl,
					     &pctrl->pctl);
	if (ret) {
		dev_err(pctrl->dev, "pinctrl registration failed\n");
		return ret;
	}

	ret = pinctrl_enable(pctrl->pctl);
	if (ret) {
		dev_err(pctrl->dev, "pinctrl enable failed\n");
		return ret;
	}

	ret = rzg2l_gpio_register(pctrl);
	if (ret) {
		dev_err(pctrl->dev, "failed to add GPIO chip: %i\n", ret);
		return ret;
	}

	return 0;
}

static void rzg2l_pinctrl_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int rzg2l_pinctrl_prepare_mem_suspend(struct rzg2l_pinctrl *pctrl)
{
	struct device *dev = pctrl->dev;
	struct rzg2l_backup_data *backup;
	u32 nports = pctrl->data->n_ports;

	backup = devm_kzalloc(dev, sizeof(*backup), GFP_KERNEL);
	if (!backup)
		goto err;

	backup->p_reg = devm_kzalloc(dev, nports * sizeof(*(backup->p_reg)), GFP_KERNEL);
	if (!backup->p_reg)
		goto err;

	backup->pm_reg = devm_kzalloc(dev, nports * sizeof(*(backup->pm_reg)), GFP_KERNEL);
	if (!backup->pm_reg)
		goto err;

	backup->pmc_reg = devm_kzalloc(dev, nports * sizeof(*(backup->pmc_reg)), GFP_KERNEL);
	if (!backup->pmc_reg)
		goto err;

	backup->pfc_reg  = devm_kzalloc(dev, nports * sizeof(*(backup->pfc_reg)), GFP_KERNEL);
	if (!backup->pfc_reg)
		goto err;

	backup->isel_reg = devm_kzalloc(dev, nports * sizeof(*(backup->isel_reg)), GFP_KERNEL);
	if (!backup->isel_reg)
		goto err;

	pctrl->backup = backup;
	return 0;
err:
	return -ENOMEM;
}

static int rzg2l_pinctrl_probe(struct platform_device *pdev)
{
	struct rzg2l_pinctrl *pctrl;
	struct resource *res, *irq;
	int i, ret;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->dev = &pdev->dev;

	pctrl->data = of_device_get_match_data(&pdev->dev);
	if (!pctrl->data)
		return -EINVAL;

	pctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pctrl->base))
		return PTR_ERR(pctrl->base);

	ret = rzg2l_pinctrl_prepare_mem_suspend(pctrl);
	if (ret < 0)
		return ret;

	pctrl->clk = devm_clk_get_optional(pctrl->dev, NULL);
	if (IS_ERR(pctrl->clk)) {
		ret = PTR_ERR(pctrl->clk);
		dev_err(pctrl->dev, "failed to get GPIO clk : %i\n", ret);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "missing IO resource\n");
		return -ENXIO;
	}

	pctrl->base_tint = ioremap(res->start, resource_size(res));
	if (IS_ERR(pctrl->base_tint))
		return PTR_ERR(pctrl->base_tint);

	if (pctrl->data->irq_mask) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!res) {
			dev_err(&pdev->dev, "missing IO resource\n");
			return -ENXIO;
		}

		pctrl->tint_irq_mask = ioremap(res->start, resource_size(res));
		if (IS_ERR(pctrl->tint_irq_mask))
			return PTR_ERR(pctrl->tint_irq_mask);
	}

	for (i = 0; i < TINT_MAX; i++) {
		char *irqstr[TINT_MAX];

		irq = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!irq) {
			dev_err(pctrl->dev, "missing IRQ\n");
			return -EINVAL;
		};

		if (i == 0)
			pctrl->irq_start = irq->start;

		irqstr[i] = kasprintf(GFP_KERNEL, "tint%d", i);

		if (devm_request_irq(pctrl->dev, irq->start,
				     rzg2l_pinctrl_irq_handler, IRQF_SHARED,
				     irqstr[i], pctrl)) {
			dev_err(pctrl->dev, "failed to request IRQ\n");
			return -ENOENT;
		}
	}

	spin_lock_init(&pctrl->lock);
	mutex_init(&pctrl->mutex);

	platform_set_drvdata(pdev, pctrl);

	if (pctrl->clk) {
		ret = clk_prepare_enable(pctrl->clk);
		if (ret) {
			dev_err(pctrl->dev, "failed to enable GPIO clk: %i\n", ret);
			return ret;
		}

		ret = devm_add_action_or_reset(&pdev->dev, rzg2l_pinctrl_clk_disable,
					       pctrl->clk);
		if (ret) {
			dev_err(pctrl->dev,
				"failed to register GPIO clk disable action, %i\n",
				ret);
			return ret;
		}
	};

	ret = rzg2l_pinctrl_register(pctrl);
	if (ret)
		return ret;

	/* Mask all TINT interrupts mask if TMSK is existed */
	if (pctrl->data->irq_mask)
		writel(0xffffffff, pctrl->tint_irq_mask + TMSK);

	dev_info(pctrl->dev, "%s support registered\n", DRV_NAME);
	return 0;
}

static int rzg2l_pinctrl_remove(struct platform_device *pdev)
{
	struct rzg2l_pinctrl *pctrl = platform_get_drvdata(pdev);

	gpiochip_remove(&pctrl->gpio_chip);

	iounmap(pctrl->base_tint);

	if (pctrl->data->irq_mask)
		iounmap(pctrl->tint_irq_mask);

	return 0;
}

static void rzv2h_pinctrl_backup(struct rzg2l_pinctrl *pctrl)
{
	struct rzg2l_backup_data *backup = pctrl->backup;

	backup->oen_reg = readb(pctrl->base + pctrl->data->oen_reg_offset);
}

static void rzg2l_pinctrl_backup(struct rzg2l_pinctrl *pctrl)
{
	const u32 *port_pin_configs = pctrl->data->port_pin_configs;
	struct rzg2l_backup_data *backup;
	u32 pwpr = pctrl->data->pwpr;
	u32 port_offset;
	int i;

	backup = (struct rzg2l_backup_data *)pctrl->backup;

	for (i = 0; i < pctrl->data->n_ports; i++) {
		port_offset = RZG2L_GPIO_PORT_GET_INDEX(port_pin_configs[i]) + 0x10;

		backup->p_reg[i] = readb(pctrl->base + P(port_offset));
		backup->pm_reg[i] = readw(pctrl->base + PM(port_offset));
		backup->pmc_reg[i] = readb(pctrl->base + PMC(port_offset));
		backup->pfc_reg[i] = readl(pctrl->base + PFC(port_offset));
		backup->isel_reg[i] = readl(pctrl->base + ISEL(port_offset));
	}

	if (pwpr == PWPR_RZ_V2H)
		rzv2h_pinctrl_backup(pctrl);
}

static void rzv2h_pinctrl_restore(struct rzg2l_pinctrl *pctrl)
{
	struct rzg2l_backup_data *backup = pctrl->backup;
	u32 pwpr = pctrl->data->pwpr;

	/* Enable writing to the OEN register */
	writel(readl(pctrl->base + pwpr) | PWPR_REGWE_B,
					pctrl->base + pwpr);

	writeb(backup->oen_reg, pctrl->base + pctrl->data->oen_reg_offset);

	/* Disable writing to the OEN register */
	writel(readl(pctrl->base + pwpr) & ~PWPR_REGWE_B,
					pctrl->base + pwpr);
}

static void rzg2l_pinctrl_restore(struct rzg2l_pinctrl *pctrl)
{
	const u32 *port_pin_configs = pctrl->data->port_pin_configs;
	struct rzg2l_backup_data *backup;
	u32 port_offset;
	u32 pwpr = pctrl->data->pwpr;
	int i;

	backup = (struct rzg2l_backup_data *)pctrl->backup;

	/* set the PWPR register to allow PFC register to write */
	if (pwpr == PWPR) {
		writel(0x0, pctrl->base + pwpr);
		writel(PWPR_PFCWE, pctrl->base + pwpr);
	} else if (pwpr == PWPR_RZ_V2H) {
		writel(readl(pctrl->base + pwpr) | PWPR_REGWE_A,
					pctrl->base + pwpr);
	}

	/* Backup common registers */
	for (i = 0; i < pctrl->data->n_ports; i++) {
		port_offset = RZG2L_GPIO_PORT_GET_INDEX(port_pin_configs[i]) + 0x10;

		writeb(backup->p_reg[i], pctrl->base + P(port_offset));
		writew(backup->pm_reg[i], pctrl->base + PM(port_offset));
		writeb(backup->pmc_reg[i], pctrl->base + PMC(port_offset));
		writel(backup->pfc_reg[i], pctrl->base + PFC(port_offset));
		writel(backup->isel_reg[i], pctrl->base + ISEL(port_offset));
	}

	/* set the PWPR register to de-allow PFC register to write */
	if (pwpr == PWPR) {
		writel(0x0, pctrl->base + PWPR);
		writel(PWPR_B0WI, pctrl->base + PWPR);
	} else if (pwpr == PWPR_RZ_V2H) {
		writel(readl(pctrl->base + pwpr) & ~PWPR_REGWE_A,
				pctrl->base + pwpr);
	}

	/* Restore platform-specific registers */
	if (pwpr == PWPR_RZ_V2H)
		rzv2h_pinctrl_restore(pctrl);
}

static int __maybe_unused rzg2l_pinctrl_suspend_noirq(struct device *dev)
{
	struct rzg2l_pinctrl *pctrl = dev_get_drvdata(dev);

	rzg2l_pinctrl_backup(pctrl);

	if (atomic_read(&pctrl->wakeup_path))
		device_set_wakeup_path(dev);

	return 0;
}

static int __maybe_unused rzg2l_pinctrl_resume_noirq(struct device *dev)
{
	struct rzg2l_pinctrl *pctrl = dev_get_drvdata(dev);

	rzg2l_pinctrl_restore(pctrl);

	return 0;
}

static const struct dev_pm_ops rzg2l_pinctrl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rzg2l_pinctrl_suspend_noirq, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(rzg2l_pinctrl_suspend_noirq, rzg2l_pinctrl_resume_noirq)
};

static struct rzg2l_pinctrl_data r9a07g043_data = {
	.port_pins = rzg2l_gpio_names,
	.port_pin_configs = r9a07g043_gpio_configs,
	.dedicated_pins = rzg2l_dedicated_pins.common,
	.n_port_pins = ARRAY_SIZE(r9a07g043_gpio_configs) * RZG2L_PINS_PER_PORT,
	.n_dedicated_pins = ARRAY_SIZE(rzg2l_dedicated_pins.common),
	.pin_info = rzg2ul_pin_info,
	.ngpioints = ARRAY_SIZE(rzg2ul_pin_info),
	.irq_mask = false,
	.extended_reg_offset = 0,
	.have_clrirq_reg = false,
	.pwpr = PWPR,
};

static struct rzg2l_pinctrl_data r9a07g043f_data = {
	.port_pins = rzg2l_gpio_names,
	.port_pin_configs = r9a07g043_gpio_configs,
	.dedicated_pins = rzg2l_dedicated_pins.common,
	.n_port_pins = ARRAY_SIZE(r9a07g043_gpio_configs) * RZG2L_PINS_PER_PORT,
	.n_dedicated_pins = ARRAY_SIZE(rzg2l_dedicated_pins.common),
	.pin_info = rzg2ul_pin_info,
	.ngpioints = ARRAY_SIZE(rzg2ul_pin_info),
	.irq_mask = true,
	.extended_reg_offset = 0,
	.have_clrirq_reg = false,
	.pwpr = PWPR,
};

static struct rzg2l_pinctrl_data r9a07g044_data = {
	.port_pins = rzg2l_gpio_names,
	.port_pin_configs = rzg2l_gpio_configs,
	.dedicated_pins = rzg2l_dedicated_pins.common,
	.n_port_pins = ARRAY_SIZE(rzg2l_gpio_names),
	.n_dedicated_pins = ARRAY_SIZE(rzg2l_dedicated_pins.common) +
		ARRAY_SIZE(rzg2l_dedicated_pins.rzg2l_pins),
	.pin_info = rzg2l_pin_info,
	.ngpioints = ARRAY_SIZE(rzg2l_pin_info),
	.irq_mask = false,
	.extended_reg_offset = 0,
	.have_clrirq_reg = false,
	.pwpr = PWPR,
};

static struct rzg2l_pinctrl_data r9a09g057_data = {
	.port_pins = rzv2h_gpio_names,
	.port_pin_configs = r9a09g057_gpio_configs,
	.dedicated_pins = rzv2h_pins,
	.n_port_pins = ARRAY_SIZE(rzv2h_gpio_names),
	.n_dedicated_pins = ARRAY_SIZE(rzv2h_pins),
	.pin_info = rzv2h_pin_info,
	.ngpioints = ARRAY_SIZE(rzv2h_pin_info),
	.irq_mask = false,
	.extended_reg_offset = 0x10,
	.have_clrirq_reg = true,
	.pwpr = PWPR_RZ_V2H,
	.oen_reg_offset = 0x3C40,
	.n_ports = ARRAY_SIZE(r9a09g057_gpio_configs),
};

static const struct of_device_id rzg2l_pinctrl_of_table[] = {
	{
		.compatible = "renesas,r9a07g043-pinctrl",
		.data = &r9a07g043_data,
	},
	{
		.compatible = "renesas,r9a07g043f-pinctrl",
		.data = &r9a07g043f_data,
	},
	{
		.compatible = "renesas,r9a07g044-pinctrl",
		.data = &r9a07g044_data,
	},
	{
		.compatible = "renesas,r9a09g057-pinctrl",
		.data = &r9a09g057_data,
	},
	{ /* sentinel */ }
};

static struct platform_driver rzg2l_pinctrl_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(rzg2l_pinctrl_of_table),
		.pm = &rzg2l_pinctrl_pm_ops,
	},
	.probe = rzg2l_pinctrl_probe,
	.remove = rzg2l_pinctrl_remove,
};

static int __init rzg2l_pinctrl_init(void)
{
	return platform_driver_register(&rzg2l_pinctrl_driver);
}
core_initcall(rzg2l_pinctrl_init);

MODULE_AUTHOR("Lad Prabhakar <prabhakar.mahadev-lad.rj@bp.renesas.com>");
MODULE_DESCRIPTION("Pin and gpio controller driver for RZ/G2L family");
MODULE_LICENSE("GPL v2");
