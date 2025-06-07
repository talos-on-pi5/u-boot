// SPDX-License-Identifier: GPL-2.0+
/*
 * Clock driver for RP1 PCIe multifunction chip.
 *
 * Copyright (C) 2024 EPAM Systems
 *
 * Derived from linux clk-rp1 driver
 *   Copyright (C) 2023 Raspberry Pi Ltd.
 */

#include <clk.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/math64.h>

#include <dt-bindings/clk/rp1.h>

#define GPCLK_OE_CTRL			0x00000

#define CLK_ETH_TSU_CTRL		0x00134
#define CLK_ETH_TSU_DIV_INT		0x00138
#define CLK_ETH_TSU_SEL			0x00140

#define FC_NUM(idx, off)		((idx) * 32 + (off))

#define DIV_INT_8BIT_MAX		0x000000ffu /* max divide for most clocks */

/* Clock fields for all clocks */
#define CLK_CTRL_ENABLE			BIT(11)
#define CLK_DIV_FRAC_BITS		16

#define KHz				1000
#define MHz				(KHz * KHz)

#define MAX_CLK_PARENTS	16
#define DIV_U64_NEAREST(a, b) div_u64(((a) + ((b) >> 1)), (b))

struct rp1_clockman {
	struct udevice *dev;
	void __iomem *regs;
	spinlock_t regs_lock; /* spinlock for all clocks */
};

struct rp1_pll_core_data {
	const char *name;
	u32 cs_reg;
	u32 pwr_reg;
	u32 fbdiv_int_reg;
	u32 fbdiv_frac_reg;
	unsigned long flags;
	u32 fc0_src;
};

struct rp1_pll_data {
	const char *name;
	const char *source_pll;
	u32 ctrl_reg;
	unsigned long flags;
	u32 fc0_src;
};

struct rp1_pll_ph_data {
	const char *name;
	const char *source_pll;
	unsigned int phase;
	unsigned int fixed_divider;
	u32 ph_reg;
	unsigned long flags;
	u32 fc0_src;
};

struct rp1_pll_divider_data {
	const char *name;
	const char *source_pll;
	u32 sec_reg;
	unsigned long flags;
	u32 fc0_src;
};

struct rp1_clock_data {
	const char *name;
	const char *const parents[MAX_CLK_PARENTS];
	int num_std_parents;
	int num_aux_parents;
	unsigned long flags;
	u32 oe_mask;
	u32 clk_src_mask;
	u32 ctrl_reg;
	u32 div_int_reg;
	u32 div_frac_reg;
	u32 sel_reg;
	u32 div_int_max;
	unsigned long max_freq;
	u32 fc0_src;
};

struct rp1_pll_core {
	struct clk hw;
	struct rp1_clockman *clockman;
	const struct rp1_pll_core_data *data;
	unsigned long cached_rate;
};

struct rp1_pll {
	struct clk hw;
	struct clk_divider div;
	struct rp1_clockman *clockman;
	const struct rp1_pll_data *data;
	unsigned long cached_rate;
};

struct rp1_pll_ph {
	struct clk hw;
	struct rp1_clockman *clockman;
	const struct rp1_pll_ph_data *data;
};

struct rp1_clock {
	struct clk hw;
	struct rp1_clockman *clockman;
	const struct rp1_clock_data *data;
	unsigned long cached_rate;
};

struct rp1_clk_change {
	struct clk *hw;
	unsigned long new_rate;
};

struct rp1_clk_change rp1_clk_chg_tree[3];

static inline
void clockman_write(struct rp1_clockman *clockman, u32 reg, u32 val)
{
	writel(val, clockman->regs + reg);
}

static inline u32 clockman_read(struct rp1_clockman *clockman, u32 reg)
{
	return readl(clockman->regs + reg);
}

static struct rp1_clock_data rp1_data[] = {
[RP1_CLK_ETH_TSU] = {
			.name = "clk_eth_tsu",
			.parents = {"xosc",
				    "pll_video_sec",
				    "clksrc_gp0",
				    "clksrc_gp1",
				    "clksrc_gp2",
				    "clksrc_gp3",
				    "clksrc_gp4",
				    "clksrc_gp5"},
			.num_std_parents = 0,
			.num_aux_parents = 8,
			.ctrl_reg = CLK_ETH_TSU_CTRL,
			.div_int_reg = CLK_ETH_TSU_DIV_INT,
			.div_frac_reg = 0,
			.sel_reg = CLK_ETH_TSU_SEL,
			.div_int_max = DIV_INT_8BIT_MAX,
			.max_freq = 50 * MHz,
			.fc0_src = FC_NUM(5, 7),
	},
};

static u32 rp1_clock_choose_div(unsigned long rate, unsigned long parent_rate,
				const struct rp1_clock_data *data)
{
	u64 div;

	/*
	 * Due to earlier rounding, calculated parent_rate may differ from
	 * expected value. Don't fail on a small discrepancy near unity divide.
	 */
	if (!rate || rate > parent_rate + (parent_rate >> CLK_DIV_FRAC_BITS))
		return 0;

	/*
	 * Always express div in fixed-point format for fractional division;
	 * If no fractional divider is present, the fraction part will be zero.
	 */
	if (data->div_frac_reg) {
		div = (u64)parent_rate << CLK_DIV_FRAC_BITS;
		div = DIV_U64_NEAREST(div, rate);
	} else {
		div = DIV_U64_NEAREST(parent_rate, rate);
		div <<= CLK_DIV_FRAC_BITS;
	}

	div = clamp(div,
		    1ull << CLK_DIV_FRAC_BITS,
		    (u64)data->div_int_max << CLK_DIV_FRAC_BITS);

	return div;
}

static ulong rp1_clock_set_rate(struct clk *hw, unsigned long rate)
{
	struct rp1_clockman *clockman = dev_get_priv(hw->dev);
	const struct rp1_clock_data *data = &rp1_data[hw->id];
	u32 div = rp1_clock_choose_div(rate, 0x2faf080, data);

	if (hw->id != RP1_CLK_ETH_TSU)
		return 0;

	WARN(rate > 4000000000ll, "rate is -ve (%d)\n", (int)rate);

	if (WARN(!div,
		 "clk divider calculated as 0! (%s, rate %ld, parent rate %d)\n",
		 data->name, rate, 0x2faf080))
		div = 1 << CLK_DIV_FRAC_BITS;

	spin_lock(&clockman->regs_lock);

	clockman_write(clockman, data->div_int_reg, div >> CLK_DIV_FRAC_BITS);
	if (data->div_frac_reg)
		clockman_write(clockman, data->div_frac_reg, div << (32 - CLK_DIV_FRAC_BITS));

	spin_unlock(&clockman->regs_lock);

	return 0;
}

static int rp1_clock_on(struct clk *hw)
{
	const struct rp1_clock_data *data = &rp1_data[hw->id];
	struct rp1_clockman *clockman = dev_get_priv(hw->dev);

	if (hw->id != RP1_CLK_ETH_TSU)
		return 0;

	spin_lock(&clockman->regs_lock);
	clockman_write(clockman, data->ctrl_reg,
		       clockman_read(clockman, data->ctrl_reg) | CLK_CTRL_ENABLE);
	/* If this is a GPCLK, turn on the output-enable */
	if (data->oe_mask)
		clockman_write(clockman, GPCLK_OE_CTRL,
			       clockman_read(clockman, GPCLK_OE_CTRL) | data->oe_mask);

	spin_unlock(&clockman->regs_lock);

	return 0;
}

static int rp1_clk_probe(struct udevice *dev)
{
	struct rp1_clockman *clockman = dev_get_priv(dev);

	spin_lock_init(&clockman->regs_lock);

	clockman->regs = dev_remap_addr(dev);
	if (!clockman->regs)
		return -EINVAL;

	return 0;
}

static const struct udevice_id rp1_clk_of_match[] = {
	{ .compatible = "raspberrypi,rp1-clocks" },
	{ /* sentinel */ }
};

static struct clk_ops rp1_clk_ops = {
	.set_rate = rp1_clock_set_rate,
	.enable = rp1_clock_on,
};

U_BOOT_DRIVER(clk_rp1) = {
	.name = "rp1-clk",
	.id = UCLASS_CLK,
	.of_match = rp1_clk_of_match,
	.probe = rp1_clk_probe,
	.ops = &rp1_clk_ops,
	.priv_auto	= sizeof(struct rp1_clockman),
	.flags = CLK_IGNORE_UNUSED,
};
