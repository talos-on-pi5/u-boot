// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 EPAM Systems
 *
 * Derived from linux/drivers/pinctrl/pinctl-rp1.c
 * Copyright (C) 2023 Raspberry Pi Ltd.
 */

#include <dm.h>
#include <errno.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitops.h>

#define RP1_NUM_GPIOS	54
#define RP1_NUM_BANKS	3

#define RP1_RW_OFFSET			0x0000
#define RP1_XOR_OFFSET			0x1000
#define RP1_SET_OFFSET			0x2000
#define RP1_CLR_OFFSET			0x3000

#define RP1_GPIO_STATUS			0x0000
#define RP1_GPIO_CTRL			0x0004

#define RP1_GPIO_PCIE_INTE		0x011c
#define RP1_GPIO_PCIE_INTS		0x0124

#define RP1_GPIO_EVENTS_SHIFT_RAW	20
#define RP1_GPIO_STATUS_FALLING		BIT(20)
#define RP1_GPIO_STATUS_RISING		BIT(21)
#define RP1_GPIO_STATUS_LOW		BIT(22)
#define RP1_GPIO_STATUS_HIGH		BIT(23)

#define RP1_GPIO_EVENTS_SHIFT_FILTERED	24
#define RP1_GPIO_STATUS_F_FALLING	BIT(24)
#define RP1_GPIO_STATUS_F_RISING	BIT(25)
#define RP1_GPIO_STATUS_F_LOW		BIT(26)
#define RP1_GPIO_STATUS_F_HIGH		BIT(27)

#define RP1_GPIO_CTRL_FUNCSEL_LSB	0
#define RP1_GPIO_CTRL_FUNCSEL_MASK	0x0000001f
#define RP1_GPIO_CTRL_OUTOVER_LSB	12
#define RP1_GPIO_CTRL_OUTOVER_MASK	0x00003000
#define RP1_GPIO_CTRL_OEOVER_LSB	14
#define RP1_GPIO_CTRL_OEOVER_MASK	0x0000c000
#define RP1_GPIO_CTRL_INOVER_LSB	16
#define RP1_GPIO_CTRL_INOVER_MASK	0x00030000
#define RP1_GPIO_CTRL_IRQEN_FALLING	BIT(20)
#define RP1_GPIO_CTRL_IRQEN_RISING	BIT(21)
#define RP1_GPIO_CTRL_IRQEN_LOW		BIT(22)
#define RP1_GPIO_CTRL_IRQEN_HIGH	BIT(23)
#define RP1_GPIO_CTRL_IRQEN_F_FALLING	BIT(24)
#define RP1_GPIO_CTRL_IRQEN_F_RISING	BIT(25)
#define RP1_GPIO_CTRL_IRQEN_F_LOW	BIT(26)
#define RP1_GPIO_CTRL_IRQEN_F_HIGH	BIT(27)
#define RP1_GPIO_CTRL_IRQRESET		BIT(28)
#define RP1_GPIO_CTRL_IRQOVER_LSB	30
#define RP1_GPIO_CTRL_IRQOVER_MASK	0xc0000000

#define RP1_PUD_OFF			0
#define RP1_PUD_DOWN			1
#define RP1_PUD_UP			2

#define RP1_FSEL_COUNT			9

#define RP1_FSEL_ALT0			0x00
#define RP1_FSEL_GPIO			0x05
#define RP1_FSEL_NONE			0x09
#define RP1_FSEL_NONE_HW		0x1f

#define RP1_DIR_OUTPUT			0
#define RP1_DIR_INPUT			1

#define RP1_OUTOVER_PERI		0
#define RP1_OUTOVER_INVPERI		1
#define RP1_OUTOVER_LOW			2
#define RP1_OUTOVER_HIGH		3

#define RP1_OEOVER_PERI			0
#define RP1_OEOVER_INVPERI		1
#define RP1_OEOVER_DISABLE		2
#define RP1_OEOVER_ENABLE		3

#define RP1_INOVER_PERI			0
#define RP1_INOVER_INVPERI		1
#define RP1_INOVER_LOW			2
#define RP1_INOVER_HIGH			3

#define RP1_RIO_OUT			0x00
#define RP1_RIO_OE			0x04
#define RP1_RIO_IN			0x08

#define RP1_PAD_SLEWFAST_MASK		0x00000001
#define RP1_PAD_SLEWFAST_LSB		0
#define RP1_PAD_SCHMITT_MASK		0x00000002
#define RP1_PAD_SCHMITT_LSB		1
#define RP1_PAD_PULL_MASK		0x0000000c
#define RP1_PAD_PULL_LSB		2
#define RP1_PAD_DRIVE_MASK		0x00000030
#define RP1_PAD_DRIVE_LSB		4
#define RP1_PAD_IN_ENABLE_MASK		0x00000040
#define RP1_PAD_IN_ENABLE_LSB		6
#define RP1_PAD_OUT_DISABLE_MASK	0x00000080
#define RP1_PAD_OUT_DISABLE_LSB		7

#define RP1_PAD_DRIVE_2MA		0x00000000
#define RP1_PAD_DRIVE_4MA		0x00000010
#define RP1_PAD_DRIVE_8MA		0x00000020
#define RP1_PAD_DRIVE_12MA		0x00000030

#define FLD_GET(r, f) (((r) & (f ## _MASK)) >> (f ## _LSB))
#define FLD_SET(r, f, v) r = (((r) & ~(f ## _MASK)) | ((v) << (f ## _LSB)))

struct rp1_iobank_desc {
	int min_gpio;
	int num_gpios;
	int gpio_offset;
	int inte_offset;
	int ints_offset;
	int rio_offset;
	int pads_offset;
};

const struct rp1_iobank_desc rp1_iobanks[RP1_NUM_BANKS] = {
	/*         gpio   inte    ints     rio    pads */
	{  0, 28, 0x0000, 0x011c, 0x0124, 0x0000, 0x0004 },
	{ 28,  6, 0x4000, 0x411c, 0x4124, 0x4000, 0x4004 },
	{ 34, 20, 0x8000, 0x811c, 0x8124, 0x8000, 0x8004 },
};

struct rp1_pin_info {
	u8 num;
	u8 bank;
	u8 offset;
	u8 fsel;

	void __iomem *gpio;
	void __iomem *rio;
	void __iomem *inte;
	void __iomem *ints;
	void __iomem *pad;
};

struct rp1_gpio_priv {
	void __iomem *gpio_base;
	void __iomem *rio_base;
	void __iomem *pads_base;

	struct rp1_pin_info pins[RP1_NUM_GPIOS];
};

static struct rp1_pin_info *rp1_get_pin(struct udevice *dev,
					unsigned int offset)
{
	struct rp1_gpio_priv *priv = dev_get_priv(dev);

	if (priv && offset < RP1_NUM_GPIOS)
		return &priv->pins[offset];

	return NULL;
}

static void rp1_pad_update(struct rp1_pin_info *pin, u32 clr, u32 set)
{
	u32 padctrl = readl(pin->pad);

	padctrl &= ~clr;
	padctrl |= set;

	writel(padctrl, pin->pad);
}

static void rp1_input_enable(struct rp1_pin_info *pin, int value)
{
	rp1_pad_update(pin, RP1_PAD_IN_ENABLE_MASK,
		       value ? RP1_PAD_IN_ENABLE_MASK : 0);
}

static void rp1_output_enable(struct rp1_pin_info *pin, int value)
{
	rp1_pad_update(pin, RP1_PAD_OUT_DISABLE_MASK,
		       value ? 0 : RP1_PAD_OUT_DISABLE_MASK);
}

static u32 rp1_get_fsel(struct rp1_pin_info *pin)
{
	u32 ctrl = readl(pin->gpio + RP1_GPIO_CTRL);
	u32 oeover = FLD_GET(ctrl, RP1_GPIO_CTRL_OEOVER);
	u32 fsel = FLD_GET(ctrl, RP1_GPIO_CTRL_FUNCSEL);

	if (oeover != RP1_OEOVER_PERI || fsel >= RP1_FSEL_COUNT)
		fsel = RP1_FSEL_NONE;

	return fsel;
}

static void rp1_set_fsel(struct rp1_pin_info *pin, u32 fsel)
{
	u32 ctrl = readl(pin->gpio + RP1_GPIO_CTRL);

	if (fsel >= RP1_FSEL_COUNT)
		fsel = RP1_FSEL_NONE_HW;

	rp1_input_enable(pin, 1);
	rp1_output_enable(pin, 1);

	if (fsel == RP1_FSEL_NONE) {
		FLD_SET(ctrl, RP1_GPIO_CTRL_OEOVER, RP1_OEOVER_DISABLE);
	} else {
		FLD_SET(ctrl, RP1_GPIO_CTRL_OUTOVER, RP1_OUTOVER_PERI);
		FLD_SET(ctrl, RP1_GPIO_CTRL_OEOVER, RP1_OEOVER_PERI);
	}
	FLD_SET(ctrl, RP1_GPIO_CTRL_FUNCSEL, fsel);

	writel(ctrl, pin->gpio + RP1_GPIO_CTRL);
}

static int rp1_get_dir(struct rp1_pin_info *pin)
{
	return !(readl(pin->rio + RP1_RIO_OE) & (1 << pin->offset)) ?
		RP1_DIR_INPUT : RP1_DIR_OUTPUT;
}

static void rp1_set_dir(struct rp1_pin_info *pin, bool is_input)
{
	int offset = is_input ? RP1_CLR_OFFSET : RP1_SET_OFFSET;

	writel(1 << pin->offset, pin->rio + RP1_RIO_OE + offset);
}

static int rp1_get_value(struct rp1_pin_info *pin)
{
	return !!(readl(pin->rio + RP1_RIO_IN) & (1 << pin->offset));
}

static void rp1_set_value(struct rp1_pin_info *pin, int value)
{
	/* Assume the pin is already an output */
	writel(1 << pin->offset,
	       pin->rio + RP1_RIO_OUT + (value ? RP1_SET_OFFSET : RP1_CLR_OFFSET));
}

static int rp1_gpio_get(struct udevice *dev, unsigned int offset)
{
	struct rp1_pin_info *pin = rp1_get_pin(dev, offset);
	int ret;

	if (!pin)
		return -EINVAL;

	ret = rp1_get_value(pin);
	return ret;
}

static int rp1_gpio_set(struct udevice *dev, unsigned int offset, int value)
{
	struct rp1_pin_info *pin = rp1_get_pin(dev, offset);

	if (pin)
		rp1_set_value(pin, value);

	return 0;
}

static int rp1_gpio_direction_input(struct udevice *dev, unsigned int offset)
{
	struct rp1_pin_info *pin = rp1_get_pin(dev, offset);

	if (!pin)
		return -EINVAL;

	rp1_set_dir(pin, RP1_DIR_INPUT);
	rp1_set_fsel(pin, RP1_FSEL_GPIO);
	return 0;
}

static int rp1_gpio_direction_output(struct udevice *dev, unsigned int offset, int value)
{
	struct rp1_pin_info *pin = rp1_get_pin(dev, offset);

	if (!pin)
		return -EINVAL;

	rp1_set_value(pin, value);
	rp1_set_dir(pin, RP1_DIR_OUTPUT);
	rp1_set_fsel(pin, RP1_FSEL_GPIO);
	return 0;
}

static int rp1_gpio_get_direction(struct udevice *dev, unsigned int offset)
{
	struct rp1_pin_info *pin = rp1_get_pin(dev, offset);
	u32 fsel;

	if (!pin)
		return -EINVAL;

	fsel = rp1_get_fsel(pin);
	if (fsel != RP1_FSEL_GPIO)
		return -EINVAL;

	return (rp1_get_dir(pin) == RP1_DIR_OUTPUT) ?
			GPIOF_OUTPUT :
			GPIOF_INPUT;
}

static const struct dm_gpio_ops rp1_gpio_ops = {
	.direction_input = rp1_gpio_direction_input,
	.direction_output = rp1_gpio_direction_output,
	.get_value = rp1_gpio_get,
	.set_value = rp1_gpio_set,
	.get_function = rp1_gpio_get_direction,
};

static int rp1_gpio_probe(struct udevice *dev)
{
	int i;
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct rp1_gpio_priv *priv = dev_get_priv(dev);

	priv->gpio_base = dev_remap_addr_index(dev, 0);
	if (!priv->gpio_base)
		return -EINVAL;

	priv->rio_base = dev_remap_addr_index(dev, 1);
	if (!priv->rio_base)
		return -EINVAL;

	priv->pads_base = dev_remap_addr_index(dev, 2);
	if (!priv->pads_base)
		return -EINVAL;

	uc_priv->gpio_count = RP1_NUM_GPIOS;
	uc_priv->bank_name = dev->name;

	for (i = 0; i < RP1_NUM_BANKS; i++) {
		const struct rp1_iobank_desc *bank = &rp1_iobanks[i];
		int j;

		for (j = 0; j < bank->num_gpios; j++) {
			struct rp1_pin_info *pin =
				&priv->pins[bank->min_gpio + j];

			pin->num = bank->min_gpio + j;
			pin->bank = i;
			pin->offset = j;

			pin->gpio = priv->gpio_base + bank->gpio_offset +
				    j * sizeof(u32) * 2;
			pin->inte = priv->gpio_base + bank->inte_offset;
			pin->ints = priv->gpio_base + bank->ints_offset;
			pin->rio  = priv->rio_base + bank->rio_offset;
			pin->pad  = priv->pads_base + bank->pads_offset +
				    j * sizeof(u32);
		}
	}

	return 0;
}

static const struct udevice_id rp1_gpio_ids[] = {
	{ .compatible = "raspberrypi,rp1-gpio" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(rp1_gpio) = {
	.name = "rp1-gpio",
	.id = UCLASS_GPIO,
	.of_match = rp1_gpio_ids,
	.ops = &rp1_gpio_ops,
	.priv_auto	= sizeof(struct rp1_gpio_priv),
	.probe = rp1_gpio_probe,
};
