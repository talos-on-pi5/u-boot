// SPDX-License-Identifier: GPL-2.0
/*
 * Based on raspberrypi/linux:drivers/reset/reset-brcmstb-rescal.c
 * Copyright (C) 2018-2020 Broadcom
 *
 */

#include <asm/io.h>
#include <dm.h>
#include <reset-uclass.h>
#include <dm/device_compat.h>
#include <linux/delay.h>

#define BRCM_RESCAL_START				0x0
#define  BRCM_RESCAL_START_BIT	BIT(0)
#define BRCM_RESCAL_CTRL				0x4
#define BRCM_RESCAL_STATUS			0x8
#define  BRCM_RESCAL_STATUS_BIT	BIT(0)

struct brcm_rescal_priv {
	void __iomem *base;
};

/* Also doubles a deassert */
static int brcm_rescal_reset_set(struct reset_ctl *rctl)
{
	struct udevice *dev = rctl->dev;
	struct brcm_rescal_priv *data = dev_get_priv(dev);
	void __iomem *base = data->base;
	u32 reg;
	ulong start;

	reg = readl(base + BRCM_RESCAL_START);
	writel(reg | BRCM_RESCAL_START_BIT, base + BRCM_RESCAL_START);
	reg = readl(base + BRCM_RESCAL_START);
	if (!(reg & BRCM_RESCAL_START_BIT)) {
		dev_err(dev, "failed to start SATA/PCIe rescal\n");
		return -EIO;
	}

	/* wait for the status bit to go high */
	start = get_timer(0);
	do {
		reg = readl(base + BRCM_RESCAL_STATUS);
		if (reg & BRCM_RESCAL_STATUS_BIT)
			break;
		udelay(100);
	} while (get_timer(start) < 10);

	if (!(reg & BRCM_RESCAL_STATUS_BIT)) {
		dev_err(dev, "time out on SATA/PCIe rescal\n");
		return -ETIMEDOUT;
	}

	reg = readl(base + BRCM_RESCAL_START);
	writel(reg & ~BRCM_RESCAL_START_BIT, base + BRCM_RESCAL_START);

	dev_dbg(dev, "SATA/PCIe rescal success\n");

	return 0;
}

/* A dummy function - deassert/reset does all the work */
static int brcm_rescal_reset_assert(struct reset_ctl *rctl)
{
	return 0;
}

static int brcm_rescal_reset_probe(struct udevice *dev)
{
	struct brcm_rescal_priv *data = dev_get_priv(dev);

	data->base = devfdt_remap_addr(dev);
	if (IS_ERR(data->base))
		return -EINVAL;

	return 0;
}

static int brcm_rescal_reset_request(struct reset_ctl *rctl)
{
	return 0;
}

static int brcm_rescal_reset_free(struct reset_ctl *rctl)
{
	return 0;
}

static int brcm_rescal_reset_xlate(struct reset_ctl *rst, struct ofnode_phandle_args *args)
{
	/* This is needed if #reset-cells == 0. */
	return 0;
}

static const struct reset_ops brcm_rescal_ops = {
	.request      = brcm_rescal_reset_request,
	.rfree        = brcm_rescal_reset_free,
	.rst_assert   = brcm_rescal_reset_assert,
	.rst_deassert = brcm_rescal_reset_set,
	.of_xlate     = brcm_rescal_reset_xlate,
};

static const struct udevice_id brcm_rescal_ids[] = {
	{ .compatible = "brcm,bcm7216-pcie-sata-rescal" },
	{ }
};

U_BOOT_DRIVER(pcie_rescal) = {
	.name        = "pcie_brcmstb_rescal",
	.id          = UCLASS_RESET,
	.of_match    = brcm_rescal_ids,
	.ops         = &brcm_rescal_ops,
	.priv_auto   = sizeof(struct brcm_rescal_priv),
	.probe       = brcm_rescal_reset_probe,
};
