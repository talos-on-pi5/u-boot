// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB PCIe controller driver
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * Based on upstream Linux kernel driver:
 * drivers/pci/controller/pcie-brcmstb.c
 * Copyright (C) 2009 - 2017 Broadcom
 *
 * Based driver by Nicolas Saenz Julienne
 * Copyright (C) 2020 Nicolas Saenz Julienne <nsaenzjulienne@suse.de>
 *
 * Extended based on raspberrypi/linux:drivers/pci/controller/pcie-brcmstb.c
 * Copyright (C) 2009 - 2019 Broadcom
 *
 */

#include <asm/arch/acpi/bcm2711.h>
#include <asm/system.h>
#include <errno.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <pci.h>
#include <asm/io.h>
#include <linux/bitfield.h>
#include <linux/log2.h>
#include <linux/iopoll.h>
#include <reset.h>

/* PCIe parameters */
#define BRCM_NUM_PCIE_OUT_WINS				4
#define PCIE_BRCM_MAX_INBOUND_WINS		16

/* MDIO registers */
#define MDIO_PORT0					0x0
#define MDIO_DATA_MASK					0x7fffffff
#define MDIO_DATA_SHIFT					0
#define MDIO_PORT_MASK					0xf0000
#define MDIO_PORT_SHIFT					16
#define MDIO_REGAD_MASK					0xffff
#define MDIO_REGAD_SHIFT				0
#define MDIO_CMD_MASK					0xfff00000
#define MDIO_CMD_SHIFT					20
#define MDIO_CMD_READ					0x1
#define MDIO_CMD_WRITE					0x0
#define MDIO_DATA_DONE_MASK				0x80000000
#define SSC_REGS_ADDR					0x1100
#define SET_ADDR_OFFSET					0x1f
#define SSC_CNTL_OFFSET					0x2
#define SSC_CNTL_OVRD_EN_MASK				0x8000
#define SSC_CNTL_OVRD_VAL_MASK				0x4000
#define SSC_STATUS_OFFSET				0x1
#define SSC_STATUS_SSC_MASK				0x400
#define SSC_STATUS_SSC_SHIFT				10
#define SSC_STATUS_PLL_LOCK_MASK			0x800
#define SSC_STATUS_PLL_LOCK_SHIFT			11
#define PCIE_BRCM_MAX_MEMC		3

/* Broadcom STB PCIe Register Offsets */
#define PCIE_RC_PL_PHY_CTL_15											0x184c
#define PCIE_RC_PL_PHY_CTL_15_PM_CLK_PERIOD_MASK	0xff

/* BCM7712/2712-specific registers */
#define PCIE_MISC_UBUS_CTRL																	0x40a4
#define PCIE_MISC_UBUS_CTRL_UBUS_PCIE_REPLY_ERR_DIS_MASK		BIT(13)
#define PCIE_MISC_UBUS_CTRL_UBUS_PCIE_REPLY_DECERR_DIS_MASK	BIT(19)
#define PCIE_MISC_UBUS_TIMEOUT															0x40a8
#define PCIE_MISC_UBUS_BAR1_CONFIG_REMAP_ACCESS_EN_MASK			BIT(0)
#define PCIE_MISC_UBUS_BAR1_CONFIG_REMAP										0x40ac
#define PCIE_MISC_UBUS_BAR4_CONFIG_REMAP										0x410c

#define PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT 0x405c

#define PCIE_MISC_HARD_PCIE_HARD_DEBUG_BCM2712 0x4304

#define PCIE_MISC_PCIE_CTRL												0x4064
#define PCIE_MISC_PCIE_CTRL_PCIE_PERSTB_MASK			0x4
#define PCIE_MISC_CTRL_1													0x40A0
#define  PCIE_MISC_CTRL_1_EN_VDM_QOS_CONTROL_MASK BIT(5)

#define PCIE_MISC_AXI_INTF_CTRL										0x416C
#define  AXI_EN_RCLK_QOS_ARRAY_FIX								BIT(13)
#define  AXI_EN_QOS_UPDATE_TIMING_FIX							BIT(12)
#define  AXI_DIS_QOS_GATING_IN_MASTER							BIT(11)
#define  AXI_REQFIFO_EN_QOS_PROPAGATION						BIT(7)
#define  AXI_MASTER_MAX_OUTSTANDING_REQUESTS_MASK	0x3f

/* VDM messages indexing TCs to AXI priorities */
#define PCIE_MISC_AXI_READ_ERROR_DATA 0x4170

enum brcm_pcie_type {
	BRCM_PCIE_BCM_GENERIC,
	BRCM_PCIE_BCM2712,
};

/**
 * struct brcm_pcie - the PCIe controller state
 * @dev: The udevice
 * @base: Base address of memory mapped IO registers of the controller
 * @type: The type of the brcm type will be used to vary procedures as necessary.
 * @gen: Non-zero value indicates limitation of the PCIe controller operation
 *       to a specific generation (1, 2 or 3)
 * @ssc: true indicates active Spread Spectrum Clocking operation
 */
struct brcm_pcie {
	struct udevice		*dev;
	void __iomem		*base;

	enum brcm_pcie_type type;
	int			gen;
	bool			ssc;

	struct reset_ctl rescal;

	int		num_memc;
	u64		memc_size[PCIE_BRCM_MAX_MEMC];
};

/**
 * Inbound window specifications
*/
struct inbound_win {
	u64 size;
	u64 pci_offset;
	u64 cpu_addr;
};

/**
 * brcm_pcie_encode_ibar_size() - Encode the inbound "BAR" region size
 * @size: The inbound region size
 *
 * This function converts size of the inbound "BAR" region to the non-linear
 * values of the PCIE_MISC_RC_BAR[123]_CONFIG_LO register SIZE field.
 *
 * Return: The encoded inbound region size
 */
static int brcm_pcie_encode_ibar_size(u64 size)
{
	int log2_in = ilog2(size);

	if (log2_in >= 12 && log2_in <= 15)
		/* Covers 4KB to 32KB (inclusive) */
		return (log2_in - 12) + 0x1c;
	else if (log2_in >= 16 && log2_in <= 37)
		/* Covers 64KB to 32GB, (inclusive) */
		return log2_in - 15;

	/* Something is awry so disable */
	return 0;
}

/**
 * brcm_pcie_rc_mode() - Check if PCIe controller is in RC mode
 * @pcie: Pointer to the PCIe controller state
 *
 * The controller is capable of serving in both RC and EP roles.
 *
 * Return: true for RC mode, false for EP mode.
 */
static bool brcm_pcie_rc_mode(struct brcm_pcie *pcie)
{
	u32 val;

	val = readl(pcie->base + PCIE_MISC_PCIE_STATUS);

	return (val & STATUS_PCIE_PORT_MASK) >> STATUS_PCIE_PORT_SHIFT;
}

/**
 * brcm_pcie_link_up() - Check whether the PCIe link is up
 * @pcie: Pointer to the PCIe controller state
 *
 * Return: true if the link is up, false otherwise.
 */
static bool brcm_pcie_link_up(struct brcm_pcie *pcie)
{
	u32 val, dla, plu;

	val = readl(pcie->base + PCIE_MISC_PCIE_STATUS);
	dla = (val & STATUS_PCIE_DL_ACTIVE_MASK) >> STATUS_PCIE_DL_ACTIVE_SHIFT;
	plu = (val & STATUS_PCIE_PHYLINKUP_MASK) >> STATUS_PCIE_PHYLINKUP_SHIFT;

	return dla && plu;
}

static int brcm_pcie_config_address(const struct udevice *dev, pci_dev_t bdf,
				    uint offset, void **paddress)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	unsigned int pci_bus = PCI_BUS(bdf);
	unsigned int pci_dev = PCI_DEV(bdf);
	unsigned int pci_func = PCI_FUNC(bdf);
	int idx;

	/*
	 * Busses 0 (host PCIe bridge) and 1 (its immediate child)
	 * are limited to a single device each
	 */
	if (pci_bus < 2 && pci_dev > 0)
		return -EINVAL;

	/* Accesses to the RC go right to the RC registers */
	if (pci_bus == 0) {
		*paddress = pcie->base + offset;
		return 0;
	}

	/* An access to our HW w/o link-up will cause a CPU Abort */
	if (!brcm_pcie_link_up(pcie))
		return -EINVAL;

	/* For devices, write to the config space index register */
	idx = PCIE_ECAM_OFFSET(pci_bus, pci_dev, pci_func, 0);

	writel(idx, pcie->base + PCIE_EXT_CFG_INDEX);
	*paddress = pcie->base + PCIE_EXT_CFG_DATA + offset;

	return 0;
}

static int brcm_pcie_read_config(const struct udevice *bus, pci_dev_t bdf,
				 uint offset, ulong *valuep,
				 enum pci_size_t size)
{
	return pci_generic_mmap_read_config(bus, brcm_pcie_config_address,
					    bdf, offset, valuep, size);
}

static int brcm_pcie_write_config(struct udevice *bus, pci_dev_t bdf,
				  uint offset, ulong value,
				  enum pci_size_t size)
{
	void *addr;
	int ret = brcm_pcie_config_address(bus, bdf, offset, &addr);

	if (ret)
		return ret;

	ulong valb, vala;
	brcm_pcie_read_config(bus, bdf, offset, &valb, size);

	ret = pci_generic_mmap_write_config(bus, brcm_pcie_config_address,
					     bdf, offset, value, size);

	brcm_pcie_read_config(bus, bdf, offset, &vala, size);

	return ret;
}

static const char *link_speed_to_str(unsigned int cls)
{
	switch (cls) {
	case PCI_EXP_LNKSTA_CLS_2_5GB: return "2.5";
	case PCI_EXP_LNKSTA_CLS_5_0GB: return "5.0";
	case PCI_EXP_LNKSTA_CLS_8_0GB: return "8.0";
	default:
		break;
	}

	return "??";
}

static u32 brcm_pcie_mdio_form_pkt(unsigned int port, unsigned int regad,
				   unsigned int cmd)
{
	u32 pkt;

	pkt = (port << MDIO_PORT_SHIFT) & MDIO_PORT_MASK;
	pkt |= (regad << MDIO_REGAD_SHIFT) & MDIO_REGAD_MASK;
	pkt |= (cmd << MDIO_CMD_SHIFT) & MDIO_CMD_MASK;

	return pkt;
}

/**
 * brcm_pcie_mdio_read() - Perform a register read on the internal MDIO bus
 * @base: Pointer to the PCIe controller IO registers
 * @port: The MDIO port number
 * @regad: The register address
 * @val: A pointer at which to store the read value
 *
 * Return: 0 on success and register value in @val, negative error value
 *         on failure.
 */
static int brcm_pcie_mdio_read(void __iomem *base, unsigned int port,
			       unsigned int regad, u32 *val)
{
	u32 data, addr;
	int ret;

	addr = brcm_pcie_mdio_form_pkt(port, regad, MDIO_CMD_READ);
	writel(addr, base + PCIE_RC_DL_MDIO_ADDR);
	readl(base + PCIE_RC_DL_MDIO_ADDR);

	ret = readl_poll_timeout(base + PCIE_RC_DL_MDIO_RD_DATA, data,
				 (data & MDIO_DATA_DONE_MASK), 100);

	*val = data & MDIO_DATA_MASK;

	return ret;
}

/**
 * brcm_pcie_mdio_write() - Perform a register write on the internal MDIO bus
 * @base: Pointer to the PCIe controller IO registers
 * @port: The MDIO port number
 * @regad: Address of the register
 * @wrdata: The value to write
 *
 * Return: 0 on success, negative error value on failure.
 */
static int brcm_pcie_mdio_write(void __iomem *base, unsigned int port,
				unsigned int regad, u16 wrdata)
{
	u32 data, addr;

	addr = brcm_pcie_mdio_form_pkt(port, regad, MDIO_CMD_WRITE);
	writel(addr, base + PCIE_RC_DL_MDIO_ADDR);
	readl(base + PCIE_RC_DL_MDIO_ADDR);
	writel(MDIO_DATA_DONE_MASK | wrdata, base + PCIE_RC_DL_MDIO_WR_DATA);

	return readl_poll_timeout(base + PCIE_RC_DL_MDIO_WR_DATA, data,
				  !(data & MDIO_DATA_DONE_MASK), 100);
}

/**
 * brcm_pcie_set_ssc() - Configure the controller for Spread Spectrum Clocking
 * @base: pointer to the PCIe controller IO registers
 *
 * Return: 0 on success, negative error value on failure.
 */
static int brcm_pcie_set_ssc(void __iomem *base)
{
	int pll, ssc;
	int ret;
	u32 tmp;

	ret = brcm_pcie_mdio_write(base, MDIO_PORT0, SET_ADDR_OFFSET,
				   SSC_REGS_ADDR);
	if (ret < 0)
		return ret;

	ret = brcm_pcie_mdio_read(base, MDIO_PORT0, SSC_CNTL_OFFSET, &tmp);
	if (ret < 0)
		return ret;

	tmp |= (SSC_CNTL_OVRD_EN_MASK | SSC_CNTL_OVRD_VAL_MASK);

	ret = brcm_pcie_mdio_write(base, MDIO_PORT0, SSC_CNTL_OFFSET, tmp);
	if (ret < 0)
		return ret;

	udelay(1000);
	ret = brcm_pcie_mdio_read(base, MDIO_PORT0, SSC_STATUS_OFFSET, &tmp);
	if (ret < 0)
		return ret;

	ssc = (tmp & SSC_STATUS_SSC_MASK) >> SSC_STATUS_SSC_SHIFT;
	pll = (tmp & SSC_STATUS_PLL_LOCK_MASK) >> SSC_STATUS_PLL_LOCK_SHIFT;

	return ssc && pll ? 0 : -EIO;
}

/**
 * brcm_pcie_set_gen() - Limits operation to a specific generation (1, 2 or 3)
 * @pcie: pointer to the PCIe controller state
 * @gen: PCIe generation to limit the controller's operation to
 */
static void brcm_pcie_set_gen(struct brcm_pcie *pcie, unsigned int gen)
{
	void __iomem *cap_base = pcie->base + BRCM_PCIE_CAP_REGS;

	u16 lnkctl2 = readw(cap_base + PCI_EXP_LNKCTL2);
	u32 lnkcap = readl(cap_base + PCI_EXP_LNKCAP);

	lnkcap = (lnkcap & ~PCI_EXP_LNKCAP_SLS) | gen;
	writel(lnkcap, cap_base + PCI_EXP_LNKCAP);

	lnkctl2 = (lnkctl2 & ~0xf) | gen;
	writew(lnkctl2, cap_base + PCI_EXP_LNKCTL2);
}

static void brcm_pcie_set_outbound_win(struct brcm_pcie *pcie,
				       unsigned int win, u64 phys_addr,
				       u64 pcie_addr, u64 size)
{
	void __iomem *base = pcie->base;
	u32 phys_addr_mb_high, limit_addr_mb_high;
	phys_addr_t phys_addr_mb, limit_addr_mb;
	int high_addr_shift;
	u32 tmp;

	if (!size || (size & (size - 1)) != 0) {
		printf("Skipping bogus outbound window: phys_addr=0x%llx, pcie_addr=0x%llx, size=0x%llx\n", phys_addr, pcie_addr, size);
		return;
	}

	/* Map the space in the MMU */
	mmu_map_region(phys_addr, size, false);
	mmu_set_region_dcache_behaviour(phys_addr, size, DCACHE_OFF);

	/* Flush and invalidate cache for safety (esp. on MMIO) */
	if (size != 0 && (size & 0xFFF) == 0) {
		invalidate_dcache_range(phys_addr, phys_addr + size);
	}

	/* Give the endpoint time to become ready */
	udelay(20);

	/* Set the base of the pcie_addr window */
	writel(lower_32_bits(pcie_addr), base + PCIE_MEM_WIN0_LO(win));
	writel(upper_32_bits(pcie_addr), base + PCIE_MEM_WIN0_HI(win));

	/* Write the addr base & limit lower bits (in MBs) */
	phys_addr_mb = phys_addr / SZ_1M;
	limit_addr_mb = (phys_addr + size - 1) / SZ_1M;

	tmp = readl(base + PCIE_MEM_WIN0_BASE_LIMIT(win));
	u32p_replace_bits(&tmp, phys_addr_mb,
			  MEM_WIN0_BASE_LIMIT_BASE_MASK);
	u32p_replace_bits(&tmp, limit_addr_mb,
			  MEM_WIN0_BASE_LIMIT_LIMIT_MASK);
	writel(tmp, base + PCIE_MEM_WIN0_BASE_LIMIT(win));

	/* Write the cpu & limit addr upper bits */
	high_addr_shift = MEM_WIN0_BASE_LIMIT_BASE_HI_SHIFT;
	phys_addr_mb_high = phys_addr_mb >> high_addr_shift;
	tmp = readl(base + PCIE_MEM_WIN0_BASE_HI(win));
	u32p_replace_bits(&tmp, phys_addr_mb_high,
			  MEM_WIN0_BASE_HI_BASE_MASK);
	writel(tmp, base + PCIE_MEM_WIN0_BASE_HI(win));

	limit_addr_mb_high = limit_addr_mb >> high_addr_shift;
	tmp = readl(base + PCIE_MEM_WIN0_LIMIT_HI(win));
	u32p_replace_bits(&tmp, limit_addr_mb_high,
			  PCIE_MEM_WIN0_LIMIT_HI_LIMIT_MASK);
	writel(tmp, base + PCIE_MEM_WIN0_LIMIT_HI(win));
}

static int brcm_pcie_post_setup_bcm2712(struct brcm_pcie *pcie)
{
	const u16 data[] = { 0x50b9, 0xbda1, 0x0094, 0x97b4, 0x5030, 0x5030, 0x0007 };
	const u8 regs[] = { 0x16, 0x17, 0x18, 0x19, 0x1b, 0x1c, 0x1e };
	int ret, i;
	u32 tmp;

	/* Allow a 54MHz (xosc) refclk source */
	ret = brcm_pcie_mdio_write(pcie->base, MDIO_PORT0, SET_ADDR_OFFSET, 0x1600);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = brcm_pcie_mdio_write(pcie->base, MDIO_PORT0, regs[i], data[i]);
		if (ret < 0)
			return ret;
	}

	udelay(200);

	/*
	 * Set L1SS sub-state timers to avoid lengthy state transitions,
	 * PM clock period is 18.52ns (1/54MHz, round down).
	 */
	tmp = readl(pcie->base + PCIE_RC_PL_PHY_CTL_15);
	tmp &= ~PCIE_RC_PL_PHY_CTL_15_PM_CLK_PERIOD_MASK;
	tmp |= 0x12;
	writel(tmp, pcie->base + PCIE_RC_PL_PHY_CTL_15);

	/*
	 * BCM7712/2712 uses a UBUS-AXI bridge.
	 * Suppress AXI error responses and return 1s for read failures.
	 */
	tmp = readl(pcie->base + PCIE_MISC_UBUS_CTRL);
	u32p_replace_bits(&tmp, 1, PCIE_MISC_UBUS_CTRL_UBUS_PCIE_REPLY_ERR_DIS_MASK);
	u32p_replace_bits(&tmp, 1, PCIE_MISC_UBUS_CTRL_UBUS_PCIE_REPLY_DECERR_DIS_MASK);
	writel(tmp, pcie->base + PCIE_MISC_UBUS_CTRL);
	writel(0xffffffff, pcie->base + PCIE_MISC_AXI_READ_ERROR_DATA);

	/*
	 * Adjust timeouts. The UBUS timeout also affects Configuration Request
	 * Retry responses, as the request will get terminated if
	 * either timeout expires, so both have to be a large value
	 * (in clocks of 750MHz).
	 * Set UBUS timeout to 250ms, then set RC config retry timeout
	 * to be ~240ms.
	 *
	 * If CRSSVE=1 this will stop the core from blocking on a Retry
	 * response, but does require the device to be well-behaved...
	 */
	writel(0xB2D0000, pcie->base + PCIE_MISC_UBUS_TIMEOUT);
	writel(0xABA0000, pcie->base + PCIE_MISC_RC_CONFIG_RETRY_TIMEOUT);

	/*
	 * BCM2712 has a configurable QoS mechanism that assigns TLP Traffic Classes
	 * to separate AXI IDs with a configurable priority scheme.
	 * Dynamic priority elevation is supported through reception of Type 1
	 * Vendor Defined Messages, but several bugs make this largely ineffective.
	 */

	/* Disable broken forwarding search. Set chicken bits for 2712D0 */
	tmp = readl(pcie->base + PCIE_MISC_AXI_INTF_CTRL);
	tmp &= ~AXI_REQFIFO_EN_QOS_PROPAGATION;
	tmp |= AXI_EN_RCLK_QOS_ARRAY_FIX | AXI_EN_QOS_UPDATE_TIMING_FIX |
		AXI_DIS_QOS_GATING_IN_MASTER;
	writel(tmp, pcie->base + PCIE_MISC_AXI_INTF_CTRL);

	/*
	 * Work around spurious QoS=0 assignments to inbound traffic.
	 * If the QOS_UPDATE_TIMING_FIX bit is Reserved-0, then this is a
	 * 2712C1 chip, or a single-lane RC. Use the  best-effort alternative
	 * which is to partially throttle AXI requests in-flight to SDRAM.
	 */
	tmp = readl(pcie->base + PCIE_MISC_AXI_INTF_CTRL);
	if (!(tmp & AXI_EN_QOS_UPDATE_TIMING_FIX)) {
		tmp &= ~AXI_MASTER_MAX_OUTSTANDING_REQUESTS_MASK;
		tmp |= 15;
		writel(tmp, pcie->base + PCIE_MISC_AXI_INTF_CTRL);
	}

	/* Disable VDM reception by default */
	tmp = readl(pcie->base + PCIE_MISC_CTRL_1);
	tmp &= ~PCIE_MISC_CTRL_1_EN_VDM_QOS_CONTROL_MASK;
	writel(tmp, pcie->base + PCIE_MISC_CTRL_1);

	return 0;
}

static u32 brcm_bar_reg_offset(int bar)
{
	if (bar <= 3)
		return PCIE_MISC_RC_BAR1_CONFIG_LO + 8 * (bar - 1);
	else
		return PCIE_MISC_RC_BAR4_CONFIG_LO + 8 * (bar - 4);
}

static u32 brcm_ubus_reg_offset(int bar)
{
	if (bar <= 3)
		return PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 8 * (bar - 1);
	else
		return PCIE_MISC_UBUS_BAR4_CONFIG_REMAP + 8 * (bar - 4);
}

static void add_inbound_win(struct inbound_win *b, u8 *count, u64 size,
			    u64 cpu_addr, u64 pci_offset)
{
	b->size = size;
	b->cpu_addr = cpu_addr;
	b->pci_offset = pci_offset;
	(*count)++;
}

int brcm_of_property_read_variable_u64_array(struct udevice *dev, const char *propname,
																						u64 *out, int max_entries)
{
	const fdt64_t *prop;
	int len, count, i;

	prop = dev_read_prop(dev, propname, &len);
	if (!prop || len <= 0)
		return -ENOENT;

	count = len / sizeof(fdt64_t);
	if (count > max_entries)
		count = max_entries;

	for (i = 0; i < count; i++)
		out[i] = fdt64_to_cpu(prop[i]);

	return count;
}

static int brcm_pcie_get_inbound_wins(struct brcm_pcie *pcie, struct inbound_win *inbound_wins)
{
	u64 pci_offset, cpu_addr, size = 0, tot_size = 0;
	struct pci_region entry;
	struct udevice *dev = pcie->dev;
	u64 lowest_pcie_addr = ~(u64)0;
	int ret, i = 0;
	u8 n = 0;

	/*
	 * The HW registers (and PCIe) use order-1 numbering for BARs.  As such,
	 * we have inbound_wins[0] unused and BAR1 starts at inbound_wins[1].
	 */
	struct inbound_win *b_begin = &inbound_wins[1];
	struct inbound_win *b = b_begin;

	/*
	 * STB chips beside 7712 disable the first inbound window default.
	 * Rather being mapped to system memory it is mapped to the
	 * internal registers of the SoC.  This feature is deprecated, has
	 * security considerations, and is not implemented in our modern
	 * SoCs.
	 */
	if (pcie->type == BRCM_PCIE_BCM_GENERIC)
		add_inbound_win(b++, &n, 0, 0, 0);

	while (pci_get_dma_regions(dev, &entry, i++) == 0) {
		u64 pcie_start = entry.bus_start;
		u64 cpu_start = entry.phys_start;

		size = entry.size;
		tot_size += size;
		if (pcie_start < lowest_pcie_addr)
			lowest_pcie_addr = pcie_start;
		/*
		 * 7712 and newer chips may have many BARs, with each
		 * offering a non-overlapping viewport to system memory.
		 * That being said, each BARs size must still be a power of
		 * two.
		 */
		if (pcie->type == BRCM_PCIE_BCM2712)
			add_inbound_win(b++, &n, size, cpu_start, pcie_start);
	}

	if (lowest_pcie_addr == ~(u64)0) {
		printf("DT node has no dma-ranges\n");
		return -EINVAL;
	}

	/*
	 * 7712 and newer chips do not have an internal memory mapping system
	 * that enables multiple memory controllers.  As such, it can return
	 * now w/o doing special configuration.
	 */
	if (pcie->type == BRCM_PCIE_BCM2712)
		return n;

	ret = brcm_of_property_read_variable_u64_array(dev, "brcm,scb-sizes", pcie->memc_size, PCIE_BRCM_MAX_MEMC);
	if (ret <= 0) {
		/* Make an educated guess */
		pcie->num_memc = 1;
		pcie->memc_size[0] = 1ULL << fls64(tot_size - 1);
	} else {
		pcie->num_memc = ret;
	}

	/* Each memc is viewed through a "port" that is a power of 2 */
	for (i = 0, size = 0; i < pcie->num_memc; i++)
		size += pcie->memc_size[i];

	/* Our HW mandates that the window size must be a power of 2 */
	size = 1ULL << fls64(size - 1);

	/*
	 * For STB chips, the BAR2 cpu_addr is hardwired to the start
	 * of system memory, so we set it to 0.
	 */
	cpu_addr = 0;
	pci_offset = lowest_pcie_addr;

	/*
	 * We validate the inbound memory view even though we should trust
	 * whatever the device-tree provides. This is because of an HW issue on
	 * early Raspberry Pi 4's revisions (bcm2711). It turns out its
	 * firmware has to dynamically edit dma-ranges due to a bug on the
	 * PCIe controller integration, which prohibits any access above the
	 * lower 3GB of memory. Given this, we decided to keep the dma-ranges
	 * in check, avoiding hard to debug device-tree related issues in the
	 * future:
	 *
	 * The PCIe host controller by design must set the inbound viewport to
	 * be a contiguous arrangement of all of the system's memory.  In
	 * addition, its size mut be a power of two.  To further complicate
	 * matters, the viewport must start on a pcie-address that is aligned
	 * on a multiple of its size.  If a portion of the viewport does not
	 * represent system memory -- e.g. 3GB of memory requires a 4GB
	 * viewport -- we can map the outbound memory in or after 3GB and even
	 * though the viewport will overlap the outbound memory the controller
	 * will know to send outbound memory downstream and everything else
	 * upstream.
	 *
	 * For example:
	 *
	 * - The best-case scenario, memory up to 3GB, is to place the inbound
	 *   region in the first 4GB of pcie-space, as some legacy devices can
	 *   only address 32bits. We would also like to put the MSI under 4GB
	 *   as well, since some devices require a 32bit MSI target address.
	 *
	 * - If the system memory is 4GB or larger we cannot start the inbound
	 *   region at location 0 (since we have to allow some space for
	 *   outbound memory @ 3GB). So instead it will  start at the 1x
	 *   multiple of its size
	 */
	if (!size || (pci_offset & (size - 1)) ||
			(pci_offset < SZ_4G && pci_offset > SZ_2G)) {
		printf("Invalid inbound_win2_offset/size: size 0x%llx, off 0x%llx\n",
			size, pci_offset);
		return -EINVAL;
	}

	/* Enable inbound window 2, the main inbound window for STB chips */
	add_inbound_win(b++, &n, size, cpu_addr, pci_offset);

	/*
	 * Disable inbound window 3.  On some chips presents the same
	 * window as #2 but the data appears in a settable endianness.
	 */
	add_inbound_win(b++, &n, 0, 0, 0);

	return n;
}

static void brcm_set_inbound_win_registers(struct brcm_pcie *pcie,
				      const struct inbound_win *inbound_wins,
				      u8 num_inbound_wins)
{
	void __iomem *base = pcie->base;
	int i;

	for (i = 1; i <= num_inbound_wins; i++) {
		u64 pci_offset = inbound_wins[i].pci_offset;
		u64 cpu_addr = inbound_wins[i].cpu_addr;
		u64 size = inbound_wins[i].size;
		u32 reg_offset = brcm_bar_reg_offset(i);
		u32 tmp = lower_32_bits(pci_offset);

		u32p_replace_bits(&tmp, brcm_pcie_encode_ibar_size(size), RC_BAR1_CONFIG_LO_SIZE_MASK);

		/* Write low */
		writel_relaxed(tmp, base + reg_offset);
		/* Write high */
		writel_relaxed(upper_32_bits(pci_offset), base + reg_offset + 4);

		/*
		 * Most STB chips:
		 *     Do nothing.
		 * 7712:
		 *     All of their BARs need to be set.
		 */
		if (pcie->type == BRCM_PCIE_BCM2712) {
			/* BUS remap register settings */
			reg_offset = brcm_ubus_reg_offset(i);
			tmp = lower_32_bits(cpu_addr) & ~0xfff;
			tmp |= PCIE_MISC_UBUS_BAR1_CONFIG_REMAP_ACCESS_EN_MASK;
			writel_relaxed(tmp, base + reg_offset);
			tmp = upper_32_bits(cpu_addr);
			writel_relaxed(tmp, base + reg_offset + 4);
		}
	}
}

static int brcm_pcie_probe(struct udevice *dev)
{
	struct udevice *ctlr = pci_get_controller(dev);
	struct pci_controller *hose = dev_get_uclass_priv(ctlr);
	struct brcm_pcie *pcie = dev_get_priv(dev);
	pcie->type = (enum brcm_pcie_type)dev_get_driver_data(dev);
	pcie->dev = dev;
	void __iomem *base = pcie->base;
	bool ssc_good = false;
	int num_out_wins = 0;
	u32 scb_size_val;
	int i, ret;
	u16 nlw, cls, lnksta;
	u32 tmp;
	struct inbound_win inbound_wins[PCIE_BRCM_MAX_INBOUND_WINS];
	int num_inbound_wins = 0;

	ret = reset_get_by_name(dev, "rescal", &pcie->rescal);
	if (ret == -ENOENT) {
		/* no PHY reset defined in DT, that’s OK */
		printf("%s: no \"rescal\" reset, skipping\n", dev->name);
	} else if (ret) {
		printf("%s: failed to get rescal reset: %d\n", dev->name, ret);
		return ret;
	}

	/*
	 * Reset the bridge, assert the fundamental reset. Note for some SoCs,
	 * e.g. BCM7278, the fundamental reset should not be asserted here.
	 */
	tmp = PCIE_RGR1_SW_INIT_1_INIT_MASK;
	if (pcie->type == BRCM_PCIE_BCM_GENERIC)
		tmp |= PCIE_RGR1_SW_INIT_1_PERST_MASK;
	setbits_le32(base + PCIE_RGR1_SW_INIT_1, tmp);

	/*
	 * The delay is a safety precaution to preclude the reset signal
	 * from looking like a glitch.
	 */
	udelay(100);

	/* Take the bridge out of reset */
	clrbits_le32(base + PCIE_RGR1_SW_INIT_1, PCIE_RGR1_SW_INIT_1_INIT_MASK);

	if (pcie->type == BRCM_PCIE_BCM_GENERIC)
		clrbits_le32(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG,
					PCIE_HARD_DEBUG_SERDES_IDDQ_MASK);

	/* Wait for SerDes to be stable */
	udelay(100);

	/* Set SCB_MAX_BURST_SIZE, CFG_READ_UR_MODE, SCB_ACCESS_EN */
	tmp = MISC_CTRL_SCB_ACCESS_EN_MASK | MISC_CTRL_CFG_READ_UR_MODE_MASK;
	if (pcie->type == BRCM_PCIE_BCM2712) {
		tmp |= MISC_CTRL_MAX_BURST_SIZE_512 | MISC_CTRL_PCIE_RCB_MPS_MODE_MASK | MISC_CTRL_PCIE_RCB_64B_MODE_MASK;
	} else {
		tmp |= MISC_CTRL_MAX_BURST_SIZE_128;
	}
	clrsetbits_le32(base + PCIE_MISC_MISC_CTRL, MISC_CTRL_MAX_BURST_SIZE_MASK, tmp);

	/* Disable the PCIe->GISB memory window (RC_BAR1) */
	clrbits_le32(base + PCIE_MISC_RC_BAR1_CONFIG_LO,
		     RC_BAR1_CONFIG_LO_SIZE_MASK);

	/* Disable the PCIe->SCB memory window (RC_BAR3) */
	clrbits_le32(base + PCIE_MISC_RC_BAR3_CONFIG_LO,
		     RC_BAR3_CONFIG_LO_SIZE_MASK);

	/* Mask all interrupts since we are not handling any yet */
	writel(0xffffffff, base + PCIE_MSI_INTR2_MASK_SET);

	/* Clear any interrupts we find on boot */
	writel(0xffffffff, base + PCIE_MSI_INTR2_CLR);

	if (pcie->gen)
		brcm_pcie_set_gen(pcie, pcie->gen);

	/* Unassert the fundamental reset */
	if (pcie->type == BRCM_PCIE_BCM2712) {
		setbits_le32(base + PCIE_MISC_PCIE_CTRL, PCIE_MISC_PCIE_CTRL_PCIE_PERSTB_MASK);
	}
	clrbits_le32(pcie->base + PCIE_RGR1_SW_INIT_1,
		     PCIE_RGR1_SW_INIT_1_PERST_MASK);

	/*
	 * Wait for 100ms after PERST# deassertion; see PCIe CEM specification
	 * sections 2.2, PCIe r5.0, 6.6.1.
	 */
	mdelay(100);

	/* Give the RC/EP time to wake up, before trying to configure RC.
	 * Intermittently check status for link-up, up to a total of 100ms.
	 */
	for (i = 0; i < 100 && !brcm_pcie_link_up(pcie); i += 5)
		mdelay(5);

	if (!brcm_pcie_link_up(pcie)) {
		printf("PCIe BRCM: link down\n");
		return -EINVAL;
	}

	if (!brcm_pcie_rc_mode(pcie)) {
		printf("PCIe misconfigured; is in EP mode\n");
		return -EINVAL;
	}

	/*
	 * Inbound window setup
	 */
	num_inbound_wins = brcm_pcie_get_inbound_wins(pcie, inbound_wins);
	if (num_inbound_wins < 0)
		return num_inbound_wins;
	brcm_set_inbound_win_registers(pcie, inbound_wins, num_inbound_wins+1);

	tmp = readl(base + PCIE_MISC_MISC_CTRL);
	for (int memc = 0; memc < pcie->num_memc; memc++) {
		scb_size_val = ilog2(pcie->memc_size[memc]) - 15;

		if (memc == 0)
			u32p_replace_bits(&tmp, scb_size_val, SCB_SIZE_MASK(0));
		else if (memc == 1)
			u32p_replace_bits(&tmp, scb_size_val, SCB_SIZE_MASK(1));
		else if (memc == 2)
			u32p_replace_bits(&tmp, scb_size_val, SCB_SIZE_MASK(2));
	}
	writel(tmp, base + PCIE_MISC_MISC_CTRL);

	/*
	 * Outbound window setup
	 */
	for (i = 0; i < hose->region_count; i++) {
		struct pci_region *reg = &hose->regions[i];

		if (reg->flags != PCI_REGION_MEM)
			continue;

		if (num_out_wins >= BRCM_NUM_PCIE_OUT_WINS)
			return -EINVAL;

		brcm_pcie_set_outbound_win(pcie, num_out_wins, reg->phys_start,
					   reg->bus_start, reg->size);

		num_out_wins++;
	}

	/*
	 * For config space accesses on the RC, show the right class for
	 * a PCIe-PCIe bridge (the default setting is to be EP mode).
	 */
	clrsetbits_le32(base + PCIE_RC_CFG_PRIV1_ID_VAL3,
			PCIE_RC_CFG_PRIV1_ID_VAL3_CLASS_CODE_MASK, 0x060400);

	if (pcie->ssc) {
		ret = brcm_pcie_set_ssc(pcie->base);
		if (!ret)
			ssc_good = true;
		else
			printf("PCIe BRCM: failed attempt to enter SSC mode\n");
	}

	lnksta = readw(base + BRCM_PCIE_CAP_REGS + PCI_EXP_LNKSTA);
	cls = lnksta & PCI_EXP_LNKSTA_CLS;
	nlw = (lnksta & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;

	printf("PCIe BRCM: link up, %s Gbps x%u %s\n", link_speed_to_str(cls),
				nlw, ssc_good ? "(SSC)" : "(!SSC)");

	if (pcie->rescal.dev) {
		reset_assert(&pcie->rescal);
		udelay(100);
		reset_deassert(&pcie->rescal);
	}

	/*
	 * RootCtl bits are reset by perst_n, which undoes pci_enable_crs()
	 * called prior to pci_add_new_bus() during probe. Re-enable here.
	 */
	if (pcie->type == BRCM_PCIE_BCM2712) {
		setbits_le32(base + BRCM_PCIE_CAP_REGS + PCI_EXP_RTCTL, PCI_EXP_RTCTL_CRSSVE);
	}

	/* PCIe->SCB endian mode for BAR */
	clrsetbits_le32(base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1,
			PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK,
			VENDOR_SPECIFIC_REG1_LITTLE_ENDIAN);

	/*
	 * We used to enable the CLKREQ# input here, but a few PCIe cards don't
	 * attach anything to the CLKREQ# line, so we shouldn't assume that
	 * it's connected and working. The controller does allow detecting
	 * whether the port on the other side of our link is/was driving this
	 * signal, so we could check before we assume. But because this signal
	 * is for power management, which doesn't make sense in a bootloader,
	 * let's instead just unadvertise ASPM support.
	 */
	clrbits_le32(base + PCIE_RC_CFG_PRIV1_LINK_CAPABILITY,
		     LINK_CAPABILITY_ASPM_SUPPORT_MASK);

	if (pcie->type == BRCM_PCIE_BCM2712) {
		return brcm_pcie_post_setup_bcm2712(pcie);
	}

	return 0;
}

static int brcm_pcie_remove(struct udevice *dev)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	void __iomem *base = pcie->base;

	/* Assert fundamental reset */
	setbits_le32(base + PCIE_RGR1_SW_INIT_1, PCIE_RGR1_SW_INIT_1_PERST_MASK);

	/* Turn off SerDes */
	setbits_le32(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG,
		     PCIE_HARD_DEBUG_SERDES_IDDQ_MASK);

	/* Shutdown bridge */
	setbits_le32(base + PCIE_RGR1_SW_INIT_1, PCIE_RGR1_SW_INIT_1_INIT_MASK);

	return 0;
}

static int brcm_pcie_of_to_plat(struct udevice *dev)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	ofnode dn = dev_ofnode(dev);
	u32 max_link_speed;
	int ret;

	/* Get the controller base address */
	pcie->base = dev_read_addr_ptr(dev);
	if (!pcie->base)
		return -EINVAL;

	pcie->ssc = ofnode_read_bool(dn, "brcm,enable-ssc");

	ret = ofnode_read_u32(dn, "max-link-speed", &max_link_speed);
	if (ret < 0 || max_link_speed > 4)
		pcie->gen = 0;
	else
		pcie->gen = max_link_speed;

	return 0;
}

static const struct dm_pci_ops brcm_pcie_ops = {
	.read_config	= brcm_pcie_read_config,
	.write_config	= brcm_pcie_write_config,
};

static const struct udevice_id brcm_pcie_ids[] = {
	{ .compatible = "brcm,bcm2711-pcie", .data = (ulong)BRCM_PCIE_BCM_GENERIC },
	{ .compatible = "brcm,bcm2712-pcie", .data = (ulong)BRCM_PCIE_BCM2712 },
	{ }
};

U_BOOT_DRIVER(pcie_brcm_base) = {
	.name			= "pcie_brcm",
	.id			= UCLASS_PCI,
	.ops			= &brcm_pcie_ops,
	.of_match		= brcm_pcie_ids,
	.probe			= brcm_pcie_probe,
	.remove			= brcm_pcie_remove,
	.of_to_plat	= brcm_pcie_of_to_plat,
	.priv_auto	= sizeof(struct brcm_pcie),
	.flags		= DM_FLAG_OS_PREPARE,
};
