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
 */

#include <asm/arch/acpi/bcm2711.h>
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
#define BRCM_MAX_INBOUND_WINS		16
#define BRCM_MAX_MEMC		3

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

enum {
	RGR1_SW_INIT_1,
	PCIE_HARD_DEBUG,
	MAX_BURST_SIZE_128,
};

enum brcm_pcie_type {
	BRCM_PCIE_BCM_GENERIC,
	BRCM_PCIE_BCM2712,
};

/**
 * struct inbound_win - PCIe inbound window mapping
 * @size:		Size of the inbound window (in bytes)
 * @pci_offset:		Base address of the window as seen from the PCIe (bus) side
 * @cpu_addr:		Base address of the window in system (CPU physical) memory
 */
struct inbound_win {
	u64 size;
	u64 pci_offset;
	u64 cpu_addr;
};

struct brcm_pcie;

/**
 * struct brcm_pcie_cfg_data - SoC-specific PCIe controller config and callbacks
 * @offsets:		Pointer to SoC-specific register offset table
 * @type:		PCIe controller hardware type (see enum brcm_pcie_type)
 *
 * @perst_set:		Function to assert/deassert PERST# (PCIe reset signal)
 * @bridge_sw_init_set:	Function to control bridge software initialization sequence
 * @rc_mode:		Function to check if controller is operating in Root Complex (RC) mode
 * @get_inbound_wins:	Function to populate PCIe inbound window mapping for this controller
 * @post_setup:		Optional: Function to run extra SoC-specific setup after standard init
 */
struct brcm_pcie_cfg_data {
	const int *offsets;
	const enum brcm_pcie_type type;

	void (*perst_set)(struct brcm_pcie *pcie, u32 val);
	void (*bridge_sw_init_set)(struct brcm_pcie *pcie, u32 val);
	bool (*rc_mode)(struct brcm_pcie *pcie);
	int (*get_inbound_wins)(struct brcm_pcie *pcie, struct inbound_win *inbound_wins);
	int (*post_setup)(struct brcm_pcie *pcie);
};

/**
 * struct brcm_pcie - the PCIe controller state
 * @dev:	Pointer to the associated U-Boot device instance
 * @base: Base address of memory mapped IO registers of the controller
 * @bus_base:	PCI bus base number managed by this controller
 *
 * @cfg:	Pointer to SoC-specific PCIe configuration data
 *
 * @gen: Non-zero value indicates limitation of the PCIe controller operation
 *       to a specific generation (1, 2 or 3)
 * @ssc: true indicates active Spread Spectrum Clocking operation
 *
 * @rescal:	Reset controller handle for "rescal" (PHY calibration)
 * @bridge_reset: Reset controller handle for PCIe bridge logic
 *
 * @num_memc:	Number of memory controllers supported by this PCIe instance
 * @memc_size:	Size (in bytes) for each memory controller aperture, indexed by MEMC number
 */
struct brcm_pcie {
	struct udevice		*dev;
	void __iomem		*base;
	u16							bus_base;

	const struct brcm_pcie_cfg_data *cfg;

	int			gen;
	bool			ssc;

	struct reset_ctl	rescal;
	struct reset_ctl	bridge_reset;

	int	num_memc;
	u64	memc_size[BRCM_MAX_MEMC];
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
 * brcm_pcie_perst_set_generic() - Assert or de-assert PERST# for PCIe controller
 * @pcie: Pointer to the PCIe controller state structure
 * @val:  Boolean value indicating whether to assert (1) or de-assert (0) PERST#
 *
 * Controls the PERST# (PCIe reset) signal for generic Broadcom PCIe controllers.
 * Asserts or de-asserts the reset line depending on the given value.
 */
static void brcm_pcie_perst_set_generic(struct brcm_pcie *pcie, u32 val)
{
	if (val)
		setbits_le32(pcie->base + PCIE_RGR1_SW_INIT_1(pcie),
			     PCIE_RGR1_SW_INIT_1_PERST_MASK);
	else
		clrbits_le32(pcie->base + PCIE_RGR1_SW_INIT_1(pcie),
			     PCIE_RGR1_SW_INIT_1_PERST_MASK);
}

/**
 * brcm_pcie_perst_set_2712() - Control PERST# for BCM2712 (Raspberry Pi 5) PCIe controller
 * @pcie: Pointer to the PCIe controller state structure
 * @val:  Boolean value indicating whether to assert (1) or de-assert (0) PERST#
 *
 * On BCM2712, the PERST# control bit has moved and the polarity is inverted.
 * This function sets or clears the PERST# accordingly for this specific SoC.
 */
static void brcm_pcie_perst_set_2712(struct brcm_pcie *pcie, u32 val)
{
	u32 tmp;

	tmp = readl(pcie->base + PCIE_MISC_PCIE_CTRL);
	u32p_replace_bits(&tmp, !val, PCIE_RGR1_SW_INIT_1_PERSTB_MASK);
	writel(tmp, pcie->base + PCIE_MISC_PCIE_CTRL);
}

/**
 * brcm_pcie_bridge_sw_init_set_generic() - Control SW_INIT for PCIe bridge (generic)
 * @pcie: Pointer to the PCIe controller state structure
 * @val:  Boolean value indicating whether to assert (1) or de-assert (0) SW_INIT
 *
 * Asserts or de-asserts the SW_INIT bit in the generic Broadcom PCIe controller,
 * initializing or releasing the PCIe bridge logic as required.
 */
static void brcm_pcie_bridge_sw_init_set_generic(struct brcm_pcie *pcie, u32 val)
{
	if (val)
		setbits_le32(pcie->base + PCIE_RGR1_SW_INIT_1(pcie),
					PCIE_RGR1_SW_INIT_1_INIT_MASK);
	else
		clrbits_le32(pcie->base + PCIE_RGR1_SW_INIT_1(pcie),
					PCIE_RGR1_SW_INIT_1_INIT_MASK);
}

/**
 * brcm_pcie_bridge_sw_init_set_2712() - Control bridge SW_INIT for BCM2712 PCIe controller
 * @pcie: Pointer to the PCIe controller state structure
 * @val:  Boolean value indicating whether to assert (1) or de-assert (0) SW_INIT
 *
 * For the BCM2712 (e.g., Raspberry Pi 5), the bridge SW_INIT control is handled
 * through a dedicated reset controller. This function asserts or de-asserts
 * the reset as needed for bridge initialization.
 */
static void brcm_pcie_bridge_sw_init_set_2712(struct brcm_pcie *pcie, u32 val)
{
	if (val)
		reset_assert(&pcie->bridge_reset);
	else
		reset_deassert(&pcie->bridge_reset);
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
	pci_bus -= pcie->bus_base;
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
	return pci_generic_mmap_write_config(bus, brcm_pcie_config_address,
					     bdf, offset, value, size);
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

/**
 * brcm_pcie_get_resets_dt() - Retrieve reset controls from device tree
 * @dev: U-Boot device pointer
 *
 * Looks up and initializes the reset controls for "rescal" and "bridge"
 * from the device tree, storing the reset handles in the PCIe state struct.
 */
static void brcm_pcie_get_resets_dt(struct udevice *dev)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	int ret;

	ret = reset_get_by_name(dev, "rescal", &pcie->rescal);
	if (ret) {
		printf("Unable to get rescal reset\n");
		return;
	}

	ret = reset_get_by_name(dev, "bridge", &pcie->bridge_reset);
	if (ret) {
		printf("Unable to get bridge reset\n");
		return;
	}
}

/**
 * brcm_pcie_do_reset() - De-assert the rescal reset for PCIe controller
 * @dev: U-Boot device pointer
 *
 * De-asserts the "rescal" reset line to bring the PCIe controller out of reset.
 */
static void brcm_pcie_do_reset(struct udevice *dev)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	int ret;

	ret = reset_deassert(&pcie->rescal);
	if (ret)
		printf("failed to deassert 'rescal'\n");
}

/**
 * brcm_pcie_bar_reg_offset() - Get register offset for RC BAR config
 * @bar: BAR number (1-6)
 *
 * Calculates and returns the register offset for configuring the Root Complex
 * BAR (Base Address Register) for the specified BAR number in the Broadcom
 * PCIe controller.
 *
 * Return: The offset (in bytes) of the BAR config register for the given BAR.
 */
static u32 brcm_pcie_bar_reg_offset(int bar)
{
	if (bar <= 3)
		return PCIE_MISC_RC_BAR1_CONFIG_LO + 8 * (bar - 1);
	else
		return PCIE_MISC_RC_BAR4_CONFIG_LO + 8 * (bar - 4);
}

/**
 * brcm_pcie_ubus_reg_offset() - Get register offset for UBUS BAR remap config
 * @bar: BAR number (1-6)
 *
 * Calculates and returns the register offset for configuring the UBUS BAR
 * remap register for the specified BAR number in the Broadcom PCIe controller.
 * This is used for address remapping between the PCIe bus and the UBUS fabric.
 *
 * Return: The offset (in bytes) of the UBUS BAR remap register for the given BAR.
 */
static u32 brcm_pcie_ubus_reg_offset(int bar)
{
	if (bar <= 3)
		return PCIE_MISC_UBUS_BAR1_CONFIG_REMAP + 8 * (bar - 1);
	else
		return PCIE_MISC_UBUS_BAR4_CONFIG_REMAP + 8 * (bar - 4);
}

/**
 * add_inbound_win() - Add an inbound PCIe window configuration
 * @b:      Pointer to an inbound_win structure to populate
 * @count:  Pointer to the current inbound window count (incremented by this function)
 * @size:   Size of the inbound window in bytes
 * @cpu_addr:  CPU (system physical) address where the window starts
 * @pci_offset: PCIe bus address offset for this window
 *
 * Populates an inbound_win structure with the provided size, CPU address, and
 * PCIe offset, then increments the inbound window count.
 */
static void add_inbound_win(struct inbound_win *b, u8 *count, u64 size,
			    u64 cpu_addr, u64 pci_offset)
{
	b->size = size;
	b->cpu_addr = cpu_addr;
	b->pci_offset = pci_offset;
	(*count)++;
}

/**
 * brcm_pcie_get_inbound_wins_bcm2712() - Get inbound PCIe windows for BCM2712
 * @pcie:         Pointer to the Broadcom PCIe controller structure
 * @inbound_wins: Array of inbound_win structures to populate
 *
 * Retrieves and populates the list of inbound PCIe address windows specific to
 * the Broadcom BCM2712 SoC. These inbound windows define the regions of system
 * memory that can be accessed by PCIe devices via DMA. The function fills out
 * the provided inbound_win array with the relevant window parameters.
 *
 * Return: The number of inbound windows populated, or a negative error code on failure.
 */
static int brcm_pcie_get_inbound_wins_bcm2712(struct brcm_pcie *pcie, struct inbound_win *inbound_wins)
{
	u64 tot_size = 0;
	struct pci_region entry;
	struct udevice *dev = pcie->dev;
	u64 lowest_pci_addr = ~(u64)0;
	int i = 0;
	u8 n = 0;

	/*
	 * The HW registers (and PCIe) use order-1 numbering for BARs.  As such,
	 * we have inbound_wins[0] unused and BAR1 starts at inbound_wins[1].
	 */
	struct inbound_win *b_begin = &inbound_wins[1];
	struct inbound_win *b = b_begin;

	/*
	 * 7712 and newer chips may have many BARs, with each
	 * offering a non-overlapping viewport to system memory.
	 * That being said, each BARs size must still be a power of
	 * two.
	 */
	while (pci_get_dma_regions(dev, &entry, i++) == 0) {
		u64 pci_start = entry.bus_start;
		u64 pci_offset = entry.bus_start - entry.phys_start;
		u64 cpu_start = entry.phys_start;
		u64 size = 1ULL << fls64(entry.size - 1);

		tot_size += size;
		if (pci_start < lowest_pci_addr)
			lowest_pci_addr = pci_start;

		add_inbound_win(b++, &n, size, cpu_start, pci_offset);
	}

	if (lowest_pci_addr == ~(u64)0) {
		printf("DT node has no dma-ranges\n");
		return -EINVAL;
	}

	return n;
}

/**
 * brcm_pcie_get_inbound_wins_generic() - Get and configure inbound PCIe windows for generic Broadcom SoCs
 * @pcie:         Pointer to the Broadcom PCIe controller structure
 * @inbound_wins: Array of inbound_win structures to populate
 *
 * Sets up the inbound PCIe address windows for generic (non-BCM2712) Broadcom
 * PCIe controllers. This function:
 *   - Disables inbound window 1 (which on legacy STB chips was mapped to internal SoC registers, not RAM; this feature is deprecated for security reasons).
 *   - Enables inbound window 2, the primary inbound window for DMA from PCIe devices into system memory. The window size is rounded up to the next power of two, and address translation is set based on the region's physical and bus addresses.
 *   - Disables inbound window 3, which on some chips provides an alternative access path with selectable endianness.
 *
 * The function populates the provided inbound_win array with these window configurations and updates the PCIe controller's memory controller count and size.
 *
 * Return: The total number of inbound windows configured.
 */
static int brcm_pcie_get_inbound_wins_generic(struct brcm_pcie *pcie, struct inbound_win *inbound_wins)
{
	struct udevice *dev = pcie->dev;
	struct pci_region region;
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
	add_inbound_win(b++, &n, 0, 0, 0);

	/* Enable inbound window 2, the main inbound window for STB chips */
	pci_get_dma_regions(dev, &region, 0);
	u64 win2_size = 1ULL << fls64(region.size - 1);
	u64 win2_pcie_offset = region.bus_start - region.phys_start;
	add_inbound_win(b++, &n, win2_size, region.phys_start, win2_pcie_offset);
	pcie->num_memc = 1;
	pcie->memc_size[0] = region.size;

	/*
	 * Disable inbound window 3. On some chips presents the same
	 * window as #2 but the data appears in a settable endianness.
	 */
	add_inbound_win(b++, &n, 0, 0, 0);

	return n;
}

/**
 * brcm_set_inbound_win_registers() - Program inbound PCIe window registers
 * @pcie:             Pointer to the Broadcom PCIe controller structure
 * @inbound_wins:     Array of inbound_win structures to configure
 * @num_inbound_wins: Number of inbound windows to program
 *
 * This function loops through all provided inbound windows (starting at index 1)
 * and writes the required values to the hardware registers for proper inbound
 * DMA operation by PCIe devices.
 */
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
		u32 reg_offset = brcm_pcie_bar_reg_offset(i);
		u32 tmp = lower_32_bits(pci_offset);

		u32p_replace_bits(&tmp, brcm_pcie_encode_ibar_size(size), RC_BAR1_CONFIG_LO_SIZE_MASK);

		/* Write low */
		writel(tmp, base + reg_offset);
		/* Write high */
		writel(upper_32_bits(pci_offset), base + reg_offset + 4);

		/*
		 * Most STB chips:
		 *     Do nothing.
		 * 7712:
		 *     All of their BARs need to be set.
		 */
		if (pcie->cfg->type == BRCM_PCIE_BCM2712) {
			/* BUS remap register settings */
			reg_offset = brcm_pcie_ubus_reg_offset(i);
			tmp = lower_32_bits(cpu_addr) & ~0xfff;
			tmp |= PCIE_MISC_UBUS_BAR1_CONFIG_REMAP_ACCESS_EN_MASK;
			writel(tmp, base + reg_offset);
			tmp = upper_32_bits(cpu_addr);
			writel(tmp, base + reg_offset + 4);
		}
	}
}

/**
 * brcm_pcie_post_setup_bcm2712() - Perform post-setup configuration for BCM2712 PCIe controller
 * @pcie: Pointer to the Broadcom PCIe controller structure
 *
 * Performs final hardware configuration for the BCM2712 PCIe controller after initial setup.
 *
 * Return: 0 on success, or a negative error code if register programming fails.
 */
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

static int brcm_pcie_probe(struct udevice *dev)
{
	struct udevice *ctlr = pci_get_controller(dev);
	struct pci_controller *hose = dev_get_uclass_priv(ctlr);
	struct brcm_pcie *pcie = dev_get_priv(dev);
	void __iomem *base = pcie->base;
	bool ssc_good = false;
	struct inbound_win inbound_wins[BRCM_MAX_INBOUND_WINS];
	int num_inbound_wins = 0;
	int num_out_wins = 0;
	unsigned int scb_size_val;
	int i, ret;
	u16 nlw, cls, lnksta;
	u32 tmp;

	pcie->dev = dev;
	pcie->bus_base = hose->first_busno;

	/*
	 * Deassert rescal reset if present.
	 */
	if(pcie->rescal.dev)
		brcm_pcie_do_reset(dev);

	/*
	 * Reset the bridge, assert the fundamental reset. Note for some SoCs,
	 * e.g. BCM7278, the fundamental reset should not be asserted here.
	 */
	pcie->cfg->bridge_sw_init_set(pcie, 1);
	if (pcie->cfg->type != BRCM_PCIE_BCM2712)
		pcie->cfg->perst_set(pcie, 1);

	/*
	 * The delay is a safety precaution to preclude the reset signal
	 * from looking like a glitch.
	 */
	udelay(100);

	/* Take the bridge out of reset */
	pcie->cfg->bridge_sw_init_set(pcie, 0);

	clrbits_le32(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG(pcie),
		     PCIE_HARD_DEBUG_SERDES_IDDQ_MASK);

	/* Wait for SerDes to be stable */
	udelay(100);

	/* Set SCB_MAX_BURST_SIZE, CFG_READ_UR_MODE, SCB_ACCESS_EN */
	clrsetbits_le32(base + PCIE_MISC_MISC_CTRL,
			MISC_CTRL_MAX_BURST_SIZE_MASK,
			MISC_CTRL_SCB_ACCESS_EN_MASK |
			MISC_CTRL_CFG_READ_UR_MODE_MASK |
			MISC_CTRL_MAX_BURST_SIZE_128(pcie) |
			MISC_CTRL_PCIE_RCB_MPS_MODE_MASK
	);

	/* Mask all interrupts since we are not handling any yet */
	writel(0xffffffff, base + PCIE_MSI_INTR2_MASK_SET);

	/* Clear any interrupts we find on boot */
	writel(0xffffffff, base + PCIE_MSI_INTR2_CLR);

	if (pcie->gen)
		brcm_pcie_set_gen(pcie, pcie->gen);

	/* Unassert the fundamental reset */
	pcie->cfg->perst_set(pcie, 0);

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

	if (!pcie->cfg->rc_mode(pcie)) {
		printf("PCIe misconfigured; is in EP mode\n");
		return -EINVAL;
	}

	/*
	 * Inbound window setup
	 */
	num_inbound_wins = pcie->cfg->get_inbound_wins(pcie, inbound_wins);
	if (num_inbound_wins < 0)
		return num_inbound_wins;
	brcm_set_inbound_win_registers(pcie, inbound_wins, num_inbound_wins+1);

	tmp = readl(base + PCIE_MISC_MISC_CTRL);
	for (int memc = 0; memc < pcie->num_memc; memc++) {
		scb_size_val = ilog2(pcie->memc_size[memc]) - 15;

		if (memc == 0)
			u32p_replace_bits(&tmp, scb_size_val, PCIE_MISC_MISC_CTRL_SCB0_SIZE_MASK);
		else if (memc == 1)
			u32p_replace_bits(&tmp, scb_size_val, PCIE_MISC_MISC_CTRL_SCB1_SIZE_MASK);
		else if (memc == 2)
			u32p_replace_bits(&tmp, scb_size_val, PCIE_MISC_MISC_CTRL_SCB2_SIZE_MASK);
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

	if (pcie->cfg->post_setup)
		return pcie->cfg->post_setup(pcie);

	return 0;
}

static int brcm_pcie_remove(struct udevice *dev)
{
	struct brcm_pcie *pcie = dev_get_priv(dev);
	void __iomem *base = pcie->base;

	/* Assert fundamental reset */
	pcie->cfg->perst_set(pcie, 1);

	/* Turn off SerDes */
	setbits_le32(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG(pcie),
		     PCIE_HARD_DEBUG_SERDES_IDDQ_MASK);

	/* Shutdown bridge */
	pcie->cfg->bridge_sw_init_set(pcie, 1);

	/*
	 * For the controllers that are utilizing reset for bridge Sw init,
	 * such as BCM2712, reset should be deasserted after assertion.
	 * Leaving it in asserted state may lead to unexpected hangs in
	 * the Linux Kernel driver because it do not perform reset initialization
	 * and start accessing device memory.
	 */
	if (pcie->cfg->type == BRCM_PCIE_BCM2712)
		pcie->cfg->bridge_sw_init_set(pcie, 0);

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

	pcie->cfg = (const struct brcm_pcie_cfg_data *)dev_get_driver_data(dev);

	if (pcie->cfg->type == BRCM_PCIE_BCM2712)
		brcm_pcie_get_resets_dt(dev);

	return 0;
}

static const struct dm_pci_ops brcm_pcie_ops = {
	.read_config	= brcm_pcie_read_config,
	.write_config	= brcm_pcie_write_config,
};

static const int brcm_pcie_offsets[] = {
	[RGR1_SW_INIT_1]     = 0x9210,
	[PCIE_HARD_DEBUG]    = 0x4204,
	[MAX_BURST_SIZE_128] = 0x0,
};

static const struct brcm_pcie_cfg_data brcm_pcie_bcm2711_cfg = {
	.offsets   = brcm_pcie_offsets,
	.type               = BRCM_PCIE_BCM_GENERIC,
	.perst_set          = brcm_pcie_perst_set_generic,
	.bridge_sw_init_set = brcm_pcie_bridge_sw_init_set_generic,
	.rc_mode            = brcm_pcie_rc_mode,
	.get_inbound_wins   = brcm_pcie_get_inbound_wins_generic,
};

static const int brcm_pcie_offsets_bcm2712[] = {
	[RGR1_SW_INIT_1]     = 0x0,
	[PCIE_HARD_DEBUG]    = 0x4304,
	[MAX_BURST_SIZE_128] = 0x100000,
};

static const struct brcm_pcie_cfg_data brcm_pcie_bcm2712_cfg = {
	.offsets            = brcm_pcie_offsets_bcm2712,
	.type               = BRCM_PCIE_BCM2712,
	.perst_set          = brcm_pcie_perst_set_2712,
	.bridge_sw_init_set = brcm_pcie_bridge_sw_init_set_2712,
	.rc_mode            = brcm_pcie_rc_mode,
	.post_setup         = brcm_pcie_post_setup_bcm2712,
	.get_inbound_wins   = brcm_pcie_get_inbound_wins_bcm2712,
};

static const struct udevice_id brcm_pcie_ids[] = {
	{ .compatible = "brcm,bcm2711-pcie", .data = (ulong)&brcm_pcie_bcm2711_cfg },
	{ .compatible = "brcm,bcm2712-pcie", .data = (ulong)&brcm_pcie_bcm2712_cfg },
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
