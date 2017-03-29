/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2012 Advanced Micro Devices, Inc.
 * Copyright (C) 2017 Damien Zammit <damien@zamaudio.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <lib.h>
#include <arch/acpi.h>
#include <arch/cpu.h>
#include <arch/io.h>
#include <arch/stages.h>
#include <cbmem.h>
#include <console/console.h>
#include <cpu/amd/car.h>
#include <cpu/x86/bist.h>
#include <cpu/x86/lapic.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <device/pnp_def.h>
#include <spd.h>
#include <cpu/amd/model_10xxx_rev.h>
#include <northbridge/amd/amdfam10/raminit.h>
#include <northbridge/amd/amdfam10/amdfam10.h>
#include <northbridge/amd/amdfam10/early_ht.h>
#include <northbridge/amd/amdht/ht_wrapper.h>
#include <cpu/amd/family_10h-family_15h/init_cpus.h>
#include <delay.h>
#include <northbridge/amd/amdfam10/reset_test.h>
#include <stdint.h>
#include <string.h>
#include <superio/ite/common/ite.h>
#include <superio/ite/it8728f/it8728f.h>
#include <southbridge/amd/sb900/sb900.h>
#include <southbridge/amd/sb900/smbus.h>
#include <arch/early_variables.h>

#define MMIO_NON_POSTED_START 0xfed00000
#define MMIO_NON_POSTED_END   0xfedfffff
#define SB_MMIO 0xFED80000
#define SB_MMIO_MISC32(x) *(volatile u32 *)(SB_MMIO + 0xE00 + (x))

#define SERIAL_DEV PNP_DEV(0x2e, IT8728F_SP1)
#define GPIO_DEV PNP_DEV(0x2e, IT8728F_GPIO)
#define CLKIN_DEV PNP_DEV(0x2e, IT8728F_GPIO)

#define SMBUS_IO_BASE 0xb00
#define SMBUS_AUX_IO_BASE 0xb20

void activate_spd_rom(const struct mem_controller *ctrl);
int spd_read_byte(unsigned device, unsigned address);
extern struct sys_info sysinfo_car;

static void sbxxx_enable_48mhzout(void)
{
	/* most likely programming to 48MHz out signal */
	/* Set auxiliary output clock frequency on OSCOUT1 pin to be 48MHz */
	u32 reg32;
	reg32 = SB_MMIO_MISC32(0x28);
	//reg32 &= 0xffc7ffff;
	//reg32 |= 0x00100000;
	reg32 &= 0xfff8ffff;
	SB_MMIO_MISC32(0x28) = reg32;

	/* Enable Auxiliary Clock1, disable FCH 14 MHz OscClk */
	reg32 = SB_MMIO_MISC32(0x40);
	//reg32 &= ~0x80u;
	reg32 &= 0xffffbffb;
	SB_MMIO_MISC32(0x40) = reg32;
}

static __inline__ __attribute__((always_inline)) void wrmsr_amd (u32 msr, u64 value)
{
	__asm__ __volatile__ (
		"wrmsr"
		:
		: "c" (msr), "A" (value)
	);
}

static void writepmreg (int reg, int data)
{
	outb(reg, 0xCD6);
	outb(data, 0xCD7);
}

void activate_spd_rom(const struct mem_controller *ctrl)
{
	u32 iobase = SMBUS_IO_BASE;

	writepmreg (0x2D, iobase >> 8);
	writepmreg (0x2C, iobase | 1);
	/* Set SMBUS clock to 400KHz */
	outb(66000000 / 400000 / 4, iobase + 0x0E);
}

inline int spd_read_byte(unsigned device, unsigned address)
{
	return do_smbus_read_byte(SMBUS_IO_BASE, device, address);
}

static const uint8_t spd_addr_fam15[] = {
	// Socket 0 Node 0 ("Node 0")
	RC00, DIMM0, DIMM1, 0, 0, 0, 0, 0, 0,
};

#include "cpu/amd/quadcore/quadcore.c"

static void set_ddr3_voltage(uint8_t node, uint8_t index) {
	uint8_t byte;
	uint8_t value = 0;

	if (index == 0)
		value = 0x0;
	else if (index == 1)
		value = 0x1;
	else if (index == 2)
		value = 0x4;
	else if (index == 3)
		value = 0x5;
	if (node == 1)
		value <<= 1;

	/* Set GPIOs */
	byte = pci_read_config8(PCI_DEV(0, 0x14, 3), 0xd1);
	if (node == 0)
		byte &= ~0x5;
	if (node == 1)
		byte &= ~0xa;
	byte |= value;
	pci_write_config8(PCI_DEV(0, 0x14, 3), 0xd1, byte);

	/* Enable GPIO output drivers */
	byte = pci_read_config8(PCI_DEV(0, 0x14, 3), 0xd0);
	byte &= 0x0f;
	pci_write_config8(PCI_DEV(0, 0x14, 3), 0xd0, byte);
}

void DIMMSetVoltages(struct MCTStatStruc *pMCTstat,
				struct DCTStatStruc *pDCTstatA) {
	u8 dimm;
	u8 socket = 0;
	u8 set_voltage = 0x1;

	set_ddr3_voltage(socket, set_voltage);

	/* Save 1.5V DIMM voltages for MCT and SMBIOS use */
	for (dimm = 0; dimm < MAX_DIMMS_SUPPORTED; dimm++) {
		pDCTstatA->DimmConfiguredVoltage[dimm] = set_voltage;
	}
	/* Allow the DDR supply voltages to settle */
	udelay(100000);
	printk(BIOS_DEBUG, "DIMM voltage set to index %02x\n", set_voltage);
}

unsigned int get_sbdn(unsigned int bus)
{
	device_t dev;

	dev = PCI_DEV(0, 0x14, 0);
	return (dev >> 15) & 0x1f;
}

void soft_reset(void)
{
}

static u8 bitscanr (u32 value)
{
	return (u8)log2(value);
}

static void amd_initmmio(void)
{
	u64 msrboth;

	/*
	  Set the MMIO Configuration Base Address and Bus Range onto MMIO configuration base
	  Address MSR register.
	*/
	msrboth = CONFIG_MMCONF_BASE_ADDRESS | (bitscanr (CONFIG_MMCONF_BUS_NUMBER) << 2) | 1;
	wrmsr_amd (0xC0010058, msrboth);

	/* Set ROM cache onto WP to decrease post time */
	msrboth = (0x0100000000ull - CACHE_ROM_SIZE) | 5ull;
	wrmsr_amd (0x20C, msrboth);
	msrboth = ((1ULL << CONFIG_CPU_ADDR_BITS) - CACHE_ROM_SIZE) | 0x800ull;
	wrmsr_amd (0x20D, msrboth);
}

#if 0
static void amd_initcpuio(void)
{
	msr_t MsrReg;
	u64 msrboth;
	u32 PciData;
	device_t PciAddress;

	/* Enable legacy video routing: D18F1xF4 VGA Enable */
	PciAddress = PCI_DEV(0, 0x18, 1);
	pci_write_config32(PciAddress, 0xF4, 1);

	/* The platform BIOS needs to ensure the memory ranges of SB800 legacy
	 * devices (TPM, HPET, BIOS RAM, Watchdog Timer, I/O APIC and ACPI) are
	 * set to non-posted regions.
	 */
	PciData = 0x00FEDF00; /* last address before processor local APIC at FEE00000 */
	PciData |= 1 << 7;    /* set NP (non-posted) bit */
	pci_write_config32(PciAddress, 0x84, PciData);

	PciData = (0xFED00000 >> 8) | 3; /* lowest NP address is HPET at FED00000 */
	pci_write_config32(PciAddress, 0x80, PciData);

	/* Map the remaining PCI hole as posted MMIO */
	PciData = 0x00FECF00; /* last address before non-posted range */
	pci_write_config32(PciAddress, 0x8C, PciData);

	MsrReg = rdmsr(0xC001001A);
	msrboth = ((u64)MsrReg.hi << 32) | MsrReg.lo;
	msrboth = ((msrboth >> 8) | 3) & 0xffffffff;
	pci_write_config32(PciAddress, 0x88, (u32)msrboth);

	/* Send all IO (0000-FFFF) to southbridge. */
	PciData = 0x0000F000;
	pci_write_config32(PciAddress, 0xC4, PciData);
	PciData = 0x00000003;
	pci_write_config32(PciAddress, 0xC0, PciData);
}
#endif

void cache_as_ram_main(unsigned long bist, unsigned long cpu_init_detectedx)
{
	u32 val;
	u8 byte;
	device_t dev;
	uint32_t bsp_apicid = 0;
	msr_t msr;

	struct sys_info *sysinfo = &sysinfo_car;

	/* Limit the maximum HT speed to 2.6GHz to prevent lockups
	* due to HT CPU <--> CPU wiring not being validated to 3.2GHz
	*/
	sysinfo->ht_link_cfg.ht_speed_limit = 2600;

	sb900_pci_port80();

	amd_initmmio();
	//amd_initcpuio();

	/* Set LPC decode enables. */
	dev = PCI_DEV(0, 0x14, 3);
	pci_write_config32(dev, 0x44, 0xff03ffd5);

	if (!cpu_init_detectedx && boot_cpu()) {
		/* enable SIO LPC decode */
		byte = pci_read_config8(dev, 0x48);
		byte |= 3;	/* 2e, 2f */
		pci_write_config8(dev, 0x48, byte);

		/* enable serial decode */
		byte = pci_read_config8(dev, 0x44);
		byte |= (1 << 6);  /* 0x3f8 */
		pci_write_config8(dev, 0x44, byte);

		post_code(0x30);

		/* Enable the AcpiMmio space */
		outb(0x24, 0xcd6);
		outb(0x1, 0xcd7);

		/* run ite */
		sbxxx_enable_48mhzout();
		ite_conf_clkin(CLKIN_DEV, ITE_UART_CLK_PREDIVIDE_48);
		ite_kill_watchdog(GPIO_DEV);
		ite_enable_serial(SERIAL_DEV, CONFIG_TTYS0_BASE);

		console_init();

		/* turn on secondary smbus at b20 */
		outb(0x28, 0xcd6);
		byte = inb(0xcd7);
		byte |= 1;
		outb(byte, 0xcd7);
	}
	printk(BIOS_DEBUG, "Console inited!\n");

	if (bist == 0)
		bsp_apicid = init_cpus(cpu_init_detectedx, sysinfo);

	post_code(0x34);
	/* Halt if there was a built in self test failure */
	report_bist_failure(bist);

	/* Load MPB */
	val = cpuid_eax(1);
	printk(BIOS_DEBUG, "BSP Family_Model: %08x\n", val);
	printk(BIOS_DEBUG, "cpu_init_detectedx = %08lx\n", cpu_init_detectedx);

	/* On Larne, after LpcClkDrvSth is set, it needs some time to be stable, because of the buffer ICS551M */
	int i;
	for(i = 0; i < 200000; i++)
		val = inb(0xcd6);

	set_sysinfo_in_ram(0);

	//update_microcode(val);

	cpuSetAMDMSR(0);

	/* Pass NULL to this function to skip */
	amd_ht_init(NULL);
	//amd_ht_fixup(sysinfo);

	post_code(0x35);

	finalize_node_setup(sysinfo);
	//setup_mb_resource_map();
	post_code(0x36);

	/* Wait for all the APs core0 started by finalize_node_setup. */
	wait_all_core0_started();

	if (IS_ENABLED(CONFIG_LOGICAL_CPUS)) {
		/* Core0 on each node is configured. Now setup any additional cores. */
		printk(BIOS_DEBUG, "start_other_cores()\n");
		start_other_cores(bsp_apicid);
		post_code(0x37);
		wait_all_other_cores_started(bsp_apicid);
	}

	if (IS_ENABLED(CONFIG_SET_FIDVID)) {
		msr = rdmsr(0xc0010071);
		printk(BIOS_DEBUG, "\nBegin FIDVID MSR 0xc0010071 0x%08x 0x%08x\n", msr.hi, msr.lo);

		/* FIXME: The sb fid change may survive the warm reset and only need to be done once */
		//enable_fid_change_on_sb(sysinfo->sbbusn, sysinfo->sbdn);

		post_code(0x38);

		if (!warm_reset_detect(0)) {	// BSP is node 0
			init_fidvid_bsp(bsp_apicid, sysinfo->nodes);
		} else {
			init_fidvid_stage2(bsp_apicid, 0);	// BSP is node 0
		}

		post_code(0x39);

		/* show final fid and vid */
		msr=rdmsr(0xc0010071);
		printk(BIOS_DEBUG, "End FIDVIDMSR 0xc0010071 0x%08x 0x%08x\n", msr.hi, msr.lo);
	}

	post_code(0x3a);

	set_ddr3_voltage(0, 1);

	post_code(0x3b);

	///* Set up peripheral control lines */
	//set_peripheral_control_lines();
	fill_mem_ctrl(sysinfo->nodes, sysinfo->ctrl, spd_addr_fam15);

	post_code(0x40);
	printk(BIOS_DEBUG, "do raminit...");
	raminit_amdmct(sysinfo);
	printk(BIOS_DEBUG, "done\n");
	post_code(0x41);

	//execute_memory_test();

	post_code(0x42);

	cbmem_initialize_empty();

	amdmct_cbmem_store_info(sysinfo);

	post_cache_as_ram();

	post_code(0xee);  /* Should never see this post code. */
}

BOOL AMD_CB_ManualBUIDSwapList (u8 node, u8 link, const u8 **List)
{
	/* Force BUID to 0 */
	static const u8 swaplist[] = { 0xFF, CONFIG_HT_CHAIN_UNITID_BASE,
				CONFIG_HT_CHAIN_END_UNITID_BASE, 0xFF };
	if ((node == 0) && (link == 0)) {       /* BSP SB link */
		*List = swaplist;
		return 1;
	}

	return 0;
}
