/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2012 Advanced Micro Devices, Inc.
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

#include <stdint.h>
#include <string.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <arch/acpi.h>
#include <arch/io.h>
#include <arch/stages.h>
#include <device/pnp_def.h>
#include <arch/cpu.h>
#include <cpu/x86/lapic.h>
#include <console/console.h>
#include <commonlib/loglevel.h>
#include <cpu/amd/car.h>
#include <northbridge/amd/agesa/agesawrapper.h>
#include <cpu/x86/bist.h>
#include <cpu/x86/lapic.h>
#include <southbridge/amd/agesa/hudson/hudson.h>
#include <cpu/amd/agesa/s3_resume.h>
#include "cbmem.h"

#include <superio/ite/common/ite.h>
#include <superio/ite/it8728f/it8728f.h>

//#define MMIO_NON_POSTED_START 0xfed00000
//#define MMIO_NON_POSTED_END   0xfedfffff
#define SB_MMIO 0xFED80000
#define SB_MMIO_MISC32(x) *(volatile u32 *)(SB_MMIO + 0xE00 + (x))

#define SERIAL_DEV PNP_DEV(0x2e, IT8728F_SP1)
#define GPIO_DEV PNP_DEV(0x2e, IT8728F_GPIO)
#define CLKIN_DEV PNP_DEV(0x2e, IT8728F_GPIO)

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

void cache_as_ram_main(unsigned long bist, unsigned long cpu_init_detectedx)
{
	u32 val;
	u8 byte;
	pci_devfn_t dev;

#if IS_ENABLED(CONFIG_POST_DEVICE_PCI_PCIE)
	hudson_pci_port80();
#endif
#if IS_ENABLED(CONFIG_POST_DEVICE_LPC)
	hudson_lpc_port80();
#endif

	/* In Hudson RRG, PMIOxD2[5:4] is "Drive strength control for
	 *  LpcClk[1:0]".  To be consistent with Parmer, setting to 4mA
	 *  even though the register is not documented in the Kabini BKDG.
	 *  Otherwise the serial output is bad code.
	 */
	//outb(0xD2, 0xcd6);
	//outb(0x00, 0xcd7);

	amd_initmmio();

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

	/* Halt if there was a built in self test failure */
	post_code(0x34);
	report_bist_failure(bist);

	/* Load MPB */
	val = cpuid_eax(1);
	printk(BIOS_DEBUG, "BSP Family_Model: %08x\n", val);
	printk(BIOS_DEBUG, "cpu_init_detectedx = %08lx\n", cpu_init_detectedx);

	/* On Larne, after LpcClkDrvSth is set, it needs some time to be stable, because of the buffer ICS551M */
	int i;
	for(i = 0; i < 200000; i++)
		val = inb(0xcd6);

	post_code(0x37);
	agesawrapper_amdinitreset();
	post_code(0x38);
	printk(BIOS_DEBUG, "Got past yangtze_early_setup\n");

	post_code(0x39);

	agesawrapper_amdinitearly();
	int s3resume = acpi_is_wakeup_s3();
	if (!s3resume) {
		post_code(0x40);
		agesawrapper_amdinitpost();
		post_code(0x41);
		agesawrapper_amdinitenv();
		/* TODO: Disable cache is not ok. */
		disable_cache_as_ram();
	} else { /* S3 detect */
		printk(BIOS_INFO, "S3 detected\n");

		post_code(0x60);
		agesawrapper_amdinitresume();

		amd_initcpuio();
		agesawrapper_amds3laterestore();

		post_code(0x61);
		prepare_for_resume();
	}

	outb(0xEA, 0xCD6);
	outb(0x1, 0xcd7);

	post_code(0x50);
	copy_and_run();

	post_code(0x54);  /* Should never see this post code. */
}
