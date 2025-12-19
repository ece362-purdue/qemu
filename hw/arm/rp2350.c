/*
 * RP2350 SoC
 *
 * Copyright (c) 2025 Niraj Menon <niraj@purdue.edu>
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h" 
#include "hw/sysbus.h"
#include "hw/arm/boot.h"
#include "hw/arm/armv7m.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "hw/char/pl011.h"
#include "hw/misc/unimp.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/arm/machines-qom.h"
#include "qom/object.h"
#include "hw/qdev-clock.h"
#include "hw/loader.h"

/* RP2350 Memory Map */
#define RP2350_BOOTROM_BASE     0x00000000
#define RP2350_BOOTROM_SIZE     (32 * KiB) 

#define RP2350_FLASH_BASE       0x10000000
#define RP2350_FLASH_SIZE       (16 * MiB)

#define RP2350_SRAM_BASE        0x20000000
#define RP2350_SRAM_SIZE        (520 * KiB)

#define RP2350_UART0_BASE       0x40034000
#define RP2350_UART0_IRQ        20

/* QOM Declaration */
#define TYPE_RP2350_MACHINE MACHINE_TYPE_NAME("rp2350")
OBJECT_DECLARE_SIMPLE_TYPE(RP2350State, RP2350_MACHINE)

struct RP2350State {
    MachineState parent;
    ARMv7MState cpu;  
    MemoryRegion bootrom;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion flash_alias; 
};

/* Minimal dummy timer to prevent sleep_ms hang */
static uint64_t dummy_timer_read(void *opaque, hwaddr offset, unsigned size) {
    static uint64_t time = 0;
    time += 1000; 
    return time;
}

static const MemoryRegionOps dummy_timer_ops = {
    .read = dummy_timer_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void rp2350_init(MachineState *machine)
{
    RP2350State *s = RP2350_MACHINE(machine);

    /* 1. Initialize Memory Regions */
    memory_region_init_ram(&s->sram, NULL, "rp2350.sram", RP2350_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RP2350_SRAM_BASE, &s->sram);

    memory_region_init_rom(&s->flash, NULL, "rp2350.flash", RP2350_FLASH_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RP2350_FLASH_BASE, &s->flash);

    /* BootROM / Flash Alias Logic */
    if (machine->firmware) {
        memory_region_init_rom(&s->bootrom, NULL, "rp2350.bootrom", RP2350_BOOTROM_SIZE, &error_fatal);
        memory_region_add_subregion(get_system_memory(), RP2350_BOOTROM_BASE, &s->bootrom);
        
        int image_size = load_image_targphys(machine->firmware, RP2350_BOOTROM_BASE, RP2350_BOOTROM_SIZE, NULL);
        if (image_size < 0) {
            error_report("Could not load BootROM image '%s'", machine->firmware);
            exit(1);
        }
    } else {
        memory_region_init_alias(&s->flash_alias, NULL, "rp2350.flash.boot", 
                                 &s->flash, 0, RP2350_FLASH_SIZE);
        memory_region_add_subregion(get_system_memory(), RP2350_BOOTROM_BASE, &s->flash_alias);
    }

    /* 2. Initialize CPU */
    object_initialize_child(OBJECT(machine), "cpu", &s->cpu, TYPE_ARMV7M);
    qdev_prop_set_string(DEVICE(&s->cpu), "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    qdev_prop_set_bit(DEVICE(&s->cpu), "enable-bitband", true);

    /* Create a 150MHz System Clock */
    Clock *sysclk = clock_new(OBJECT(machine), "sysclk");
    clock_set_hz(sysclk, 150000000); // 150 MHz
    
    /* Connect Clock to CPU */
    qdev_connect_clock_in(DEVICE(&s->cpu), "cpuclk", sysclk);
    object_property_set_link(OBJECT(&s->cpu), "memory", OBJECT(get_system_memory()), &error_abort);
    
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpu), &error_fatal)) {
        return;
    }

    /* 3. Peripherals - FIXED UART CLOCKING */
    /* Instead of sysbus_create_simple, we do it manually to connect the clock */
    DeviceState *pl011 = qdev_new("pl011");
    
    /* Connect the serial device (stdio, file, socket, etc.) */
    Chardev *chr = serial_hd(0);
    qdev_prop_set_chr(pl011, "chardev", chr);
    
    /* VITAL: Connect the clock! Without this, UART is 0Hz and dead. */
    qdev_connect_clock_in(pl011, "clk", sysclk);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pl011), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pl011), 0, RP2350_UART0_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(pl011), 0, qdev_get_gpio_in(DEVICE(&s->cpu), RP2350_UART0_IRQ));

    /* Dummy Timer */
    MemoryRegion *timer_mr = g_new(MemoryRegion, 1);
    memory_region_init_io(timer_mr, NULL, &dummy_timer_ops, NULL, "rp2350.timer", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x40054000, timer_mr);

    /* 4. Unimplemented Devices */
    create_unimplemented_device("rp2350.sysinfo", 0x40000000, 0x4000); 
    create_unimplemented_device("rp2350.clocks",  0x40008000, 0x10000);
    create_unimplemented_device("rp2350.io_bank0", 0x40014000, 0x10000);

    /* 5. Load Firmware */
    armv7m_load_kernel(s->cpu.cpu, machine->kernel_filename, 0, RP2350_FLASH_SIZE);
}

static void rp2350_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Raspberry Pi RP2350 (Cortex-M33)";
    mc->init = rp2350_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    mc->ignore_memory_transaction_failures = true; 
}

static const TypeInfo rp2350_info = {
    .name = TYPE_RP2350_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(RP2350State),
    .class_init = rp2350_class_init,
    .interfaces = arm_machine_interfaces,
};

static void rp2350_machine_init(void)
{
    type_register_static(&rp2350_info);
}

type_init(rp2350_machine_init);