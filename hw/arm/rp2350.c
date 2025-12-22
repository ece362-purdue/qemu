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

/* RP2350 SIO FIFO Implementation (core0<->core1 mailbox) */
#include "qemu/log.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "target/arm/idau.h"

/* RP2350 Memory Map */
#define RP2350_BOOTROM_BASE     0x00000000
#define RP2350_BOOTROM_SIZE     (32 * KiB) 

#define RP2350_FLASH_BASE       0x10000000
#define RP2350_FLASH_SIZE       (16 * MiB)

#define RP2350_SRAM_BASE        0x20000000
#define RP2350_SRAM_SIZE        (520 * KiB)

#define RP2350_BOOTRAM_BASE     0x400e0000
#define RP2350_BOOTRAM_SIZE     (1 * KiB)  /* 0x400e0000 - 0x400e0400 */

/* USB DPRAM - used by bootrom as stack during NS code execution */
#define RP2350_USBDPRAM_NS_BASE  0x40100000
#define RP2350_USBDPRAM_S_BASE   0x50100000
#define RP2350_USBDPRAM_SIZE     (4 * KiB)

#define RP2350_UART0_BASE       0x40070000
#define RP2350_UART0_IRQ        33

/* SIO */
#define RP2350_SIO_BASE         0xd0000000
#define RP2350_SIO_SIZE         0x1000
#define RP2350_CPUS             2

/* Minimal IDAU for BootROM bring-up (implemented in hw/misc/rp2350-idau.c). */
#define TYPE_RP2350_IDAU "rp2350-idau"

typedef struct RP2350FIFOState {
    MemoryRegion iomem;
    ARMv7MState *core1;

    /* FIFO State for Multicore Launch */
    int launch_state;
    uint32_t vector_table;
    uint32_t sp;
    uint32_t entry;

    /* Mailboxes: TX core0->core1, RX core1->core0 */
    uint32_t tx_fifo[4];
    uint32_t rx_fifo[4];
    int tx_r, tx_w, tx_count;
    int rx_r, rx_w, rx_count;
    bool roe;
    bool wof;
} RP2350FIFOState;

static inline void fifo_reset(RP2350FIFOState *s)
{
    s->tx_r = s->tx_w = s->tx_count = 0;
    s->rx_r = s->rx_w = s->rx_count = 0;
    s->roe = false;
    s->wof = false;
}

static inline void fifo_rx_push(RP2350FIFOState *s, uint32_t v)
{
    if (s->rx_count == 4) {
        s->wof = true;
        return;
    }
    s->rx_fifo[s->rx_w] = v;
    s->rx_w = (s->rx_w + 1) & 3;
    s->rx_count++;
}

static inline uint32_t fifo_rx_pop(RP2350FIFOState *s, bool *ok)
{
    if (s->rx_count == 0) {
        s->roe = true;
        *ok = false;
        return 0;
    }
    uint32_t v = s->rx_fifo[s->rx_r];
    s->rx_r = (s->rx_r + 1) & 3;
    s->rx_count--;
    *ok = true;
    return v;
}

static inline void fifo_tx_push(RP2350FIFOState *s, uint32_t v)
{
    if (s->tx_count == 4) {
        s->wof = true;
        return;
    }
    s->tx_fifo[s->tx_w] = v;
    s->tx_w = (s->tx_w + 1) & 3;
    s->tx_count++;
}

static uint64_t rp2350_fifo_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350FIFOState *s = opaque;
    uint64_t val = 0;

    switch (addr) {
    case 0x00: /* CPUID */
        val = current_cpu ? current_cpu->cpu_index : 0;
        break;
    case 0x50: /* FIFO_ST */
        if (s->rx_count > 0) {
            val |= 1; /* VLD */
        }
        if (s->tx_count < 4) {
            val |= (1 << 1); /* RDY */
        }
        if (s->wof) {
            val |= (1 << 2);
        }
        if (s->roe) {
            val |= (1 << 3);
        }
        break;
    case 0x58: { /* FIFO_RD */
        bool ok;
        val = fifo_rx_pop(s, &ok);
        break;
    }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad read offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }
    return val;
}

static void rp2350_fifo_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    RP2350FIFOState *s = opaque;

    switch (addr) {
    case 0x50: /* FIFO_ST (write clears ROE/WOF) */
        s->roe = false;
        s->wof = false;
        break;
    case 0x54: /* FIFO_WR */
        fifo_tx_push(s, val);
        if (s->tx_count > 0) {
            /* BootROM side consumes immediately during handshake */
            s->tx_r = (s->tx_r + 1) & 3;
            s->tx_count--;
        }
        fifo_rx_push(s, val);

        /* Multicore Launch State Machine: 0,0,1,VTOR,SP,ENTRY */
        switch (s->launch_state) {
        case 0:
        case 1:
            s->launch_state = (val == 0) ? (s->launch_state + 1) : 0;
            break;
        case 2:
            s->launch_state = (val == 1) ? 3 : 0;
            break;
        case 3:
            s->vector_table = val;
            s->launch_state = 4;
            break;
        case 4:
            s->sp = val;
            s->launch_state = 5;
            break;
        case 5:
            s->entry = val;
            s->launch_state = 0;

            if (s->core1 && s->core1->cpu) {
                CPUState *cs = CPU(s->core1->cpu);
                ARMCPU *arm_cpu = ARM_CPU(cs);

                cpu_set_pc(cs, s->entry);
                arm_cpu->env.regs[13] = s->sp;
                arm_cpu->env.v7m.vecbase[M_REG_NS] = s->vector_table;
                arm_cpu->env.v7m.vecbase[M_REG_S] = s->vector_table;

                cs->halted = 0;
                qemu_cpu_kick(cs);
            }
            break;
        default:
            s->launch_state = 0;
            break;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad write offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps rp2350_fifo_ops = {
    .read = rp2350_fifo_read,
    .write = rp2350_fifo_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* QOM Declaration */
#define TYPE_RP2350_MACHINE MACHINE_TYPE_NAME("rp2350")
OBJECT_DECLARE_SIMPLE_TYPE(RP2350State, RP2350_MACHINE)

struct RP2350State {
    MachineState parent;
    ARMv7MState cpu[RP2350_CPUS];
    Object idau;
    MemoryRegion bootrom;
    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion bootram;
    MemoryRegion flash_alias; 
    MemoryRegion usbdpram;
    MemoryRegion usbdpram_s_alias;

    RP2350FIFOState fifo;
    /* Per-core board memory view (aliases system memory) for ARMv7M devices */
    MemoryRegion board_mem[RP2350_CPUS];

    /* Minimal RESETS peripheral */
    MemoryRegion resets;

    /* Minimal XOSC peripheral */
    MemoryRegion xosc;

    /* Minimal PLL peripherals */
    MemoryRegion pll_sys;
    MemoryRegion pll_usb;

    /* Minimal POWMAN peripheral */
    MemoryRegion powman;
};

/* Minimal RESETS peripheral - always reports all peripherals out of reset */
#define RP2350_RESETS_BASE      0x40020000
#define RP2350_RESETS_SIZE      0x1000

static uint64_t rp2350_resets_read(void *opaque, hwaddr offset, unsigned size) {
    switch (offset) {
    case 0x00: /* RESET */
        return 0; /* All resets deasserted */
    case 0x04: /* WDSEL */
        return 0;
    case 0x08: /* RESET_DONE */
        return 0x1fffffff; /* All peripherals out of reset */
    default:
        return 0;
    }
}

static void rp2350_resets_write(void *opaque, hwaddr offset, uint64_t value, unsigned size) {
    /* Ignore all writes - we pretend peripherals are always ready */
}

static const MemoryRegionOps rp2350_resets_ops = {
    .read = rp2350_resets_read,
    .write = rp2350_resets_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Minimal XOSC peripheral - always reports stable */
#define RP2350_XOSC_BASE        0x40048000
#define RP2350_XOSC_SIZE        0x1000

static uint64_t rp2350_xosc_read(void *opaque, hwaddr offset, unsigned size) {
    switch (offset) {
    case 0x00: /* CTRL */
        return 0x00fab000; /* Enabled */
    case 0x04: /* STATUS */
        return 0x80001000; /* STABLE=1, ENABLED=1 */
    default:
        return 0;
    }
}

static void rp2350_xosc_write(void *opaque, hwaddr offset, uint64_t value, unsigned size) {
    /* Ignore writes */
}

static const MemoryRegionOps rp2350_xosc_ops = {
    .read = rp2350_xosc_read,
    .write = rp2350_xosc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Minimal PLL peripheral - always reports locked */
#define RP2350_PLL_SYS_BASE     0x40050000
#define RP2350_PLL_USB_BASE     0x40058000
#define RP2350_PLL_SIZE         0x1000

#define RP2350_POWMAN_BASE      0x400d0000
#define RP2350_POWMAN_SIZE      0x1000

static uint64_t rp2350_pll_read(void *opaque, hwaddr offset, unsigned size) {
    switch (offset) {
    case 0x00: /* CS */
        return 0x80000001; /* LOCK=1, REFDIV=1 */
    default:
        return 0;
    }
}

static void rp2350_pll_write(void *opaque, hwaddr offset, uint64_t value, unsigned size) {
    /* Ignore writes */
}

static const MemoryRegionOps rp2350_pll_ops = {
    .read = rp2350_pll_read,
    .write = rp2350_pll_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* Minimal POWMAN peripheral - power manager */
static uint64_t rp2350_powman_read(void *opaque, hwaddr offset, unsigned size) {
    switch (offset) {
    case 0x00: /* STATE - bit 16 = power good/ready */
        return 0x00010000; /* Power is good */
    default:
        return 0;
    }
}

static void rp2350_powman_write(void *opaque, hwaddr offset, uint64_t value, unsigned size) {
    /* Ignore writes */
}

static const MemoryRegionOps rp2350_powman_ops = {
    .read = rp2350_powman_read,
    .write = rp2350_powman_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
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

    /* Reset FIFO model state */
    fifo_reset(&s->fifo);

    /* 1. Initialize Memory Regions */
    memory_region_init_ram(&s->sram, NULL, "rp2350.sram", RP2350_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RP2350_SRAM_BASE, &s->sram);

    /* BOOTRAM - used by BootROM for stack and RCP canary storage */
    memory_region_init_ram(&s->bootram, NULL, "rp2350.bootram", RP2350_BOOTRAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RP2350_BOOTRAM_BASE, &s->bootram);

    /* USB DPRAM - used by bootrom as stack during NS code execution */
    memory_region_init_ram(&s->usbdpram, NULL, "rp2350.usbdpram", RP2350_USBDPRAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), RP2350_USBDPRAM_NS_BASE, &s->usbdpram);
    /* Secure alias of USB DPRAM */
    memory_region_init_alias(&s->usbdpram_s_alias, NULL, "rp2350.usbdpram.s",
                             &s->usbdpram, 0, RP2350_USBDPRAM_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_USBDPRAM_S_BASE, &s->usbdpram_s_alias);

    /* Minimal RESETS peripheral */
    memory_region_init_io(&s->resets, NULL, &rp2350_resets_ops, s,
                          "rp2350.resets", RP2350_RESETS_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_RESETS_BASE, &s->resets);

    /* Minimal XOSC peripheral */
    memory_region_init_io(&s->xosc, NULL, &rp2350_xosc_ops, s,
                          "rp2350.xosc", RP2350_XOSC_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_XOSC_BASE, &s->xosc);

    /* Minimal PLL peripherals */
    memory_region_init_io(&s->pll_sys, NULL, &rp2350_pll_ops, s,
                          "rp2350.pll_sys", RP2350_PLL_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_PLL_SYS_BASE, &s->pll_sys);

    memory_region_init_io(&s->pll_usb, NULL, &rp2350_pll_ops, s,
                          "rp2350.pll_usb", RP2350_PLL_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_PLL_USB_BASE, &s->pll_usb);

    /* Minimal POWMAN peripheral */
    memory_region_init_io(&s->powman, NULL, &rp2350_powman_ops, s,
                          "rp2350.powman", RP2350_POWMAN_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_POWMAN_BASE, &s->powman);

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

    /* Create a 150MHz System Clock */
    Clock *sysclk = clock_new(OBJECT(machine), "sysclk");
    clock_set_hz(sysclk, 150000000); // 150 MHz

    /* RP2350 IDAU (security attribution source for v8-M) */
    object_initialize_child(OBJECT(machine), "idau", &s->idau,
                            TYPE_RP2350_IDAU);

    /* 2. Initialize CPUs */
    for (int n = 0; n < RP2350_CPUS; n++) {
        object_initialize_child(OBJECT(machine), "cpu[*]", &s->cpu[n],
                                TYPE_ARMV7M);
        qdev_prop_set_string(DEVICE(&s->cpu[n]), "cpu-type",
                             ARM_CPU_TYPE_NAME("cortex-m33"));
        qdev_prop_set_bit(DEVICE(&s->cpu[n]), "enable-bitband", true);

        /*
         * ARMv7M's "memory" property must be an unparented MemoryRegion.
         * Provide a per-core alias that covers all of system memory.
         */
        char *mrname = g_strdup_printf("rp2350.board-mem[%d]", n);
        memory_region_init_alias(&s->board_mem[n], OBJECT(machine), mrname,
                                 get_system_memory(), 0, UINT64_MAX);
        g_free(mrname);
        object_property_set_link(OBJECT(&s->cpu[n]), "memory",
                                 OBJECT(&s->board_mem[n]), &error_abort);

        /* Wire SoC-specific attribution provider (used by TT). */
        if (object_property_find(OBJECT(&s->cpu[n]), "idau")) {
            object_property_set_link(OBJECT(&s->cpu[n]), "idau",
                                     OBJECT(&s->idau), &error_abort);
        }

        /* Keep within the NVIC model limits in this tree. */
        object_property_set_int(OBJECT(&s->cpu[n]), "num-irq", 64,
                                &error_abort);

        /* Connect Clock to CPU */
        qdev_connect_clock_in(DEVICE(&s->cpu[n]), "cpuclk", sysclk);

        /* Core 1 starts halted */
        if (n == 1) {
            qdev_prop_set_bit(DEVICE(&s->cpu[n]), "start-powered-off", true);
        }

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpu[n]), &error_fatal)) {
            return;
        }
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
    sysbus_connect_irq(SYS_BUS_DEVICE(pl011), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_UART0_IRQ));

    /* SIO FIFO */
    s->fifo.core1 = &s->cpu[1];
    memory_region_init_io(&s->fifo.iomem, OBJECT(machine), &rp2350_fifo_ops,
                          &s->fifo, "rp2350.sio", RP2350_SIO_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_SIO_BASE,
                                &s->fifo.iomem);

    /* Dummy Timer */
    MemoryRegion *timer_mr = g_new(MemoryRegion, 1);
    memory_region_init_io(timer_mr, NULL, &dummy_timer_ops, NULL, "rp2350.timer", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0x40054000, timer_mr);

    /* 4. Unimplemented Devices */
    /*
     * BootROM touches various peripheral registers early. Until we model
     * them, provide a catch-all peripheral window so accesses don't HardFault.
     */
    create_unimplemented_device("rp2350.periph",  0x40000000, 0x100000);
    create_unimplemented_device("rp2350.sysinfo", 0x40000000, 0x4000); 
    create_unimplemented_device("rp2350.clocks",  0x40008000, 0x10000);
    create_unimplemented_device("rp2350.io_bank0", 0x40014000, 0x10000);

    /* 5. Load Firmware */
    armv7m_load_kernel(s->cpu[0].cpu, machine->kernel_filename, 0,
                       RP2350_FLASH_SIZE);
}

static void rp2350_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Raspberry Pi RP2350 (Cortex-M33)";
    mc->init = rp2350_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    mc->ignore_memory_transaction_failures = true; 

    mc->max_cpus = RP2350_CPUS;
    mc->default_cpus = RP2350_CPUS;
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