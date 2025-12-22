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
#include "target/arm/internals.h"
#include "target/arm/arm-powerctl.h"

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

/* IO_BANK0 - GPIO function selection */
#define RP2350_IO_BANK0_BASE    0x40028000
#define RP2350_IO_BANK0_SIZE    0x1000
#define RP2350_NUM_GPIOS        48

/* PADS_BANK0 - GPIO pad control */
#define RP2350_PADS_BANK0_BASE  0x40038000
#define RP2350_PADS_BANK0_SIZE  0x1000

/* Minimal IDAU for BootROM bring-up (implemented in hw/misc/rp2350-idau.c). */
#define TYPE_RP2350_IDAU "rp2350-idau"

/* SIO Register Offsets */
#define SIO_CPUID           0x000
#define SIO_GPIO_IN         0x004
#define SIO_GPIO_HI_IN      0x008
#define SIO_GPIO_OUT        0x010
#define SIO_GPIO_HI_OUT     0x014
#define SIO_GPIO_OUT_SET    0x018
#define SIO_GPIO_HI_OUT_SET 0x01c
#define SIO_GPIO_OUT_CLR    0x020
#define SIO_GPIO_HI_OUT_CLR 0x024
#define SIO_GPIO_OUT_XOR    0x028
#define SIO_GPIO_HI_OUT_XOR 0x02c
#define SIO_GPIO_OE         0x030
#define SIO_GPIO_HI_OE      0x034
#define SIO_GPIO_OE_SET     0x038
#define SIO_GPIO_HI_OE_SET  0x03c
#define SIO_GPIO_OE_CLR     0x040
#define SIO_GPIO_HI_OE_CLR  0x044
#define SIO_GPIO_OE_XOR     0x048
#define SIO_GPIO_HI_OE_XOR  0x04c
#define SIO_FIFO_ST         0x050
#define SIO_FIFO_WR         0x054
#define SIO_FIFO_RD         0x058
#define SIO_SPINLOCK_ST     0x05c

typedef struct RP2350SIOState {
    MemoryRegion iomem;
    ARMv7MState *core1;

    /* GPIO State */
    uint32_t gpio_in;       /* GPIO0-31 input values */
    uint32_t gpio_hi_in;    /* GPIO32-47 input values (bits 15:0) */
    uint32_t gpio_out;      /* GPIO0-31 output values */
    uint32_t gpio_hi_out;   /* GPIO32-47 output values */
    uint32_t gpio_oe;       /* GPIO0-31 output enable */
    uint32_t gpio_hi_oe;    /* GPIO32-47 output enable */

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

    /* Spinlocks */
    uint32_t spinlock[32];
} RP2350SIOState;

static inline void sio_reset(RP2350SIOState *s)
{
    s->gpio_in = 0;
    s->gpio_hi_in = 0;
    s->gpio_out = 0;
    s->gpio_hi_out = 0;
    s->gpio_oe = 0;
    s->gpio_hi_oe = 0;
    s->tx_r = s->tx_w = s->tx_count = 0;
    s->rx_r = s->rx_w = s->rx_count = 0;
    s->roe = false;
    s->wof = false;
    memset(s->spinlock, 0, sizeof(s->spinlock));
}

static inline void fifo_rx_push(RP2350SIOState *s, uint32_t v)
{
    if (s->rx_count == 4) {
        s->wof = true;
        return;
    }
    s->rx_fifo[s->rx_w] = v;
    s->rx_w = (s->rx_w + 1) & 3;
    s->rx_count++;
}

static inline uint32_t fifo_rx_pop(RP2350SIOState *s, bool *ok)
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

static inline void fifo_tx_push(RP2350SIOState *s, uint32_t v)
{
    if (s->tx_count == 4) {
        s->wof = true;
        return;
    }
    s->tx_fifo[s->tx_w] = v;
    s->tx_w = (s->tx_w + 1) & 3;
    s->tx_count++;
}

static uint64_t rp2350_sio_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350SIOState *s = opaque;
    uint64_t val = 0;

    switch (addr) {
    case SIO_CPUID:
        val = current_cpu ? current_cpu->cpu_index : 0;
        break;

    /* GPIO Input */
    case SIO_GPIO_IN:
        /* For outputs, GPIO_IN reflects GPIO_OUT when OE is set */
        val = (s->gpio_out & s->gpio_oe) | (s->gpio_in & ~s->gpio_oe);
        break;
    case SIO_GPIO_HI_IN:
        val = (s->gpio_hi_out & s->gpio_hi_oe) | (s->gpio_hi_in & ~s->gpio_hi_oe);
        break;

    /* GPIO Output */
    case SIO_GPIO_OUT:
        val = s->gpio_out;
        break;
    case SIO_GPIO_HI_OUT:
        val = s->gpio_hi_out;
        break;

    /* GPIO Output Enable */
    case SIO_GPIO_OE:
        val = s->gpio_oe;
        break;
    case SIO_GPIO_HI_OE:
        val = s->gpio_hi_oe;
        break;

    /* FIFO Status */
    case SIO_FIFO_ST:
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

    /* FIFO Read */
    case SIO_FIFO_RD: {
        bool ok;
        val = fifo_rx_pop(s, &ok);
        break;
    }

    /* Spinlock State */
    case SIO_SPINLOCK_ST: {
        val = 0;
        for (int i = 0; i < 32; i++) {
            if (s->spinlock[i]) {
                val |= (1u << i);
            }
        }
        break;
    }

    /* Spinlock registers (0x100-0x17c) */
    default:
        if (addr >= 0x100 && addr < 0x180) {
            int lock_num = (addr - 0x100) / 4;
            int cpu_id = current_cpu ? current_cpu->cpu_index : 0;
            if (s->spinlock[lock_num] == 0) {
                /* Lock is free, claim it */
                s->spinlock[lock_num] = cpu_id + 1; /* Store owner (1 or 2) */
                val = (1u << lock_num);
            } else {
                /* Lock is held, return 0 */
                val = 0;
            }
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "rp2350_sio_read: unimplemented offset 0x%"HWADDR_PRIx"\n",
                          addr);
        }
        break;
    }
    return val;
}

static void rp2350_sio_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    RP2350SIOState *s = opaque;

    switch (addr) {
    /* GPIO Output */
    case SIO_GPIO_OUT:
        s->gpio_out = val;
        break;
    case SIO_GPIO_HI_OUT:
        s->gpio_hi_out = val & 0xFFFF; /* Only bits 15:0 valid */
        break;
    case SIO_GPIO_OUT_SET:
        s->gpio_out |= val;
        break;
    case SIO_GPIO_HI_OUT_SET:
        s->gpio_hi_out |= (val & 0xFFFF);
        break;
    case SIO_GPIO_OUT_CLR:
        s->gpio_out &= ~val;
        break;
    case SIO_GPIO_HI_OUT_CLR:
        s->gpio_hi_out &= ~(val & 0xFFFF);
        break;
    case SIO_GPIO_OUT_XOR:
        s->gpio_out ^= val;
        break;
    case SIO_GPIO_HI_OUT_XOR:
        s->gpio_hi_out ^= (val & 0xFFFF);
        break;

    /* GPIO Output Enable */
    case SIO_GPIO_OE:
        s->gpio_oe = val;
        break;
    case SIO_GPIO_HI_OE:
        s->gpio_hi_oe = val & 0xFFFF;
        break;
    case SIO_GPIO_OE_SET:
        s->gpio_oe |= val;
        break;
    case SIO_GPIO_HI_OE_SET:
        s->gpio_hi_oe |= (val & 0xFFFF);
        break;
    case SIO_GPIO_OE_CLR:
        s->gpio_oe &= ~val;
        break;
    case SIO_GPIO_HI_OE_CLR:
        s->gpio_hi_oe &= ~(val & 0xFFFF);
        break;
    case SIO_GPIO_OE_XOR:
        s->gpio_oe ^= val;
        break;
    case SIO_GPIO_HI_OE_XOR:
        s->gpio_hi_oe ^= (val & 0xFFFF);
        break;

    /* FIFO Status (write clears ROE/WOF) */
    case SIO_FIFO_ST:
        s->roe = false;
        s->wof = false;
        break;

    /* FIFO Write */
    case SIO_FIFO_WR:
        fifo_tx_push(s, val);
        if (s->tx_count > 0) {
            /* BootROM side consumes immediately during handshake */
            s->tx_r = (s->tx_r + 1) & 3;
            s->tx_count--;
        }
        fifo_rx_push(s, val);

        /* Multicore Launch State Machine: 0,0,1,VTOR,SP,ENTRY */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "SIO FIFO_WR: val=0x%08x, launch_state=%d\n",
                      (uint32_t)val, s->launch_state);
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

            qemu_log_mask(LOG_GUEST_ERROR,
                          "SIO: Core 1 launch requested - entry=0x%08x sp=0x%08x vtor=0x%08x\n",
                          s->entry, s->sp, s->vector_table);

            if (s->core1 && s->core1->cpu) {
                CPUState *cs = CPU(s->core1->cpu);
                ARMCPU *arm_cpu = ARM_CPU(cs);

                /* Safety: ensure stack pointer points into system RAM */
                hwaddr sp_hw = s->sp;
                if (sp_hw < RP2350_SRAM_BASE || sp_hw > (RP2350_SRAM_BASE + RP2350_SRAM_SIZE)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "SIO: Core 1 SP 0x%08"HWADDR_PRIx" outside SRAM (not starting)\n",
                                  sp_hw);
                } else {
                    /* Set vector table base for Core 1 in both env and init fields */
                    arm_cpu->env.v7m.vecbase[M_REG_NS] = s->vector_table;
                    arm_cpu->env.v7m.vecbase[M_REG_S] = s->vector_table;
                    arm_cpu->init_svtor = s->vector_table;
                    arm_cpu->init_nsvtor = s->vector_table;

                    /* Prepare core1 stack so trampoline pop {r0,r1,pc} will work.
                     * Firmware pushes: entry, stack_bottom, core1_wrapper onto the
                     * stack pointer before the final handshake; emulate by writing
                     * these three words to memory at SP - 12 so that when core1
                     * starts and executes its trampoline, it pops the values.
                     */
                    hwaddr arg_addr = sp_hw - 12; /* write 3 words such that pop {r0,r1,pc} reads them */
                    uint32_t entry_le = cpu_to_le32((uint32_t)s->entry);
                    uint32_t stack_bottom_le = cpu_to_le32((uint32_t)s->sp);
                    /* core1_wrapper fallback to entry if unknown */
                    uint32_t wrapper_le = cpu_to_le32((uint32_t)s->entry);

                    address_space_write(&address_space_memory, arg_addr,
                                        MEMTXATTRS_UNSPECIFIED,
                                        &entry_le, sizeof(entry_le));
                    address_space_write(&address_space_memory, arg_addr + 4,
                                        MEMTXATTRS_UNSPECIFIED,
                                        &stack_bottom_le, sizeof(stack_bottom_le));
                    address_space_write(&address_space_memory, arg_addr + 8,
                                        MEMTXATTRS_UNSPECIFIED,
                                        &wrapper_le, sizeof(wrapper_le));

                    /* Read back what we wrote to verify */
                    uint32_t rb0 = 0, rb1 = 0, rb2 = 0;
                    address_space_read(&address_space_memory, arg_addr,
                                       MEMTXATTRS_UNSPECIFIED,
                                       &rb0, sizeof(rb0));
                    address_space_read(&address_space_memory, arg_addr + 4,
                                       MEMTXATTRS_UNSPECIFIED,
                                       &rb1, sizeof(rb1));
                    address_space_read(&address_space_memory, arg_addr + 8,
                                       MEMTXATTRS_UNSPECIFIED,
                                       &rb2, sizeof(rb2));
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "SIO: Wrote trampoline args @0x%08"HWADDR_PRIx": [0]=0x%08x [1]=0x%08x [2]=0x%08x\n",
                                  arg_addr, rb0, rb1, rb2);

                    /* Set SP and LR */
                    arm_cpu->env.regs[13] = s->sp;
                    arm_cpu->env.regs[14] = 0xffffffff; /* LR */

                    /* Set PC to entry (firmware sets thumb bit). Use cpu_set_pc. */
                    cpu_set_pc(cs, s->entry);

                    /* Ensure Thread mode (no exception) */
                    arm_cpu->env.v7m.exception = 0;

                    /* Mark as powered on and not halted */
                    arm_cpu->power_state = PSCI_ON;
                    cs->halted = 0;

                    /* Wake up the CPU */
                    qemu_cpu_kick(cs);
                    qemu_log_mask(LOG_GUEST_ERROR, "SIO: Core 1 started (kicked)\n");
                }
            } else {
                qemu_log_mask(LOG_GUEST_ERROR, "SIO: Core1 CPU object not available\n");
            }
            break;
        default:
            s->launch_state = 0;
            break;
        }
        break;

    /* Spinlock registers (0x100-0x17c) - write releases lock */
    default:
        if (addr >= 0x100 && addr < 0x180) {
            int lock_num = (addr - 0x100) / 4;
            s->spinlock[lock_num] = 0; /* Release lock */
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "rp2350_sio_write: unimplemented offset 0x%"HWADDR_PRIx"\n",
                          addr);
        }
        break;
    }
}

static const MemoryRegionOps rp2350_sio_ops = {
    .read = rp2350_sio_read,
    .write = rp2350_sio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== IO_BANK0 - GPIO Function Selection ========== */
typedef struct RP2350IOBank0State {
    MemoryRegion iomem;
    /* Each GPIO has STATUS (ro) and CTRL (rw) registers */
    /* ctrl[i] bits [4:0] = FUNCSEL, default 0x1f (NULL) */
    uint32_t gpio_ctrl[RP2350_NUM_GPIOS];
    uint32_t gpio_status[RP2350_NUM_GPIOS];
} RP2350IOBank0State;

static void io_bank0_reset(RP2350IOBank0State *s)
{
    for (int i = 0; i < RP2350_NUM_GPIOS; i++) {
        s->gpio_ctrl[i] = 0x1f; /* FUNCSEL = NULL (0x1f) */
        s->gpio_status[i] = 0;
    }
}

static uint64_t rp2350_io_bank0_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350IOBank0State *s = opaque;
    uint64_t val = 0;

    /* GPIO status/ctrl: GPIOx_STATUS at 0x000 + 8*x, GPIOx_CTRL at 0x004 + 8*x */
    if (addr < (RP2350_NUM_GPIOS * 8)) {
        int gpio = addr / 8;
        int reg = addr % 8;
        if (reg == 0) {
            /* STATUS register */
            val = s->gpio_status[gpio];
        } else if (reg == 4) {
            /* CTRL register */
            val = s->gpio_ctrl[gpio];
        }
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_io_bank0_read: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
    return val;
}

static void rp2350_io_bank0_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
    RP2350IOBank0State *s = opaque;

    /* GPIO status/ctrl: GPIOx_STATUS at 0x000 + 8*x, GPIOx_CTRL at 0x004 + 8*x */
    if (addr < (RP2350_NUM_GPIOS * 8)) {
        int gpio = addr / 8;
        int reg = addr % 8;
        if (reg == 4) {
            /* CTRL register - writable */
            s->gpio_ctrl[gpio] = val;
        }
        /* STATUS register is read-only */
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_io_bank0_write: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
}

static const MemoryRegionOps rp2350_io_bank0_ops = {
    .read = rp2350_io_bank0_read,
    .write = rp2350_io_bank0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== PADS_BANK0 - GPIO Pad Control ========== */
typedef struct RP2350PadsBank0State {
    MemoryRegion iomem;
    uint32_t voltage_select;
    /* Pad control for each GPIO: IE, OD, PUE, PDE, SCHMITT, SLEWFAST, DRIVE */
    /* Default: IE=1, OD=0, so default value is 0x56 typically */
    uint32_t gpio_pad[RP2350_NUM_GPIOS];
} RP2350PadsBank0State;

static void pads_bank0_reset(RP2350PadsBank0State *s)
{
    s->voltage_select = 0;
    for (int i = 0; i < RP2350_NUM_GPIOS; i++) {
        /* Default: IE=1, DRIVE=1, SCHMITT=1, others=0 → 0x56 */
        s->gpio_pad[i] = 0x56;
    }
}

static uint64_t rp2350_pads_bank0_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350PadsBank0State *s = opaque;
    uint64_t val = 0;

    if (addr == 0x00) {
        /* VOLTAGE_SELECT */
        val = s->voltage_select;
    } else if (addr >= 0x04 && addr < (0x04 + RP2350_NUM_GPIOS * 4)) {
        /* GPIO pad registers: GPIO0 at 0x04, GPIO1 at 0x08, etc. */
        int gpio = (addr - 0x04) / 4;
        if (gpio < RP2350_NUM_GPIOS) {
            val = s->gpio_pad[gpio];
        }
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_pads_bank0_read: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
    return val;
}

static void rp2350_pads_bank0_write(void *opaque, hwaddr addr, uint64_t val,
                                     unsigned size)
{
    RP2350PadsBank0State *s = opaque;

    if (addr == 0x00) {
        /* VOLTAGE_SELECT */
        s->voltage_select = val & 0x1;
    } else if (addr >= 0x04 && addr < (0x04 + RP2350_NUM_GPIOS * 4)) {
        /* GPIO pad registers */
        int gpio = (addr - 0x04) / 4;
        if (gpio < RP2350_NUM_GPIOS) {
            s->gpio_pad[gpio] = val;
        }
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_pads_bank0_write: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
}

static const MemoryRegionOps rp2350_pads_bank0_ops = {
    .read = rp2350_pads_bank0_read,
    .write = rp2350_pads_bank0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== TIMER - Microsecond Timer (struct only) ========== */
typedef struct RP2350TimerState {
    MemoryRegion iomem;
    int64_t start_time_ns;  /* QEMU clock time when timer started */
    uint32_t alarm[4];
    uint32_t armed;
    uint32_t pause;
    uint32_t inte;
    uint32_t intf;
    uint32_t latched_hi;    /* For atomic 64-bit reads */
} RP2350TimerState;

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

    RP2350SIOState sio;
    RP2350IOBank0State io_bank0;
    RP2350PadsBank0State pads_bank0;
    RP2350TimerState timer0;
    RP2350TimerState timer1;
    /* Per-core board memory view (aliases system memory) for ARMv7M devices */
    MemoryRegion board_mem[RP2350_CPUS];

    /* Minimal RESETS peripheral */
    MemoryRegion resets;

    /* Minimal SCB/NVIC region (separate from resets) */
    MemoryRegion scb;

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

/* Minimal SCB/NVIC handler to return safe defaults for a few offsets
 * so runtime initializers that probe system control registers don't
 * trigger bad-read errors or HardFaults. This is intentionally conservative.
 */
static uint64_t rp2350_scb_read(void *opaque, hwaddr offset, unsigned size) {
    switch (offset) {
    case 0x00: /* CPUID Base */
        return 0x410FC241; /* Example: Cortex-M33 CPUID base (implement safe value) */
    case 0x08: /* ACTLR */
        return 0x0; /* Default ACTLR value */
    case 0x0D00: /* ICSR (interrupt control) */
        return 0x0;
    default:
        /* Return zero for other system-reg probes */
        return 0;
    }
}

static void rp2350_scb_write(void *opaque, hwaddr offset, uint64_t value, unsigned size) {
    /* Ignore writes to SCB/ACTLR/ICSR etc. */
    (void)opaque; (void)offset; (void)value; (void)size;
}

static const MemoryRegionOps rp2350_scb_ops = {
    .read = rp2350_scb_read,
    .write = rp2350_scb_write,
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

/* ========== TIMER - Microsecond Timer ========== */
#define RP2350_TIMER0_BASE      0x400b0000
#define RP2350_TIMER1_BASE      0x400b8000
#define RP2350_TIMER_SIZE       0x1000

/* Timer register offsets */
#define TIMER_TIMEHW        0x00
#define TIMER_TIMELW        0x04
#define TIMER_TIMEHR        0x08
#define TIMER_TIMELR        0x0c
#define TIMER_ALARM0        0x10
#define TIMER_ALARM1        0x14
#define TIMER_ALARM2        0x18
#define TIMER_ALARM3        0x1c
#define TIMER_ARMED         0x20
#define TIMER_TIMERAWH      0x24
#define TIMER_TIMERAWL      0x28
#define TIMER_DBGPAUSE      0x2c
#define TIMER_PAUSE         0x30
#define TIMER_LOCKED        0x34
#define TIMER_SOURCE        0x38
#define TIMER_INTR          0x3c
#define TIMER_INTE          0x40
#define TIMER_INTF          0x44
#define TIMER_INTS          0x48

static uint64_t timer_get_time_us(RP2350TimerState *s)
{
    /* Get current QEMU virtual time in nanoseconds, convert to microseconds */
    int64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now_ns - s->start_time_ns;
    return (uint64_t)(elapsed_ns / 1000);  /* ns to us */
}

static uint64_t rp2350_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    RP2350TimerState *s = opaque;
    uint64_t time_us;
    uint32_t val = 0;

    switch (offset) {
    case TIMER_TIMEHR:
        /* Reading TIMEHR latches TIMELR, return high 32 bits */
        time_us = timer_get_time_us(s);
        s->latched_hi = (uint32_t)(time_us >> 32);
        val = s->latched_hi;
        break;
    case TIMER_TIMELR:
        /* Returns low 32 bits (uses latched value from TIMEHR read) */
        time_us = timer_get_time_us(s);
        val = (uint32_t)time_us;
        break;
    case TIMER_TIMERAWH:
        /* Raw high 32 bits, no latching */
        time_us = timer_get_time_us(s);
        val = (uint32_t)(time_us >> 32);
        break;
    case TIMER_TIMERAWL:
        /* Raw low 32 bits, no latching */
        time_us = timer_get_time_us(s);
        val = (uint32_t)time_us;
        break;
    case TIMER_ALARM0:
    case TIMER_ALARM1:
    case TIMER_ALARM2:
    case TIMER_ALARM3:
        val = s->alarm[(offset - TIMER_ALARM0) / 4];
        break;
    case TIMER_ARMED:
        val = s->armed;
        break;
    case TIMER_PAUSE:
        val = s->pause;
        break;
    case TIMER_INTE:
        val = s->inte;
        break;
    case TIMER_INTF:
        val = s->intf;
        break;
    case TIMER_INTS:
        val = s->inte & s->intf;
        break;
    default:
        break;
    }
    return val;
}

static void rp2350_timer_write(void *opaque, hwaddr offset, uint64_t val,
                                unsigned size)
{
    RP2350TimerState *s = opaque;

    switch (offset) {
    case TIMER_ALARM0:
    case TIMER_ALARM1:
    case TIMER_ALARM2:
    case TIMER_ALARM3:
        s->alarm[(offset - TIMER_ALARM0) / 4] = val;
        s->armed |= (1 << ((offset - TIMER_ALARM0) / 4));
        break;
    case TIMER_ARMED:
        /* Writing 1 disarms the alarm */
        s->armed &= ~val;
        break;
    case TIMER_PAUSE:
        s->pause = val & 1;
        break;
    case TIMER_INTE:
        s->inte = val & 0xf;
        break;
    case TIMER_INTF:
        s->intf = val & 0xf;
        break;
    case TIMER_INTR:
        /* Write to clear interrupts */
        s->intf &= ~val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps rp2350_timer_ops = {
    .read = rp2350_timer_read,
    .write = rp2350_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void rp2350_timer_reset(RP2350TimerState *s)
{
    s->start_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->armed = 0;
    s->pause = 0;
    s->inte = 0;
    s->intf = 0;
    memset(s->alarm, 0, sizeof(s->alarm));
}

static void rp2350_init(MachineState *machine)
{
    RP2350State *s = RP2350_MACHINE(machine);

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

    /* 
     * BootROM / Flash Loading Logic:
     * - If firmware (-bios) is provided: load bootrom to address 0
     * - Otherwise: alias flash to address 0 for direct kernel boot
     * 
     * Note: For testing without full bootrom, use --wrap linker flags
     * to stub out ROM functions like get_rand_32() in the firmware.
     */
    if (machine->firmware) {
        memory_region_init_ram(&s->bootrom, NULL, "rp2350.bootrom", RP2350_BOOTROM_SIZE, &error_fatal);
        memory_region_add_subregion(get_system_memory(), RP2350_BOOTROM_BASE, &s->bootrom);
        
        int image_size = load_image_targphys(machine->firmware, RP2350_BOOTROM_BASE, RP2350_BOOTROM_SIZE, NULL);
        if (image_size < 0) {
            error_report("Could not load BootROM image '%s'", machine->firmware);
            exit(1);
        }
    } else {
        /* No bootrom - alias flash to address 0 for direct kernel boot */
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

    /* SIO (includes GPIO and FIFO) */
    s->sio.core1 = &s->cpu[1];
    sio_reset(&s->sio);
    memory_region_init_io(&s->sio.iomem, OBJECT(machine), &rp2350_sio_ops,
                          &s->sio, "rp2350.sio", RP2350_SIO_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_SIO_BASE,
                                &s->sio.iomem);

    /* IO_BANK0 - GPIO function selection */
    io_bank0_reset(&s->io_bank0);
    memory_region_init_io(&s->io_bank0.iomem, OBJECT(machine), &rp2350_io_bank0_ops,
                          &s->io_bank0, "rp2350.io_bank0", RP2350_IO_BANK0_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_IO_BANK0_BASE,
                                &s->io_bank0.iomem);

    /* PADS_BANK0 - GPIO pad control */
    pads_bank0_reset(&s->pads_bank0);
    memory_region_init_io(&s->pads_bank0.iomem, OBJECT(machine), &rp2350_pads_bank0_ops,
                          &s->pads_bank0, "rp2350.pads_bank0", RP2350_PADS_BANK0_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_PADS_BANK0_BASE,
                                &s->pads_bank0.iomem);

    /* TIMER0 - Microsecond timer */
    rp2350_timer_reset(&s->timer0);
    memory_region_init_io(&s->timer0.iomem, OBJECT(machine), &rp2350_timer_ops,
                          &s->timer0, "rp2350.timer0", RP2350_TIMER_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_TIMER0_BASE,
                                &s->timer0.iomem);

    /* TIMER1 - Microsecond timer */
    rp2350_timer_reset(&s->timer1);
    memory_region_init_io(&s->timer1.iomem, OBJECT(machine), &rp2350_timer_ops,
                          &s->timer1, "rp2350.timer1", RP2350_TIMER_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_TIMER1_BASE,
                                &s->timer1.iomem);

    /* 4. Unimplemented Devices */
    /*
     * BootROM touches various peripheral registers early. Until we model
     * them, provide a catch-all peripheral window so accesses don't HardFault.
     */
    /* Broad catch-all for unknown peripherals (avoid HardFault on access) */
    create_unimplemented_device("rp2350.periph",  0x40000000, 0x00100000);
    /* Smaller aliases for common early-touch regions (keeps logs clearer)
     * NOTE: these overlap with the broad periph window above but are left
     * as explicit devices so their log prefixes are meaningful when seen.
     */
    create_unimplemented_device("rp2350.sysinfo", 0x40000000, 0x00004000);
    create_unimplemented_device("rp2350.clocks",  0x40008000, 0x00010000);
    create_unimplemented_device("rp2350.io_bank0", 0x40014000, 0x00010000);

    /* Map SCB/NVIC area with safe handlers to return defaults for CPUID/ACTLR etc. */
    memory_region_init_io(&s->scb, NULL, &rp2350_scb_ops, s,
                          "rp2350.scb", 0x1000);
    memory_region_add_subregion(get_system_memory(), 0xe000e000, &s->scb);

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