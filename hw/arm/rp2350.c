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
#include "hw/ssi/pl022.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
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

/* IO_IRQ_BANK0 - GPIO interrupt (RP2350 has it at IRQ 21) */
#define RP2350_IO_IRQ_BANK0     21

/* SIO */
#define RP2350_SIO_BASE         0xd0000000
#define RP2350_SIO_SIZE         0x1000
#define RP2350_CPUS             2

/* IO_BANK0 - GPIO function selection */
#define RP2350_IO_BANK0_BASE    0x40028000
#define RP2350_IO_BANK0_SIZE    0x4000  /* Include atomic SET/CLR/XOR aliases */
#define RP2350_NUM_GPIOS        48

/* PSM - Power State Machine */
#define RP2350_PSM_BASE         0x40018000
#define RP2350_PSM_SIZE         0x4000  /* Include atomic SET/CLR/XOR aliases */

/* PADS_BANK0 - GPIO pad control */
#define RP2350_PADS_BANK0_BASE  0x40038000
#define RP2350_PADS_BANK0_SIZE  0x1000

/* ADC - Analog-to-Digital Converter */
#define RP2350_ADC_BASE         0x400a0000
#define RP2350_ADC_SIZE         0x1000

/* DMA - Direct Memory Access */
#define RP2350_DMA_BASE         0x50000000
#define RP2350_DMA_SIZE         0x1000

/* PWM - Pulse Width Modulation */
#define RP2350_PWM_BASE         0x400a8000
#define RP2350_PWM_SIZE         0x4000  /* 16KB: base + XOR/SET/CLR aliases */

/* SPI - Synchronous Serial Port (PL022) */
#define RP2350_SPI0_BASE        0x40080000
#define RP2350_SPI1_BASE        0x40088000
#define RP2350_SPI_SIZE         0x4000  /* 16KB: base + XOR/SET/CLR aliases */
#define RP2350_SPI0_IRQ         31
#define RP2350_SPI1_IRQ         32

/* ADC/DMA Shared Constants */
#define RP2350_NUM_DMA_CHANNELS 16
#define RP2350_ADC_FIFO_SIZE    16      /* ADC sample FIFO depth */
#define DREQ_ADC                48      /* DMA Request select for ADC */

/* PWM Constants */
#define RP2350_NUM_PWM_SLICES   12      /* PWM has 12 slices (channels) */

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

/* QEMU-only magic address: write GPIO pin mask to inject rising edge on inputs.
 * This triggers IO_IRQ_BANK0 on the appropriate core. Used by autotests to
 * simulate keypad presses without real hardware. */
#define SIO_QEMU_GPIO_INJECT 0x1F0

/* Forward declaration */
typedef struct RP2350State RP2350State;

/* Number of INTx registers (6 registers, 8 GPIOs per reg) - needed before IO_BANK0 struct */
#define IO_BANK0_NUM_IRQ_REGS       6

/* IO_BANK0 state - defined early so SIO can reference it for GPIO injection */
typedef struct RP2350IOBank0State {
    MemoryRegion iomem;
    /* Each GPIO has STATUS (ro) and CTRL (rw) registers */
    /* ctrl[i] bits [4:0] = FUNCSEL, default 0x1f (NULL) */
    uint32_t gpio_ctrl[RP2350_NUM_GPIOS];
    uint32_t gpio_status[RP2350_NUM_GPIOS];
    
    /* Interrupt registers: 6 registers each (8 GPIOs per register, 4 bits per GPIO) */
    uint32_t intr[IO_BANK0_NUM_IRQ_REGS];         /* Raw interrupts (write to clear edge) */
    uint32_t proc0_inte[IO_BANK0_NUM_IRQ_REGS];   /* PROC0 interrupt enable */
    uint32_t proc0_intf[IO_BANK0_NUM_IRQ_REGS];   /* PROC0 interrupt force */
    uint32_t proc1_inte[IO_BANK0_NUM_IRQ_REGS];   /* PROC1 interrupt enable */
    uint32_t proc1_intf[IO_BANK0_NUM_IRQ_REGS];   /* PROC1 interrupt force */
    uint32_t dormant_wake_inte[IO_BANK0_NUM_IRQ_REGS];  /* Dormant wake enable */
    uint32_t dormant_wake_intf[IO_BANK0_NUM_IRQ_REGS];  /* Dormant wake force */
    
    /* IRQ lines for deassertion */
    qemu_irq io_irq_bank0[2];  /* [0]=core0, [1]=core1 */
} RP2350IOBank0State;

/* ADC state */
typedef struct RP2350ADCState {
    MemoryRegion iomem;
    
    /* ADC Registers */
    uint32_t cs;            /* Control/Status (EN, START_ONCE, START_MANY, READY, ERR, AINSEL, RROBIN) */
    uint32_t result;        /* 12-bit result of last conversion */
    uint32_t fcs;           /* FIFO Control/Status (EN, SHIFT, ERR, DREQ_EN, EMPTY, FULL, UNDER, OVER, LEVEL, THRESH) */
    uint32_t div;           /* Clock divider (INT + FRAC for conversion rate) */
    uint32_t intr;          /* Raw interrupt status (FIFO) */
    uint32_t inte;          /* Interrupt enable (FIFO) */
    uint32_t intf;          /* Interrupt force (FIFO) */
    
    /* FIFO: circular buffer of 12-bit samples */
    uint16_t fifo_data[RP2350_ADC_FIFO_SIZE];  /* 16 entries */
    int fifo_r, fifo_w;     /* Read/write indices */
    int fifo_count;         /* Current FIFO level */
    
    /* Conversion simulation */
    QEMUTimer *conv_timer;  /* QEMU timer for simulating conversion delay */
    RP2350State *parent;    /* Back pointer to parent machine state */
} RP2350ADCState;

/* DMA Channel state */
typedef struct RP2350DMAChannelState {
    uint32_t read_addr;     /* Current read address */
    uint32_t write_addr;    /* Current write address */
    uint32_t trans_count;   /* Transfer count + MODE field (bits 31:28) */
    uint32_t ctrl_trig;     /* Control register (EN, DATA_SIZE, INCR_READ/WRITE, TREQ_SEL, etc) */
} RP2350DMAChannelState;

/* DMA state */
typedef struct RP2350DMAState {
    MemoryRegion iomem;
    
    /* 16 DMA channels */
    RP2350DMAChannelState channel[RP2350_NUM_DMA_CHANNELS];
    
    /* Global DMA Interrupt registers */
    uint32_t intr;          /* Raw interrupt status (16 bits, one per channel) */
    uint32_t inte0;         /* Interrupt enable for IRQ 0 */
    uint32_t intf0;         /* Interrupt force for IRQ 0 */
    uint32_t ints0;         /* Interrupt status for IRQ 0 */
    
    /* Back references */
    RP2350State *parent;
    RP2350ADCState *adc;    /* Pointer to ADC for DREQ handling */
} RP2350DMAState;

/* PWM Slice state */
typedef struct RP2350PWMSliceState {
    uint32_t csr;           /* Control/Status (EN, PH_CORRECT, A_INV, B_INV, DIVMODE, PH_RET, PH_ADV) */
    uint32_t div;           /* Clock divider (INT + FRAC) */
    uint32_t ctr;           /* Counter value */
    uint32_t cc;            /* Compare values (A and B channels, 16-bit each) */
    uint32_t top;           /* Wrap/top value */
} RP2350PWMSliceState;

/* PWM state */
typedef struct RP2350PWMState {
    MemoryRegion iomem;
    
    /* 12 PWM slices (0-11), each with independent counter */
    RP2350PWMSliceState slice[RP2350_NUM_PWM_SLICES];
    
    /* Global PWM registers */
    uint32_t en;            /* Global enable for all channels (12 bits) */
    uint32_t intr;          /* Raw interrupt status (12 bits) */
    uint32_t inte;          /* Interrupt enable (12 bits, one per slice) */
    uint32_t intf;          /* Interrupt force */
    
    /* Back reference */
    RP2350State *parent;
} RP2350PWMState;

/* SPI Wrapper State - handles atomic aliases for PL022 */
typedef struct RP2350SPIState {
    MemoryRegion iomem;     /* 16KB region for base + aliases */
    PL022State *pl022;      /* Underlying PL022 device */
    int index;              /* SPI instance (0 or 1) */
} RP2350SPIState;

typedef struct RP2350SIOState {
    MemoryRegion iomem;
    ARMv7MState *core1;
    RP2350State *parent;    /* Back pointer for GPIO injection */
    RP2350IOBank0State *io_bank0;  /* Pointer to IO_BANK0 state for GPIO injection */

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

    /* IRQ lines for GPIO injection (IO_IRQ_BANK0 for each core) */
    qemu_irq io_irq_bank0[2];  /* [0]=core0, [1]=core1 */
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

                    /* Mark as powered on and not halted, clear any stop requests */
                    arm_cpu->power_state = PSCI_ON;
                    cs->halted = 0;
                    cs->stop = false;
                    cs->stopped = false;
                    cs->exit_request = false;

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
        } else if (addr == SIO_QEMU_GPIO_INJECT) {
            /* QEMU-only: inject GPIO rising edge on specified pins.
             * Writing a bitmask here will:
             * 1. Set the corresponding bits in gpio_in
             * 2. Set rising-edge bits in io_bank0 INTR and INTF registers
             * 3. Raise IO_IRQ_BANK0 on core1
             * This allows autotests to simulate keypad presses.
             * 
             * Note: We set both proc0_intf and proc1_intf because the firmware
             * may be buggy and use proc0_irq_ctrl even when running on core1.
             * Setting INTF forces the interrupt status regardless of INTE.
             */
            uint32_t mask = (uint32_t)val;
            
            qemu_log_mask(LOG_GUEST_ERROR,
                          "SIO_QEMU_GPIO_INJECT: mask=0x%08x\n", mask);
            
            /* Set gpio_in bits */
            s->gpio_in |= mask;
            
            /* Set rising-edge interrupt bits in io_bank0.intr[] and intf[]
             * Each GPIO has 4 bits: [3]=edge_high, [2]=edge_low, [1]=level_high, [0]=level_low
             * Rising edge = edge_high = bit 3 within each 4-bit group
             * GPIO_IRQ_EDGE_RISE = 0x8 (bit 3)
             */
            if (s->io_bank0) {
                for (int gpio = 0; gpio < 32; gpio++) {
                    if (mask & (1u << gpio)) {
                        int reg_idx = gpio / 8;
                        int bit_pos = (gpio % 8) * 4 + 3;  /* edge_high bit */
                        s->io_bank0->intr[reg_idx] |= (1u << bit_pos);
                        /* Set INTF (force) for both cores to force INTS high */
                        s->io_bank0->proc0_intf[reg_idx] |= (1u << bit_pos);
                        s->io_bank0->proc1_intf[reg_idx] |= (1u << bit_pos);
                        qemu_log_mask(LOG_GUEST_ERROR,
                                      "SIO_QEMU_GPIO_INJECT: set INTR/INTF[%d] bit %d for GPIO %d\n",
                                      reg_idx, bit_pos, gpio);
                    }
                }
            }
            
            /* Raise IO_IRQ_BANK0 on core1 if the IRQ line is valid. */
            if (s->io_irq_bank0[1]) {
                qemu_set_irq(s->io_irq_bank0[1], 1);
                qemu_log_mask(LOG_GUEST_ERROR,
                              "SIO_QEMU_GPIO_INJECT: raised IO_IRQ_BANK0 on core1\n");
            }
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

/* IO_BANK0 Register Offsets */
#define IO_BANK0_GPIO_STATUS_BASE   0x000   /* Status regs: 0x000 + gpio*8 */
#define IO_BANK0_GPIO_CTRL_BASE     0x004   /* Ctrl regs: 0x004 + gpio*8 */
#define IO_BANK0_INTR_BASE          0x230   /* Raw interrupts: INTR[0-5] */
#define IO_BANK0_PROC0_INTE_BASE    0x248   /* PROC0 interrupt enable: INTE[0-5] */
#define IO_BANK0_PROC0_INTF_BASE    0x260   /* PROC0 interrupt force: INTF[0-5] */
#define IO_BANK0_PROC0_INTS_BASE    0x278   /* PROC0 interrupt status: INTS[0-5] */
#define IO_BANK0_PROC1_INTE_BASE    0x290   /* PROC1 interrupt enable: INTE[0-5] */
#define IO_BANK0_PROC1_INTF_BASE    0x2A8   /* PROC1 interrupt force: INTF[0-5] */
#define IO_BANK0_PROC1_INTS_BASE    0x2C0   /* PROC1 interrupt status: INTS[0-5] */
#define IO_BANK0_DORMANT_WAKE_INTE_BASE 0x2D8  /* Dormant wake interrupt enable */
#define IO_BANK0_DORMANT_WAKE_INTF_BASE 0x2F0  /* Dormant wake interrupt force */
#define IO_BANK0_DORMANT_WAKE_INTS_BASE 0x308  /* Dormant wake interrupt status */

/* Note: IO_BANK0_NUM_IRQ_REGS and RP2350IOBank0State are defined earlier (before SIO) */

static void io_bank0_reset(RP2350IOBank0State *s)
{
    for (int i = 0; i < RP2350_NUM_GPIOS; i++) {
        s->gpio_ctrl[i] = 0x1f; /* FUNCSEL = NULL (0x1f) */
        s->gpio_status[i] = 0;
    }
    /* Clear all interrupt registers */
    for (int i = 0; i < IO_BANK0_NUM_IRQ_REGS; i++) {
        s->intr[i] = 0;
        s->proc0_inte[i] = 0;
        s->proc0_intf[i] = 0;
        s->proc1_inte[i] = 0;
        s->proc1_intf[i] = 0;
        s->dormant_wake_inte[i] = 0;
        s->dormant_wake_intf[i] = 0;
    }
}

/* Atomic operation types for RP2350 peripheral aliases */
#define ATOMIC_NORMAL   0   /* Direct read/write at +0x0000 */
#define ATOMIC_XOR      1   /* XOR alias at +0x1000 */
#define ATOMIC_SET      2   /* SET alias at +0x2000 */
#define ATOMIC_CLR      3   /* CLR alias at +0x3000 */

/* Helper to apply atomic write operation */
static inline void atomic_write_op(uint32_t *reg, uint32_t val, int op) {
    switch (op) {
        case ATOMIC_NORMAL: *reg = val; break;
        case ATOMIC_XOR:    *reg ^= val; break;
        case ATOMIC_SET:    *reg |= val; break;
        case ATOMIC_CLR:    *reg &= ~val; break;
    }
}

static uint64_t rp2350_io_bank0_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350IOBank0State *s = opaque;
    uint64_t val = 0;
    
    /* Extract atomic operation type and base offset */
    int atomic_op = (addr >> 12) & 0x3;
    hwaddr reg_offset = addr & 0xFFF;
    
    /* For reads, atomic aliases return the same value as base register */
    (void)atomic_op;

    /* GPIO status/ctrl: GPIOx_STATUS at 0x000 + 8*x, GPIOx_CTRL at 0x004 + 8*x */
    if (reg_offset < (RP2350_NUM_GPIOS * 8)) {
        int gpio = reg_offset / 8;
        int reg = reg_offset % 8;
        if (reg == 0) {
            /* STATUS register */
            val = s->gpio_status[gpio];
        } else if (reg == 4) {
            /* CTRL register */
            val = s->gpio_ctrl[gpio];
        }
    }
    /* Raw Interrupts: INTR[0-5] at 0x230-0x244 */
    else if (reg_offset >= IO_BANK0_INTR_BASE && reg_offset < IO_BANK0_INTR_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_INTR_BASE) / 4;
        val = s->intr[reg];
    }
    /* PROC0 Interrupt Enable: INTE[0-5] at 0x248-0x25C */
    else if (reg_offset >= IO_BANK0_PROC0_INTE_BASE && reg_offset < IO_BANK0_PROC0_INTE_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC0_INTE_BASE) / 4;
        val = s->proc0_inte[reg];
    }
    /* PROC0 Interrupt Force: INTF[0-5] at 0x260-0x274 */
    else if (reg_offset >= IO_BANK0_PROC0_INTF_BASE && reg_offset < IO_BANK0_PROC0_INTF_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC0_INTF_BASE) / 4;
        val = s->proc0_intf[reg];
    }
    /* PROC0 Interrupt Status: INTS[0-5] at 0x278-0x28C (read-only, computed) */
    else if (reg_offset >= IO_BANK0_PROC0_INTS_BASE && reg_offset < IO_BANK0_PROC0_INTS_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC0_INTS_BASE) / 4;
        /* INTS = (INTR & INTE) | INTF */
        val = (s->intr[reg] & s->proc0_inte[reg]) | s->proc0_intf[reg];
    }
    /* PROC1 Interrupt Enable: INTE[0-5] at 0x290-0x2A4 */
    else if (reg_offset >= IO_BANK0_PROC1_INTE_BASE && reg_offset < IO_BANK0_PROC1_INTE_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC1_INTE_BASE) / 4;
        val = s->proc1_inte[reg];
    }
    /* PROC1 Interrupt Force: INTF[0-5] at 0x2A8-0x2BC */
    else if (reg_offset >= IO_BANK0_PROC1_INTF_BASE && reg_offset < IO_BANK0_PROC1_INTF_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC1_INTF_BASE) / 4;
        val = s->proc1_intf[reg];
    }
    /* PROC1 Interrupt Status: INTS[0-5] at 0x2C0-0x2D4 (read-only, computed) */
    else if (reg_offset >= IO_BANK0_PROC1_INTS_BASE && reg_offset < IO_BANK0_PROC1_INTS_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC1_INTS_BASE) / 4;
        /* INTS = (INTR & INTE) | INTF */
        val = (s->intr[reg] & s->proc1_inte[reg]) | s->proc1_intf[reg];
    }
    /* Dormant Wake Interrupt Enable: INTE[0-5] at 0x2D8-0x2EC */
    else if (reg_offset >= IO_BANK0_DORMANT_WAKE_INTE_BASE && reg_offset < IO_BANK0_DORMANT_WAKE_INTE_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_DORMANT_WAKE_INTE_BASE) / 4;
        val = s->dormant_wake_inte[reg];
    }
    /* Dormant Wake Interrupt Force: INTF[0-5] at 0x2F0-0x304 */
    else if (reg_offset >= IO_BANK0_DORMANT_WAKE_INTF_BASE && reg_offset < IO_BANK0_DORMANT_WAKE_INTF_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_DORMANT_WAKE_INTF_BASE) / 4;
        val = s->dormant_wake_intf[reg];
    }
    /* Dormant Wake Interrupt Status: INTS[0-5] at 0x308-0x31C (read-only, computed) */
    else if (reg_offset >= IO_BANK0_DORMANT_WAKE_INTS_BASE && reg_offset < IO_BANK0_DORMANT_WAKE_INTS_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_DORMANT_WAKE_INTS_BASE) / 4;
        /* INTS = (INTR & INTE) | INTF */
        val = (s->intr[reg] & s->dormant_wake_inte[reg]) | s->dormant_wake_intf[reg];
    }
    else {
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
    
    /* Extract atomic operation type and base offset */
    int atomic_op = (addr >> 12) & 0x3;
    hwaddr reg_offset = addr & 0xFFF;

    /* GPIO status/ctrl: GPIOx_STATUS at 0x000 + 8*x, GPIOx_CTRL at 0x004 + 8*x */
    if (reg_offset < (RP2350_NUM_GPIOS * 8)) {
        int gpio = reg_offset / 8;
        int reg = reg_offset % 8;
        if (reg == 4) {
            /* CTRL register - writable with atomic ops */
            atomic_write_op(&s->gpio_ctrl[gpio], val, atomic_op);
        }
        /* STATUS register is read-only */
    }
    /* Raw Interrupts: INTR[0-5] at 0x230-0x244 - write 1 to clear edge events */
    else if (reg_offset >= IO_BANK0_INTR_BASE && reg_offset < IO_BANK0_INTR_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_INTR_BASE) / 4;
        /* Write 1 to clear: only clear bits for edge events (bits 2,3 per GPIO) */
        /* Each GPIO has 4 bits: [EDGE_HIGH, EDGE_LOW, LEVEL_HIGH, LEVEL_LOW] */
        /* Bits 3,2 are edge bits that can be cleared by writing 1 */
        uint32_t edge_mask = 0;
        for (int i = 0; i < 8; i++) {
            edge_mask |= (0xC << (i * 4)); /* bits 2,3 of each 4-bit field */
        }
        uint32_t clear_bits = val & edge_mask;
        /* For INTR, write-1-to-clear doesn't use atomic ops the same way */
        s->intr[reg] &= ~clear_bits;
        /* Also clear INTF bits that were set by GPIO injection (QEMU workaround) */
        s->proc0_intf[reg] &= ~clear_bits;
        s->proc1_intf[reg] &= ~clear_bits;
        
        /* Check if all INTS are now 0, and deassert IRQ if so */
        uint32_t proc0_ints = (s->intr[reg] & s->proc0_inte[reg]) | s->proc0_intf[reg];
        uint32_t proc1_ints = (s->intr[reg] & s->proc1_inte[reg]) | s->proc1_intf[reg];
        if (proc0_ints == 0 && s->io_irq_bank0[0]) {
            qemu_set_irq(s->io_irq_bank0[0], 0);
        }
        if (proc1_ints == 0 && s->io_irq_bank0[1]) {
            qemu_set_irq(s->io_irq_bank0[1], 0);
        }
    }
    /* PROC0 Interrupt Enable: INTE[0-5] at 0x248-0x25C */
    else if (reg_offset >= IO_BANK0_PROC0_INTE_BASE && reg_offset < IO_BANK0_PROC0_INTE_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC0_INTE_BASE) / 4;
        atomic_write_op(&s->proc0_inte[reg], val, atomic_op);
    }
    /* PROC0 Interrupt Force: INTF[0-5] at 0x260-0x274 */
    else if (reg_offset >= IO_BANK0_PROC0_INTF_BASE && reg_offset < IO_BANK0_PROC0_INTF_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC0_INTF_BASE) / 4;
        atomic_write_op(&s->proc0_intf[reg], val, atomic_op);
    }
    /* PROC0 INTS is read-only */
    else if (reg_offset >= IO_BANK0_PROC0_INTS_BASE && reg_offset < IO_BANK0_PROC0_INTS_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        /* Ignore writes to read-only register */
    }
    /* PROC1 Interrupt Enable: INTE[0-5] at 0x290-0x2A4 */
    else if (reg_offset >= IO_BANK0_PROC1_INTE_BASE && reg_offset < IO_BANK0_PROC1_INTE_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC1_INTE_BASE) / 4;
        atomic_write_op(&s->proc1_inte[reg], val, atomic_op);
    }
    /* PROC1 Interrupt Force: INTF[0-5] at 0x2A8-0x2BC */
    else if (reg_offset >= IO_BANK0_PROC1_INTF_BASE && reg_offset < IO_BANK0_PROC1_INTF_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_PROC1_INTF_BASE) / 4;
        atomic_write_op(&s->proc1_intf[reg], val, atomic_op);
    }
    /* PROC1 INTS is read-only */
    else if (reg_offset >= IO_BANK0_PROC1_INTS_BASE && reg_offset < IO_BANK0_PROC1_INTS_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        /* Ignore writes to read-only register */
    }
    /* Dormant Wake Interrupt Enable: INTE[0-5] at 0x2D8-0x2EC */
    else if (reg_offset >= IO_BANK0_DORMANT_WAKE_INTE_BASE && reg_offset < IO_BANK0_DORMANT_WAKE_INTE_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_DORMANT_WAKE_INTE_BASE) / 4;
        atomic_write_op(&s->dormant_wake_inte[reg], val, atomic_op);
    }
    /* Dormant Wake Interrupt Force: INTF[0-5] at 0x2F0-0x304 */
    else if (reg_offset >= IO_BANK0_DORMANT_WAKE_INTF_BASE && reg_offset < IO_BANK0_DORMANT_WAKE_INTF_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        int reg = (reg_offset - IO_BANK0_DORMANT_WAKE_INTF_BASE) / 4;
        atomic_write_op(&s->dormant_wake_intf[reg], val, atomic_op);
    }
    /* Dormant Wake INTS is read-only */
    else if (reg_offset >= IO_BANK0_DORMANT_WAKE_INTS_BASE && reg_offset < IO_BANK0_DORMANT_WAKE_INTS_BASE + IO_BANK0_NUM_IRQ_REGS * 4) {
        /* Ignore writes to read-only register */
    }
    else {
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

/* ========== ADC - Analog-to-Digital Converter ========== */

static uint64_t rp2350_adc_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350ADCState *s = opaque;
    uint64_t val = 0;

    switch (addr) {
    case 0x00:  /* CS - Control/Status */
        val = s->cs;
        break;
    case 0x04:  /* RESULT - Most recent conversion result (read-only) */
        val = s->result & 0xfff;
        break;
    case 0x08:  /* FCS - FIFO Control/Status */
        val = s->fcs & ~(0x0f << 16);  /* Clear LEVEL field */
        val |= (s->fifo_count & 0x0f) << 16;  /* Set LEVEL from current FIFO count */
        /* Set FULL/EMPTY bits */
        if (s->fifo_count == 0) {
            val |= (1 << 8);  /* EMPTY */
        }
        if (s->fifo_count >= RP2350_ADC_FIFO_SIZE) {
            val |= (1 << 9);  /* FULL */
        }
        break;
    case 0x0c:  /* FIFO - FIFO read (pop value) */
        if (s->fifo_count > 0) {
            val = s->fifo_data[s->fifo_r];
            s->fifo_r = (s->fifo_r + 1) % RP2350_ADC_FIFO_SIZE;
            s->fifo_count--;
        }
        break;
    case 0x10:  /* DIV - Clock divider */
        val = s->div;
        break;
    case 0x14:  /* INTR - Raw interrupt status */
        val = s->intr & 0x1;
        break;
    case 0x18:  /* INTE - Interrupt enable */
        val = s->inte & 0x1;
        break;
    case 0x1c:  /* INTF - Interrupt force */
        val = s->intf & 0x1;
        break;
    case 0x20:  /* INTS - Interrupt status (after masking) */
        val = ((s->intr & s->inte) | s->intf) & 0x1;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_adc_read: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
    return val;
}

static void rp2350_adc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RP2350ADCState *s = opaque;
    val &= UINT32_MAX;

    switch (addr) {
    case 0x00:  /* CS - Control/Status */
        s->cs = (s->cs & 0x00000100) | (val & 0x01fff70f);  /* Keep READY bit, update rest */
        break;
    case 0x08:  /* FCS - FIFO Control/Status */
        s->fcs = val & 0x0f0f0f0f;
        break;
    case 0x10:  /* DIV - Clock divider */
        s->div = val & 0x00ffffff;
        break;
    case 0x14:  /* INTR - Write-to-clear raw interrupt */
        s->intr &= ~(val & 0x1);
        break;
    case 0x18:  /* INTE - Interrupt enable */
        s->inte = val & 0x1;
        break;
    case 0x1c:  /* INTF - Interrupt force */
        s->intf = val & 0x1;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_adc_write: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
}

static const MemoryRegionOps rp2350_adc_ops = {
    .read = rp2350_adc_read,
    .write = rp2350_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== DMA - Direct Memory Access ========== */

static uint64_t rp2350_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350DMAState *s = opaque;
    uint64_t val = 0;
    
    /* DMA channel register (addr 0x00-0x2ff for 16 channels, 0x40 bytes each) */
    int channel_idx = addr / 0x40;
    int reg_offset = addr % 0x40;
    
    if (channel_idx < RP2350_NUM_DMA_CHANNELS) {
        RP2350DMAChannelState *ch = &s->channel[channel_idx];
        
        switch (reg_offset) {
        case 0x00:  /* READ_ADDR */
            val = ch->read_addr;
            break;
        case 0x04:  /* WRITE_ADDR */
            val = ch->write_addr;
            break;
        case 0x08:  /* TRANS_COUNT */
            val = ch->trans_count;
            break;
        case 0x0c:  /* CTRL_TRIG */
            val = ch->ctrl_trig;
            break;
        default:
            /* Alias registers - treat as CTRL_TRIG for now */
            if (reg_offset >= 0x10 && reg_offset < 0x40) {
                val = ch->ctrl_trig;  /* Simplified: alias behavior */
            }
        }
        return val;
    }
    
    /* Global DMA registers (above channels) */
    addr -= 0x400;  /* Start of global registers after all channel data */
    
    switch (addr) {
    case 0x00:  /* INTR - Raw interrupt status */
        val = s->intr & 0xffff;
        break;
    case 0x04:  /* INTE0 - Interrupt enable for IRQ 0 */
        val = s->inte0 & 0xffff;
        break;
    case 0x08:  /* INTF0 - Interrupt force */
        val = s->intf0 & 0xffff;
        break;
    case 0x0c:  /* INTS0 - Interrupt status */
        val = ((s->intr & s->inte0) | s->intf0) & 0xffff;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_dma_read: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
    
    return val;
}

static void rp2350_dma_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RP2350DMAState *s = opaque;
    val &= UINT32_MAX;
    
    /* DMA channel register */
    int channel_idx = addr / 0x40;
    int reg_offset = addr % 0x40;
    
    if (channel_idx < RP2350_NUM_DMA_CHANNELS) {
        RP2350DMAChannelState *ch = &s->channel[channel_idx];
        
        switch (reg_offset) {
        case 0x00:  /* READ_ADDR */
            ch->read_addr = val;
            break;
        case 0x04:  /* WRITE_ADDR */
            ch->write_addr = val;
            break;
        case 0x08:  /* TRANS_COUNT */
            ch->trans_count = val;
            break;
        case 0x0c:  /* CTRL_TRIG */
            ch->ctrl_trig = val & 0xe7ffffff;
            /* TODO: Trigger DMA transfer if EN bit set */
            break;
        default:
            /* Alias registers */
            if (reg_offset >= 0x10 && reg_offset < 0x40) {
                ch->ctrl_trig = val;  /* Simplified */
            }
        }
        return;
    }
    
    /* Global DMA registers */
    addr -= 0x400;
    
    switch (addr) {
    case 0x00:  /* INTR - Write-to-clear raw interrupt */
        s->intr &= ~(val & 0xffff);
        break;
    case 0x04:  /* INTE0 - Interrupt enable */
        s->inte0 = val & 0xffff;
        break;
    case 0x08:  /* INTF0 - Interrupt force */
        s->intf0 = val & 0xffff;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "rp2350_dma_write: unimplemented offset 0x%"HWADDR_PRIx"\n",
                      addr);
    }
}

static const MemoryRegionOps rp2350_dma_ops = {
    .read = rp2350_dma_read,
    .write = rp2350_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== PWM - Pulse Width Modulation ========== */

static uint64_t rp2350_pwm_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350PWMState *s = opaque;
    uint64_t val = 0;
    
    /* Handle atomic register aliases */
    /* In RP2040/RP2350, registers at base address have atomic aliases at +0x1000/+0x2000/+0x3000 */
    /* When firmware reads from alias address, we return the base register value */
    /* The actual read still happens from the base address, aliases only affect writes */
    hwaddr base_offset = addr & 0xfff;     /* Extract actual register offset [11:0] */
    
    /* Reads always return the base register value, regardless of which alias was used */
    addr = base_offset;
    
    /* PWM slice registers (each slice is 0x14 bytes: csr, div, ctr, cc, top) */
    int slice_idx = addr / 0x14;
    int reg_offset = addr % 0x14;
    
    if (slice_idx < RP2350_NUM_PWM_SLICES) {
        RP2350PWMSliceState *slice = &s->slice[slice_idx];
        
        switch (reg_offset) {
        case 0x00:  /* CSR - Control/Status */
            val = slice->csr & 0xff;
            break;
        case 0x04:  /* DIV - Clock divider */
            val = slice->div & 0xfff;
            break;
        case 0x08:  /* CTR - Counter */
            val = slice->ctr & 0xffff;
            break;
        case 0x0c:  /* CC - Compare A/B */
            val = slice->cc;
            break;
        case 0x10:  /* TOP - Wrap value */
            val = slice->top & 0xffff;
            break;
        default:
            /* Unimplemented registers */
            break;
        }
        return val;
    }
    
    /* Global PWM interrupt registers at fixed addresses */
    /* (RP2350 has EN register at 0xf0, INTR at 0xf4, then IRQ0 regs) */
    switch (addr) {
    case 0x0f0:  /* EN - global enable for all channels */
        val = s->en & 0xfff;
        break;
    case 0x0f4:  /* INTR - Raw interrupt status */
        val = s->intr & 0xfff;
        break;
    case 0x0f8:  /* IRQ0_INTE - Interrupt enable */
        val = s->inte & 0xfff;
        break;
    case 0x0fc:  /* IRQ0_INTF - Interrupt force */
        val = s->intf & 0xfff;
        break;
    case 0x100:  /* IRQ0_INTS - Interrupt status */
        val = ((s->intr & s->inte) | s->intf) & 0xfff;
        break;
    default:
        break;
    }
    
    return val;
}

static void rp2350_pwm_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RP2350PWMState *s = opaque;
    val &= UINT32_MAX;
    
    /* Extract the alias type from the upper address bits (RP2040 alias scheme) */
    /* Aliases: RW (0x0000), XOR (0x1000), SET (0x2000), CLR (0x3000) */
    int alias_type = (addr >> 12) & 0x3;  /* Bits 13:12 select the alias */
    hwaddr base_addr = addr & 0xfff;      /* Bits 11:0 are the actual register address */
    
    /* Base address is where we actually read/write */
    addr = base_addr;
    
    /* PWM slice registers (each slice is 0x14 bytes) */
    int slice_idx = addr / 0x14;
    int reg_offset = addr % 0x14;
    
    if (slice_idx < RP2350_NUM_PWM_SLICES) {
        RP2350PWMSliceState *slice = &s->slice[slice_idx];
        uint32_t old_val;
        
        switch (reg_offset) {
        case 0x00:  /* CSR - Control/Status */
            old_val = slice->csr;
            switch (alias_type) {
            case 0:  /* RW - Normal write */
                slice->csr = val & 0xff;
                break;
            case 1:  /* XOR - Toggle bits */
                slice->csr = (old_val ^ val) & 0xff;
                break;
            case 2:  /* SET - Set bits (OR) */
                slice->csr = (old_val | val) & 0xff;
                break;
            case 3:  /* CLR - Clear bits (AND NOT) */
                slice->csr = (old_val & ~val) & 0xff;
                break;
            }
            break;
        case 0x04:  /* DIV - Clock divider */
            old_val = slice->div;
            switch (alias_type) {
            case 0:  /* RW - Normal write */
                slice->div = val & 0xfff;
                break;
            case 1:  /* XOR - Toggle bits */
                slice->div = (old_val ^ val) & 0xfff;
                break;
            case 2:  /* SET - Set bits (OR) */
                slice->div = (old_val | val) & 0xfff;
                break;
            case 3:  /* CLR - Clear bits (AND NOT) */
                slice->div = (old_val & ~val) & 0xfff;
                break;
            }
            break;
        case 0x08:  /* CTR - Counter */
            old_val = slice->ctr;
            switch (alias_type) {
            case 0:  /* RW - Normal write */
                slice->ctr = val & 0xffff;
                break;
            case 1:  /* XOR - Toggle bits */
                slice->ctr = (old_val ^ val) & 0xffff;
                break;
            case 2:  /* SET - Set bits (OR) */
                slice->ctr = (old_val | val) & 0xffff;
                break;
            case 3:  /* CLR - Clear bits (AND NOT) */
                slice->ctr = (old_val & ~val) & 0xffff;
                break;
            }
            break;
        case 0x0c:  /* CC - Compare A/B */
            old_val = slice->cc;
            switch (alias_type) {
            case 0:  /* RW - Normal write */
                slice->cc = val;
                break;
            case 1:  /* XOR - Toggle bits */
                slice->cc = old_val ^ val;
                break;
            case 2:  /* SET - Set bits (OR) */
                slice->cc = old_val | val;
                break;
            case 3:  /* CLR - Clear bits (AND NOT) */
                slice->cc = old_val & ~val;
                break;
            }
            break;
        case 0x10:  /* TOP - Wrap value */
            old_val = slice->top;
            switch (alias_type) {
            case 0:  /* RW - Normal write */
                slice->top = val & 0xffff;
                break;
            case 1:  /* XOR - Toggle bits */
                slice->top = (old_val ^ val) & 0xffff;
                break;
            case 2:  /* SET - Set bits (OR) */
                slice->top = (old_val | val) & 0xffff;
                break;
            case 3:  /* CLR - Clear bits (AND NOT) */
                slice->top = (old_val & ~val) & 0xffff;
                break;
            }
            break;
        default:
            break;
        }
        return;
    }
    
    /* Global PWM registers at fixed addresses */
    switch (addr) {
    case 0x0f0:  /* EN - global enable for all channels */
        switch (alias_type) {
        case 0:  s->en = val & 0xfff; break;
        case 1:  s->en = (s->en ^ val) & 0xfff; break;
        case 2:  s->en = (s->en | val) & 0xfff; break;
        case 3:  s->en = (s->en & ~val) & 0xfff; break;
        }
        break;
    case 0x0f4:  /* INTR - Write-to-clear raw interrupt */
        s->intr &= ~(val & 0xfff);
        break;
    case 0x0f8:  /* IRQ0_INTE - Interrupt enable */
        switch (alias_type) {
        case 0:  s->inte = val & 0xfff; break;
        case 1:  s->inte = (s->inte ^ val) & 0xfff; break;
        case 2:  s->inte = (s->inte | val) & 0xfff; break;
        case 3:  s->inte = (s->inte & ~val) & 0xfff; break;
        }
        break;
    case 0x0fc:  /* IRQ0_INTF - Interrupt force */
        switch (alias_type) {
        case 0:  s->intf = val & 0xfff; break;
        case 1:  s->intf = (s->intf ^ val) & 0xfff; break;
        case 2:  s->intf = (s->intf | val) & 0xfff; break;
        case 3:  s->intf = (s->intf & ~val) & 0xfff; break;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps rp2350_pwm_ops = {
    .read = rp2350_pwm_read,
    .write = rp2350_pwm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== SPI - Synchronous Serial Port (PL022 wrapper with atomic aliases) ========== */
/* RP2350 uses PL022 SSP devices. We need a wrapper to handle atomic register aliases
 * (XOR/SET/CLR) since the Pico SDK uses hw_set_bits/hw_clear_bits for SPI init. */

/* PL022 register offsets */
#define PL022_CR0       0x00
#define PL022_CR1       0x04
#define PL022_DR        0x08
#define PL022_SR        0x0c
#define PL022_CPSR      0x10
#define PL022_IMSC      0x14
#define PL022_RIS       0x18
#define PL022_MIS       0x1c
#define PL022_ICR       0x20
#define PL022_DMACR     0x24

/* We maintain shadow copies of writable registers for atomic alias support */
typedef struct RP2350SPIRegs {
    uint32_t cr0;
    uint32_t cr1;
    uint32_t cpsr;
    uint32_t imsc;
    uint32_t dmacr;
} RP2350SPIRegs;

static RP2350SPIRegs spi_shadow[2];  /* Shadow registers for SPI0 and SPI1 */

static uint64_t rp2350_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    RP2350SPIState *s = (RP2350SPIState *)opaque;
    hwaddr base_offset = addr & 0xfff;  /* Strip alias bits */
    RP2350SPIRegs *regs = &spi_shadow[s->index];
    uint64_t val = 0;
    MemOp op = MO_32;

    /* For reads, always return from shadow registers (which stay in sync with PL022) */
    switch (base_offset) {
    case PL022_CR0:
        return regs->cr0;
    case PL022_CR1:
        return regs->cr1;
    case PL022_CPSR:
        return regs->cpsr;
    case PL022_IMSC:
        return regs->imsc;
    case PL022_DMACR:
        return regs->dmacr;
    default:
        /* For other registers (DR, SR, RIS, MIS, etc.), read directly from PL022 */
        memory_region_dispatch_read(&s->pl022->iomem, base_offset,
                                   &val, op, MEMTXATTRS_UNSPECIFIED);
        return val;
    }
}

static void rp2350_spi_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    RP2350SPIState *s = (RP2350SPIState *)opaque;
    int alias_type = (addr >> 12) & 0x3;  /* Bits 13:12: 0=RW, 1=XOR, 2=SET, 3=CLR */
    hwaddr base_offset = addr & 0xfff;
    RP2350SPIRegs *regs = &spi_shadow[s->index];
    uint32_t *shadow_reg = NULL;
    uint32_t new_val;

    /* Determine which shadow register to use */
    switch (base_offset) {
    case PL022_CR0:
        shadow_reg = &regs->cr0;
        break;
    case PL022_CR1:
        shadow_reg = &regs->cr1;
        break;
    case PL022_CPSR:
        shadow_reg = &regs->cpsr;
        break;
    case PL022_IMSC:
        shadow_reg = &regs->imsc;
        break;
    case PL022_DMACR:
        shadow_reg = &regs->dmacr;
        break;
    default:
        /* For other registers (DR, ICR), write directly to PL022 */
        memory_region_dispatch_write(&s->pl022->iomem, base_offset,
                                     val, MO_32, MEMTXATTRS_UNSPECIFIED);
        return;
    }

    /* Apply atomic operation based on alias type */
    switch (alias_type) {
    case 0:  /* Normal write */
        new_val = val;
        break;
    case 1:  /* XOR */
        new_val = *shadow_reg ^ val;
        break;
    case 2:  /* SET (OR) */
        new_val = *shadow_reg | val;
        break;
    case 3:  /* CLR (AND NOT) */
        new_val = *shadow_reg & ~val;
        break;
    default:
        new_val = val;
        break;
    }

    /* Update shadow register */
    *shadow_reg = new_val;

    /* Write to underlying PL022 device */
    memory_region_dispatch_write(&s->pl022->iomem, base_offset,
                                 new_val, MO_32, MEMTXATTRS_UNSPECIFIED);
}

static const MemoryRegionOps rp2350_spi_ops = {
    .read = rp2350_spi_read,
    .write = rp2350_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ========== TIMER - Microsecond Timer (struct only) ========== */
typedef struct RP2350TimerState RP2350TimerState;
typedef struct RP2350State RP2350State;

struct RP2350TimerState {
    MemoryRegion iomem;
    int64_t start_time_ns;  /* QEMU clock time when timer started */
    uint32_t alarm[4];
    uint32_t armed;
    uint32_t pause;
    uint32_t inte;
    uint32_t intf;
    uint32_t latched_hi;    /* For atomic 64-bit reads */
    QEMUTimer *alarm_timer[4];  /* QEMU timers for each alarm */
    qemu_irq irq[4];            /* IRQ lines for each alarm */
    RP2350State *parent;        /* Back pointer to parent state */
    int timer_index;            /* 0 for timer0, 1 for timer1 */
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

    RP2350SIOState sio;
    RP2350IOBank0State io_bank0;
    RP2350PadsBank0State pads_bank0;
    RP2350TimerState timer0;
    RP2350TimerState timer1;
    RP2350ADCState adc;
    RP2350DMAState dma;
    RP2350PWMState pwm;
    
    /* SPI peripherals (PL022 with atomic alias wrapper) */
    RP2350SPIState spi[2];
    
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

    /* PSM (Power State Machine) peripheral */
    MemoryRegion psm;
    uint32_t psm_frce_on;
    uint32_t psm_frce_off;
};

/* PSM (Power State Machine) - controls power domains including core1 */
#define PSM_FRCE_ON_OFFSET      0x00
#define PSM_FRCE_OFF_OFFSET     0x04
#define PSM_WDSEL_OFFSET        0x08
#define PSM_DONE_OFFSET         0x0c
#define PSM_PROC1_BITS          (1 << 24)

static uint64_t rp2350_psm_read(void *opaque, hwaddr offset, unsigned size)
{
    RP2350State *s = opaque;
    uint32_t base_offset = offset & 0xfff;
    uint64_t val = 0;
    
    /* Handle atomic aliases */
    uint32_t alias = (offset >> 12) & 0x3;
    if (alias != 0) {
        base_offset = offset & 0xfff;
    }
    
    switch (base_offset) {
    case PSM_FRCE_ON_OFFSET:
        val = s->psm_frce_on;
        break;
    case PSM_FRCE_OFF_OFFSET:
        val = s->psm_frce_off;
        break;
    case PSM_WDSEL_OFFSET:
        val = 0;
        break;
    case PSM_DONE_OFFSET:
        /* Report all power domains ready, except those force-off */
        val = 0x01ffffff & ~s->psm_frce_off;
        break;
    default:
        val = 0;
        break;
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, "PSM read: offset=0x%x alias=%d -> 0x%lx\n",
                  base_offset, alias, (unsigned long)val);
    return val;
}

static void rp2350_psm_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    RP2350State *s = opaque;
    uint32_t base_offset = offset & 0xfff;
    uint32_t alias = (offset >> 12) & 0x3;
    uint32_t old_frce_off = s->psm_frce_off;
    /* RP2350 atomic aliases: 0=normal, 1=XOR, 2=SET, 3=CLR */
    
    qemu_log_mask(LOG_GUEST_ERROR, "PSM write: offset=0x%x alias=%d value=0x%lx\n",
                  base_offset, alias, (unsigned long)value);
    
    switch (base_offset) {
    case PSM_FRCE_ON_OFFSET:
        switch (alias) {
        case 0: s->psm_frce_on = value; break;
        case 1: s->psm_frce_on ^= value; break;  /* XOR */
        case 2: s->psm_frce_on |= value; break;  /* SET */
        case 3: s->psm_frce_on &= ~value; break; /* CLR */
        }
        /* If PROC1 is force-on, wake it up (clear force-off) */
        if (s->psm_frce_on & PSM_PROC1_BITS) {
            s->psm_frce_off &= ~PSM_PROC1_BITS;
        }
        break;
    case PSM_FRCE_OFF_OFFSET:
        switch (alias) {
        case 0: s->psm_frce_off = value; break;
        case 1: s->psm_frce_off ^= value; break;  /* XOR */
        case 2: s->psm_frce_off |= value; break;  /* SET */
        case 3: s->psm_frce_off &= ~value; break; /* CLR */
        }
        /* Handle PROC1 power state transitions */
        if ((s->psm_frce_off & PSM_PROC1_BITS) && !(old_frce_off & PSM_PROC1_BITS)) {
            /* PROC1 is being turned OFF - halt the CPU properly.
             * Only call cpu_exit if the CPU has been created/started,
             * otherwise just mark as halted.
             */
            CPUState *cs = CPU(s->cpu[1].cpu);
            if (cs) {
                cs->halted = 1;
                cs->stop = true;
                /* Only try to kick the CPU if it has a valid thread */
                if (cs->thread) {
                    cpu_exit(cs);
                }
                /* Reset the launch state machine so core1 can be relaunched */
                s->sio.launch_state = 0;
            }
        } else if (!(s->psm_frce_off & PSM_PROC1_BITS) && (old_frce_off & PSM_PROC1_BITS)) {
            /* PROC1 is being turned ON from reset state 
             * Simulate bootrom behavior: drain FIFO and push 0 to core0's FIFO.
             * The real bootrom does this when core1 comes out of reset.
             */
            /* Reset the SIO launch state machine */
            s->sio.launch_state = 0;
            /* Drain the TX FIFO (core0->core1 direction) */
            s->sio.tx_r = s->sio.tx_w = s->sio.tx_count = 0;
            /* Push 0 to the RX FIFO (core1->core0 direction, simulating bootrom handshake) */
            if (s->sio.rx_count < 4) {
                s->sio.rx_fifo[s->sio.rx_w] = 0;
                s->sio.rx_w = (s->sio.rx_w + 1) & 3;
                s->sio.rx_count++;
            }
        }
        break;
    case PSM_WDSEL_OFFSET:
        /* Watchdog select - ignore for now */
        break;
    default:
        break;
    }
}

static const MemoryRegionOps rp2350_psm_ops = {
    .read = rp2350_psm_read,
    .write = rp2350_psm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
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
#define RP2350_TIMER_SIZE       0x4000  /* Include atomic aliases: normal, SET, CLR, XOR */

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

/* Timer IRQ numbers for RP2350 (from datasheet) */
#define TIMER0_IRQ_0        0
#define TIMER0_IRQ_1        1
#define TIMER0_IRQ_2        2
#define TIMER0_IRQ_3        3
#define TIMER1_IRQ_0        4
#define TIMER1_IRQ_1        5
#define TIMER1_IRQ_2        6
#define TIMER1_IRQ_3        7

/* Forward declaration */
static void rp2350_timer_update_irq(RP2350TimerState *s);
static uint64_t timer_get_time_us(RP2350TimerState *s);

/* Timer alarm callback - called when a QEMU timer expires */
static void rp2350_timer_alarm_cb(void *opaque)
{
    RP2350TimerState *s = opaque;
    uint64_t now_us = timer_get_time_us(s);
    int i;
    
    /* Check each alarm */
    for (i = 0; i < 4; i++) {
        if (s->armed & (1 << i)) {
            /* Check if alarm time has been reached (compare low 32 bits) */
            if ((uint32_t)now_us >= s->alarm[i]) {
                /* Alarm fired - set interrupt flag and disarm */
                s->intf |= (1 << i);
                s->armed &= ~(1 << i);
            }
        }
    }
    
    /* Update interrupt line */
    rp2350_timer_update_irq(s);
    
    /* Reschedule if any alarms are still armed */
    if (s->armed) {
        /* Find the nearest alarm */
        uint32_t next_alarm = UINT32_MAX;
        for (i = 0; i < 4; i++) {
            if ((s->armed & (1 << i)) && s->alarm[i] < next_alarm) {
                next_alarm = s->alarm[i];
            }
        }
        if (next_alarm != UINT32_MAX) {
            /* Schedule next callback - convert from microseconds to nanoseconds */
            int64_t target_ns = s->start_time_ns + (int64_t)next_alarm * 1000;
            timer_mod(s->alarm_timer[0], target_ns);
        }
    }
}

/* Update timer alarm scheduling */
static void rp2350_timer_schedule_alarm(RP2350TimerState *s)
{
    if (!s->armed) {
        return;
    }
    
    /* Find the nearest alarm */
    uint32_t next_alarm = UINT32_MAX;
    int i;
    for (i = 0; i < 4; i++) {
        if ((s->armed & (1 << i)) && s->alarm[i] < next_alarm) {
            next_alarm = s->alarm[i];
        }
    }
    
    if (next_alarm != UINT32_MAX) {
        /* Schedule callback - convert from microseconds to nanoseconds */
        int64_t target_ns = s->start_time_ns + (int64_t)next_alarm * 1000;
        timer_mod(s->alarm_timer[0], target_ns);
    }
}

/* Update interrupt line based on INTE, INTF */
static void rp2350_timer_update_irq(RP2350TimerState *s)
{
    uint32_t ints = s->inte & s->intf;
    int i;
    
    /* Each alarm has its own IRQ line */
    for (i = 0; i < 4; i++) {
        if (s->irq[i]) {
            qemu_set_irq(s->irq[i], (ints >> i) & 1);
        }
    }
}

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

    /* Strip atomic alias bits - reads return same value regardless of alias */
    hwaddr reg_offset = offset & 0xFFF;

    switch (reg_offset) {
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

    /* Determine atomic operation type from address bits [13:12] */
    int atomic_op = (offset >> 12) & 0x3;
    hwaddr reg_offset = offset & 0xFFF;

    /* Helper macro for atomic operations on a register */
    #define ATOMIC_WRITE(reg, newval) do { \
        switch (atomic_op) { \
        case 0: (reg) = (newval); break;       /* Normal write */ \
        case 1: (reg) |= (newval); break;      /* SET */ \
        case 2: (reg) &= ~(newval); break;     /* CLR */ \
        case 3: (reg) ^= (newval); break;      /* XOR */ \
        } \
    } while(0)

    switch (reg_offset) {
    case TIMER_ALARM0:
    case TIMER_ALARM1:
    case TIMER_ALARM2:
    case TIMER_ALARM3:
        ATOMIC_WRITE(s->alarm[(reg_offset - TIMER_ALARM0) / 4], val);
        s->armed |= (1 << ((reg_offset - TIMER_ALARM0) / 4));
        /* Schedule the alarm */
        rp2350_timer_schedule_alarm(s);
        break;
    case TIMER_ARMED:
        /* Writing 1 disarms the alarm */
        s->armed &= ~val;
        break;
    case TIMER_PAUSE:
        ATOMIC_WRITE(s->pause, val & 1);
        break;
    case TIMER_INTE:
        ATOMIC_WRITE(s->inte, val & 0xf);
        rp2350_timer_update_irq(s);
        break;
    case TIMER_INTF:
        ATOMIC_WRITE(s->intf, val & 0xf);
        rp2350_timer_update_irq(s);
        break;
    case TIMER_INTR:
        /* Write to clear interrupts */
        s->intf &= ~val;
        rp2350_timer_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "TIMER write: offset=0x%lx val=0x%lx\n",
                      (unsigned long)reg_offset, (unsigned long)val);
        break;
    }

    #undef ATOMIC_WRITE
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

    /* PSM (Power State Machine) peripheral */
    s->psm_frce_on = 0;
    s->psm_frce_off = 0;
    memory_region_init_io(&s->psm, NULL, &rp2350_psm_ops, s,
                          "rp2350.psm", RP2350_PSM_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_PSM_BASE, &s->psm);

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

        /* 
         * Set initial vector table address for Cortex-M33 TrustZone.
         * Without bootrom, firmware loads directly at 0x10000000 (XIP flash).
         * Core 0 boots from there; Core 1 VTOR is set via SIO FIFO during launch.
         */
        if (n == 0) {
            /* Core 0: Vector table at flash base (where firmware.elf is loaded) */
            qdev_prop_set_uint32(DEVICE(&s->cpu[n]), "init-svtor", RP2350_FLASH_BASE);
            qdev_prop_set_uint32(DEVICE(&s->cpu[n]), "init-nsvtor", RP2350_FLASH_BASE);
        } else {
            /* Core 1: Starts halted, VTOR set via SIO FIFO during launch */
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
    s->sio.parent = s;  /* Back pointer for GPIO injection */
    s->sio.io_irq_bank0[0] = qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_IO_IRQ_BANK0);
    s->sio.io_irq_bank0[1] = qdev_get_gpio_in(DEVICE(&s->cpu[1]), RP2350_IO_IRQ_BANK0);
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
    
    /* Connect SIO to IO_BANK0 for GPIO injection */
    s->sio.io_bank0 = &s->io_bank0;
    
    /* Set up IO_BANK0 IRQ lines for deassertion */
    s->io_bank0.io_irq_bank0[0] = qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_IO_IRQ_BANK0);
    s->io_bank0.io_irq_bank0[1] = qdev_get_gpio_in(DEVICE(&s->cpu[1]), RP2350_IO_IRQ_BANK0);

    /* PADS_BANK0 - GPIO pad control */
    pads_bank0_reset(&s->pads_bank0);
    memory_region_init_io(&s->pads_bank0.iomem, OBJECT(machine), &rp2350_pads_bank0_ops,
                          &s->pads_bank0, "rp2350.pads_bank0", RP2350_PADS_BANK0_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_PADS_BANK0_BASE,
                                &s->pads_bank0.iomem);

    /* TIMER0 - Microsecond timer */
    rp2350_timer_reset(&s->timer0);
    s->timer0.parent = s;
    s->timer0.timer_index = 0;
    /* Create QEMU timer for alarm callbacks - use just one timer for all 4 alarms */
    s->timer0.alarm_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                             rp2350_timer_alarm_cb, &s->timer0);
    /* Connect IRQ lines to NVIC - Timer0 uses IRQs 0-3 */
    for (int i = 0; i < 4; i++) {
        s->timer0.irq[i] = qdev_get_gpio_in(DEVICE(&s->cpu[0]), TIMER0_IRQ_0 + i);
    }
    memory_region_init_io(&s->timer0.iomem, OBJECT(machine), &rp2350_timer_ops,
                          &s->timer0, "rp2350.timer0", RP2350_TIMER_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_TIMER0_BASE,
                                &s->timer0.iomem);

    /* TIMER1 - Microsecond timer */
    rp2350_timer_reset(&s->timer1);
    s->timer1.parent = s;
    s->timer1.timer_index = 1;
    /* Create QEMU timer for alarm callbacks */
    s->timer1.alarm_timer[0] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                             rp2350_timer_alarm_cb, &s->timer1);
    /* Connect IRQ lines to NVIC - Timer1 uses IRQs 4-7 */
    for (int i = 0; i < 4; i++) {
        s->timer1.irq[i] = qdev_get_gpio_in(DEVICE(&s->cpu[0]), TIMER1_IRQ_0 + i);
    }
    memory_region_init_io(&s->timer1.iomem, OBJECT(machine), &rp2350_timer_ops,
                          &s->timer1, "rp2350.timer1", RP2350_TIMER_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_TIMER1_BASE,
                                &s->timer1.iomem);

    /* ADC - Analog-to-Digital Converter */
    memset(&s->adc, 0, sizeof(RP2350ADCState));
    s->adc.parent = s;
    s->adc.fifo_r = s->adc.fifo_w = s->adc.fifo_count = 0;
    /* ADC starts with READY bit set */
    s->adc.cs |= (1 << 8);  /* READY */
    memory_region_init_io(&s->adc.iomem, OBJECT(machine), &rp2350_adc_ops,
                          &s->adc, "rp2350.adc", RP2350_ADC_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_ADC_BASE,
                                &s->adc.iomem);

    /* DMA - Direct Memory Access */
    memset(&s->dma, 0, sizeof(RP2350DMAState));
    s->dma.parent = s;
    s->dma.adc = &s->adc;  /* Connect ADC for DREQ handling */
    for (int i = 0; i < RP2350_NUM_DMA_CHANNELS; i++) {
        s->dma.channel[i].read_addr = 0;
        s->dma.channel[i].write_addr = 0;
        s->dma.channel[i].trans_count = 0;
        s->dma.channel[i].ctrl_trig = 0;
    }
    memory_region_init_io(&s->dma.iomem, OBJECT(machine), &rp2350_dma_ops,
                          &s->dma, "rp2350.dma", RP2350_DMA_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_DMA_BASE,
                                &s->dma.iomem);

    /* PWM - Pulse Width Modulation */
    memset(&s->pwm, 0, sizeof(RP2350PWMState));
    s->pwm.parent = s;
    for (int i = 0; i < RP2350_NUM_PWM_SLICES; i++) {
        s->pwm.slice[i].csr = 0;
        s->pwm.slice[i].div = (1 << 4);  /* Default divider = 1 */
        s->pwm.slice[i].ctr = 0;
        s->pwm.slice[i].cc = 0;
        s->pwm.slice[i].top = 0xffff;  /* Default top = 65535 */
    }
    s->pwm.inte = 0;
    s->pwm.intf = 0;
    s->pwm.intr = 0;
    memory_region_init_io(&s->pwm.iomem, OBJECT(machine), &rp2350_pwm_ops,
                          &s->pwm, "rp2350.pwm", RP2350_PWM_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_PWM_BASE,
                                &s->pwm.iomem);

    /* SPI0 - PL022 Synchronous Serial Port with atomic alias wrapper */
    s->spi[0].pl022 = PL022(qdev_new(TYPE_PL022));
    s->spi[0].index = 0;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->spi[0].pl022), &error_fatal);
    /* Don't use sysbus_mmio_map - we wrap PL022 with our own region for atomic aliases */
    memset(&spi_shadow[0], 0, sizeof(RP2350SPIRegs));
    memory_region_init_io(&s->spi[0].iomem, OBJECT(machine), &rp2350_spi_ops,
                          &s->spi[0], "rp2350.spi0", RP2350_SPI_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_SPI0_BASE,
                                &s->spi[0].iomem);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->spi[0].pl022), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_SPI0_IRQ));

    /* SPI1 - PL022 Synchronous Serial Port with atomic alias wrapper */
    s->spi[1].pl022 = PL022(qdev_new(TYPE_PL022));
    s->spi[1].index = 1;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->spi[1].pl022), &error_fatal);
    memset(&spi_shadow[1], 0, sizeof(RP2350SPIRegs));
    memory_region_init_io(&s->spi[1].iomem, OBJECT(machine), &rp2350_spi_ops,
                          &s->spi[1], "rp2350.spi1", RP2350_SPI_SIZE);
    memory_region_add_subregion(get_system_memory(), RP2350_SPI1_BASE,
                                &s->spi[1].iomem);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->spi[1].pl022), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu[0]), RP2350_SPI1_IRQ));

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