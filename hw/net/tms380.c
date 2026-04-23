/*
 * TMS380 Token Ring Adapter - QEMU PCI Device Model
 *
 * Emulates a Compaq 4/16 Token Ring PCI adapter using the TMS380
 * SIF register interface. The Linux tmspci + tms380tr drivers
 * communicate through these registers via DIO (Direct I/O).
 *
 * Copyright (c) 2026 Dan Berkowitz
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "exec/cpu-common.h"
#include "tms380.h"
#include <dlfcn.h>

/* --- DIO (Direct I/O) helpers --- */

/* Read a 16-bit word from adapter SRAM at the current DIO address */
static uint16_t dio_read16(TMS380PCIState *s)
{
    uint32_t addr = ((uint32_t)s->dio_page << 16) | s->dio_addr;
    if (addr + 1 < TMS380_SRAM_SIZE) {
        return (s->sram[addr] << 8) | s->sram[addr + 1];
    }
    return 0xFFFF;
}

/* Read 16-bit from SRAM and auto-increment the DIO address by 2 */
static uint16_t dio_read16_inc(TMS380PCIState *s)
{
    uint16_t val = dio_read16(s);
    s->dio_addr += 2;
    return val;
}

/* Write a 16-bit word to adapter SRAM at the current DIO address */
static void dio_write16(TMS380PCIState *s, uint16_t val)
{
    uint32_t addr = ((uint32_t)s->dio_page << 16) | s->dio_addr;
    if (addr + 1 < TMS380_SRAM_SIZE) {
        s->sram[addr] = (val >> 8) & 0xFF;
        s->sram[addr + 1] = val & 0xFF;
    }
}

/* Write 16-bit to SRAM and auto-increment */
static void dio_write16_inc(TMS380PCIState *s, uint16_t val)
{
    dio_write16(s, val);
    s->dio_addr += 2;
}

/* --- Backend loading --- */

static bool tms380_load_backend(TMS380PCIState *s)
{
    if (!s->backend_lib_path || !s->backend_lib_path[0]) {
        return false;
    }

    s->backend_lib = dlopen(s->backend_lib_path, RTLD_NOW);
    if (!s->backend_lib) {
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: dlopen(%s): %s\n",
                      s->backend_lib_path, dlerror());
        return false;
    }

    s->fn_create = dlsym(s->backend_lib, "tr_backend_create");
    s->fn_insert = dlsym(s->backend_lib, "tr_backend_insert");
    s->fn_send = dlsym(s->backend_lib, "tr_backend_send");
    s->fn_get_recv_fd = dlsym(s->backend_lib, "tr_backend_get_recv_fd");
    s->fn_recv = dlsym(s->backend_lib, "tr_backend_recv");
    s->fn_destroy = dlsym(s->backend_lib, "tr_backend_destroy");

    if (!s->fn_create || !s->fn_insert || !s->fn_send ||
        !s->fn_get_recv_fd || !s->fn_recv || !s->fn_destroy) {
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: missing symbols in %s\n",
                      s->backend_lib_path);
        dlclose(s->backend_lib);
        s->backend_lib = NULL;
        return false;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: backend loaded from %s\n",
                  s->backend_lib_path);
    return true;
}

/* --- Reset and initialization --- */

static void tms380_bud_done(void *opaque)
{
    TMS380PCIState *s = opaque;

    /* BUD complete — set STS_INITIALIZE to tell driver we're ready for IPB.
     * The driver polls SIFSTS after sending the soft reset command. */
    s->dev_state = TMS_STATE_BUD;
    s->sifsts = STS_INITIALIZE;
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: BUD complete, awaiting init\n");
}

static void tms380_do_reset(TMS380PCIState *s)
{
    s->dev_state = TMS_STATE_RESET;
    s->sifsts = 0;  /* No status bits during reset */
    s->sifcmd = 0;
    s->dio_addr = 0;
    s->dio_page = 0;

    /* Place MAC address at SRAM address 0x0000 (big-endian, one byte per word).
     * The tmspci driver reads it as: SIFADX=0, SIFADR=0, then 6x SIFINC >> 8 */
    for (int i = 0; i < 6; i++) {
        s->sram[i * 2] = s->mac[i];
        s->sram[i * 2 + 1] = 0x00;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: hardware reset\n");
    /* Don't start BUD timer here — wait for the soft reset command */
}

/* Called when driver sends EXEC_SOFT_RESET (0xFF00) to SIFCMD.
 * This happens after firmware download and CPHALT clear.
 * Transition directly to BUD complete — the driver polls SIFSTS
 * in a tight loop and with KVM it's too fast for a timer. */
static void tms380_soft_reset(TMS380PCIState *s)
{
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: soft reset → BUD complete\n");
    s->dev_state = TMS_STATE_BUD;
    s->sifsts = STS_INITIALIZE;
}

/* SCB and SSB test patterns that the driver checks after init.
 * The adapter must DMA-write these to the host's SCB/SSB buffers
 * to prove DMA is working. */
static const uint8_t SCB_Test[6] = {0x00, 0x00, 0xC1, 0xE2, 0xD4, 0x8B};
static const uint8_t SSB_Test[8] = {0xFF, 0xFF, 0xD1, 0xD7, 0xC5, 0xD9, 0xC3, 0xD4};

/* Handle the initialization command (after driver writes IPB and sends CMD_EXECUTE) */
static void tms380_handle_init(TMS380PCIState *s)
{
    /* The driver wrote an 11-word IPB to our SRAM at page 1, offset 0x0A00.
     * Words 7-8 = SCB DMA address (word-swapped: high word first)
     * Words 9-10 = SSB DMA address (word-swapped: high word first)
     *
     * After we clear the init status bits, the driver checks that we
     * DMA-wrote the SCB_Test and SSB_Test patterns to those host addresses.
     * We use pci_dma_write to write from our device to guest physical memory. */

    /* IPB is at page 1 (0x10000) + offset 0x0A00 in our SRAM */
    uint32_t ipb_base = 0x10000 + 0x0A00;

    /* Read SCB address from IPB words 7-8 (byte offsets 14-17).
     * The driver stores SWAPW(dma_addr): on LE x86, the first word written
     * (word7) is the LOW 16 bits of the swapped value, which is the HIGH
     * 16 bits of the original address. So: addr = (word7 << 16) | word8 */
    uint16_t scb_w7 = (s->sram[ipb_base + 14] << 8) | s->sram[ipb_base + 15];
    uint16_t scb_w8 = (s->sram[ipb_base + 16] << 8) | s->sram[ipb_base + 17];
    uint32_t scb_addr = ((uint32_t)scb_w7 << 16) | scb_w8;

    /* Read SSB address from IPB words 9-10 (byte offsets 18-21) */
    uint16_t ssb_w9 = (s->sram[ipb_base + 18] << 8) | s->sram[ipb_base + 19];
    uint16_t ssb_w10 = (s->sram[ipb_base + 20] << 8) | s->sram[ipb_base + 21];
    uint32_t ssb_addr = ((uint32_t)ssb_w9 << 16) | ssb_w10;

    /* Dump full 22-byte IPB for debugging */
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: init: IPB dump (22 bytes):\n");
    for (int i = 0; i < 22; i += 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "  word%d: %02x %02x\n",
                      i/2, s->sram[ipb_base+i], s->sram[ipb_base+i+1]);
    }
    /* Log all possible address interpretations to find the right one */
    uint8_t *b = &s->sram[ipb_base + 14];
    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: SCB raw bytes: %02x %02x %02x %02x\n",
                  b[0], b[1], b[2], b[3]);
    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: SCB interpretations:\n"
                  "  BE32: 0x%02x%02x%02x%02x\n"
                  "  LE32: 0x%02x%02x%02x%02x\n"
                  "  (w7<<16)|w8: 0x%08x\n"
                  "  (w8<<16)|w7: 0x%08x\n",
                  b[0], b[1], b[2], b[3],
                  b[3], b[2], b[1], b[0],
                  scb_addr,
                  ((uint32_t)scb_w8 << 16) | scb_w7);

    /* Save addresses for later SRB command handling */
    s->scb_addr = scb_addr;
    s->ssb_addr = ssb_addr;

    /* Force-enable bus mastering for DMA. The tmspci driver doesn't call
     * pci_set_master() but we need DMA for SCB/SSB test patterns.
     * Use pci_set_word to properly update QEMU's internal state. */
    {
        uint16_t cmd = pci_get_word(s->parent_obj.config + PCI_COMMAND);
        pci_set_word(s->parent_obj.config + PCI_COMMAND, cmd | PCI_COMMAND_MASTER);
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: PCI_COMMAND: 0x%04x → 0x%04x\n",
                      cmd, pci_get_word(s->parent_obj.config + PCI_COMMAND));
    }

    /* Write test patterns directly to guest physical memory.
     * The driver checks these to verify DMA is working. */
    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: writing SCB test pattern to phys 0x%08x\n", scb_addr);
    cpu_physical_memory_write(scb_addr, SCB_Test, sizeof(SCB_Test));

    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: writing SSB test pattern to phys 0x%08x\n", ssb_addr);
    cpu_physical_memory_write(ssb_addr, SSB_Test, sizeof(SSB_Test));

    /* Verify SCB write */
    {
        uint8_t verify[6] = {0};
        cpu_physical_memory_read(scb_addr, verify, sizeof(verify));
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tms380: SCB verify: %02x %02x %02x %02x %02x %02x\n",
                      verify[0], verify[1], verify[2], verify[3],
                      verify[4], verify[5]);
    }

    /* After init, the adapter OVERWRITES the IPB area (page 1, offset 0x0A00)
     * with the INTPTRS structure (8 x 16-bit pointers). The driver reads this
     * via tms380tr_read_ptr() to find adapter internals.
     *
     * INTPTRS layout (tms380tr.h):
     *   Word 0: BurnedInAddrPtr  — pointer to BIA in adapter RAM
     *   Word 1: SoftwareLevelPtr — pointer to software level
     *   Word 2: AdapterAddrPtr   — pointer to adapter address
     *   Word 3: AdapterParmsPtr  — pointer to adapter parameters
     *   Word 4: MACBufferPtr     — pointer to MAC buffer
     *   Word 5: LLCCountersPtr   — pointer to LLC counters
     *   Word 6: SpeedFlagPtr     — pointer to speed flag
     *   Word 7: AdapterRAMPtr    — pointer to RAM size value (in KB)
     *
     * Each pointer is a big-endian 16-bit address within page 1 of SRAM.
     * The driver dereferences these to read the actual data.
     */

    /* Place data values at known SRAM locations (page 1) */
    /* RAM size at page 1, offset 0x0B00: 64KB = 0x0040 in KB (big-endian) */
    uint32_t ram_size_addr = 0x10000 + 0x0B00;
    s->sram[ram_size_addr] = 0x00;
    s->sram[ram_size_addr + 1] = 0x40;  /* 64 KB */

    /* Speed flag at page 1, offset 0x0B10: 16 Mbps = 0x0010 */
    uint32_t speed_addr = 0x10000 + 0x0B10;
    s->sram[speed_addr] = 0x00;
    s->sram[speed_addr + 1] = 0x10;

    /* Now write the INTPTRS structure at page 1, offset 0x0A00.
     * Pointers are big-endian 16-bit SRAM offsets within page 1. */
    uint32_t intptrs = 0x10000 + 0x0A00;
    /* Word 0: BurnedInAddrPtr → offset 0x0000 (MAC is at page 0, addr 0) */
    s->sram[intptrs + 0] = 0x00; s->sram[intptrs + 1] = 0x00;
    /* Word 1: SoftwareLevelPtr → 0x0B20 */
    s->sram[intptrs + 2] = 0x0B; s->sram[intptrs + 3] = 0x20;
    /* Word 2: AdapterAddrPtr → 0x0000 */
    s->sram[intptrs + 4] = 0x00; s->sram[intptrs + 5] = 0x00;
    /* Word 3: AdapterParmsPtr → 0x0B30 */
    s->sram[intptrs + 6] = 0x0B; s->sram[intptrs + 7] = 0x30;
    /* Word 4: MACBufferPtr → 0x0B40 */
    s->sram[intptrs + 8] = 0x0B; s->sram[intptrs + 9] = 0x40;
    /* Word 5: LLCCountersPtr → 0x0B50 */
    s->sram[intptrs + 10] = 0x0B; s->sram[intptrs + 11] = 0x50;
    /* Word 6: SpeedFlagPtr → 0x0B10 */
    s->sram[intptrs + 12] = 0x0B; s->sram[intptrs + 13] = 0x10;
    /* Word 7: AdapterRAMPtr → 0x0B00 (where we stored the RAM size) */
    s->sram[intptrs + 14] = 0x0B; s->sram[intptrs + 15] = 0x00;

    /* Clear all status bits to signal init complete */
    s->sifsts = 0;
    s->dev_state = TMS_STATE_READY;
    qemu_log_mask(LOG_GUEST_ERROR, "tms380: initialization complete, ready\n");
}

/* --- SRB command handling --- */

static void tms380_raise_irq(TMS380PCIState *s, uint16_t irq_type)
{
    /* Set the IRQ type in the low nibble and STS_SYSTEM_IRQ (bit 7)
     * so the driver's interrupt handler enters its processing loop. */
    s->sifsts = (s->sifsts & ~(STS_IRQ_MASK | 0x0080)) |
                (irq_type & STS_IRQ_MASK) | 0x0080;
    pci_irq_assert(&s->parent_obj);
}

static void tms380_handle_srb(TMS380PCIState *s, uint8_t *srb, uint32_t srb_phys);

/* Read SCB from host memory and dispatch the command */
static void tms380_handle_scb(TMS380PCIState *s)
{
    uint8_t scb[6];
    cpu_physical_memory_read(s->scb_addr, scb, 6);

    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: SCB at phys 0x%08x: %02x %02x %02x %02x %02x %02x\n",
                  s->scb_addr, scb[0], scb[1], scb[2], scb[3], scb[4], scb[5]);

    /* The SCB struct is packed big-endian on the TMS380:
     *   u16 CMD
     *   u16 Parm[0]  (high word of 32-bit address)
     *   u16 Parm[1]  (low word of 32-bit address)
     * But the driver writes to host memory in native (LE) byte order,
     * using cpu_to_be16() for each field. So the bytes in memory are BE. */
    uint16_t cmd = (scb[0] << 8) | scb[1];
    /* The Parm is SWAPW'd: Parm[0]=low16, Parm[1]=high16 of the address.
     * Same SWAPW convention as IPB addresses. */
    uint16_t parm_w0 = (scb[2] << 8) | scb[3];
    uint16_t parm_w1 = (scb[4] << 8) | scb[5];
    uint32_t parm_addr = ((uint32_t)parm_w0 << 16) | parm_w1;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: SCB cmd=0x%04x parm=0x%08x (w0=0x%04x w1=0x%04x)\n",
                  cmd, parm_addr, parm_w0, parm_w1);

    /* Read the SRB/command parameter block from host memory */
    uint8_t srb[32];
    memset(srb, 0, sizeof(srb));
    if (parm_addr) {
        cpu_physical_memory_read(parm_addr, srb, sizeof(srb));
        qemu_log_mask(LOG_GUEST_ERROR,
                      "tms380: SRB raw: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                      srb[0], srb[1], srb[2], srb[3],
                      srb[4], srb[5], srb[6], srb[7]);
    }

    /* The SCB CMD field is the actual command code (e.g., 0x0003 = OPEN).
     * The Parm points to the command's parameter block (e.g., OPB for OPEN).
     * Override srb[0] with the SCB command code for our SRB handler. */
    srb[0] = cmd & 0xFF;
    tms380_handle_srb(s, srb, parm_addr);
}

static void tms380_handle_srb(TMS380PCIState *s, uint8_t *srb, uint32_t srb_phys)
{
    uint8_t cmd = srb[0];

    qemu_log_mask(LOG_GUEST_ERROR, "tms380: SRB cmd=0x%02x at phys 0x%08x\n",
                  cmd, srb_phys);

    switch (cmd) {
    case SRB_CMD_OPEN:
        if (s->fn_create && s->mau_path && s->mau_path[0]) {
            s->backend = s->fn_create(s->mau_path, s->mac);
            if (s->backend && s->fn_insert(s->backend) == 0) {
                s->dev_state = TMS_STATE_OPEN;
                srb[2] = 0x00;
                qemu_log_mask(LOG_GUEST_ERROR, "tms380: OPEN success\n");
            } else {
                if (s->backend) {
                    s->fn_destroy(s->backend);
                    s->backend = NULL;
                }
                srb[2] = 0x07;
                qemu_log_mask(LOG_GUEST_ERROR, "tms380: OPEN failed\n");
            }
        } else {
            s->dev_state = TMS_STATE_OPEN;
            srb[2] = 0x00;
            qemu_log_mask(LOG_GUEST_ERROR, "tms380: OPEN (stub)\n");
        }
        break;

    case SRB_CMD_CLOSE:
        if (s->backend && s->fn_destroy) {
            s->fn_destroy(s->backend);
            s->backend = NULL;
        }
        s->dev_state = TMS_STATE_READY;
        srb[2] = 0x00;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: CLOSE\n");
        break;

    case SRB_CMD_SET_FUNCT_ADDR:
    case SRB_CMD_SET_GROUP_ADDR:
    case SRB_CMD_READ_ERROR_LOG:
        srb[2] = 0x00;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SRB 0x%02x (stub OK)\n", cmd);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: unknown SRB 0x%02x\n", cmd);
        srb[2] = 0x04;
        break;
    }

    /* Write SRB result back to host memory */
    if (srb_phys) {
        cpu_physical_memory_write(srb_phys, srb, 32);
    }

    /* Write SSB completion to host memory.
     * SSB is big-endian: {u16 STS, u16 Parm[0], u16 Parm[1], u16 Parm[2]}
     * STS = command code (e.g., 0x0300 for OPEN)
     * Parm[0] = status (0x0080 = GOOD_COMPLETION for OPEN) */
    {
        uint8_t ssb[8] = {0};
        ssb[0] = 0x00;  /* STS high byte */
        ssb[1] = cmd;   /* STS low byte = command code */
        /* Wait — the driver reads ssb_cmd = tp->ssb.STS which is u16.
         * OPEN = 0x0300. In BE memory: byte0=0x03, byte1=0x00. */
        /* SSB is in host memory on LE x86. The driver reads fields as native u16.
         * STS = OPEN (0x0300): LE bytes = {0x00, 0x03}
         * Parm[0] = GOOD_COMPLETION (0x0080): LE bytes = {0x80, 0x00} */
        ssb[0] = 0x00; ssb[1] = cmd;   /* STS = 0x0300 for OPEN (LE) */
        ssb[2] = 0x80; ssb[3] = 0x00;  /* Parm[0] = GOOD_COMPLETION (LE) */
        cpu_physical_memory_write(s->ssb_addr, ssb, 8);
    }

    /* Raise command completion interrupt */
    tms380_raise_irq(s, STS_IRQ_COMMAND_STATUS);
}

/* --- SIF register I/O --- */

static uint64_t tms380_io_read(void *opaque, hwaddr addr, unsigned size)
{
    TMS380PCIState *s = opaque;
    uint16_t val = 0;

    switch (addr) {
    case SIF_DAT:   /* 0x00 — read SRAM at current DIO address */
        val = dio_read16(s);
        break;

    case SIF_INC:   /* 0x02 — read SRAM + auto-increment */
        val = dio_read16_inc(s);
        if (s->dev_state >= TMS_STATE_READY) {
            qemu_log_mask(LOG_GUEST_ERROR, "tms380: SIFINC read=0x%04x addr=0x%04x page=0x%04x\n",
                          val, (uint16_t)(s->dio_addr - 2), s->dio_page);
        }
        break;

    case SIF_ADR:   /* 0x04 — current DIO address */
        val = s->dio_addr;
        break;

    case SIF_CMD:   /* 0x06 — read returns status register */
        val = s->sifsts;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SIFSTS read 0x%04x state=%d\n",
                      val, s->dev_state);
        break;

    case SIF_ACL:   /* 0x08 — adapter control */
        val = s->sifacl;
        break;

    case SIF_ADD:   /* 0x0a — DIO address (alternate) */
        val = s->dio_addr;
        break;

    case SIF_ADX:   /* 0x0c — DIO extended/page address */
        val = s->dio_page;
        break;

    case SIF_DMALEN: /* 0x0e — DMA length (not used) */
        val = 0;
        break;

    default:
        break;
    }

    return val;
}

static void tms380_io_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    TMS380PCIState *s = opaque;
    uint16_t v = (uint16_t)val;

    switch (addr) {
    case SIF_DAT:   /* 0x00 — write SRAM at current DIO address */
        dio_write16(s, v);
        break;

    case SIF_INC:   /* 0x02 — write SRAM + auto-increment */
        dio_write16_inc(s, v);
        break;

    case SIF_ADR:   /* 0x04 — set DIO address */
        s->dio_addr = v;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SIFADR=0x%04x (page=0x%04x)\n",
                      v, s->dio_page);
        break;

    case SIF_CMD:   /* 0x06 — command register */
        s->sifcmd = v;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SIFCMD write 0x%04x state=%d\n",
                      v, s->dev_state);

        /* EXEC_SOFT_RESET: the driver sends 0xFF80 (or similar high-byte-set
         * command) after firmware download to start BUD. Match any write with
         * the high byte = 0xFF while in RESET state. */
        if ((v & 0xFF00) == 0xFF00 && s->dev_state == TMS_STATE_RESET) {
            tms380_soft_reset(s);
            break;
        }

        /* Handle command bits */
        if (v & (CMD_EXECUTE | CMD_INTERRUPT_ADAPTER)) {
            if (s->dev_state == TMS_STATE_BUD &&
                (s->sifsts & STS_INITIALIZE)) {
                /* BUD done, driver wrote IPB and sent init command — complete init */
                tms380_handle_init(s);
            } else if (s->dev_state == TMS_STATE_READY ||
                       s->dev_state == TMS_STATE_OPEN) {
                /* The driver sends commands via SCB in host memory.
                 * CMD_INTERRUPT_ADAPTER signals a new command is ready.
                 * Read the SCB from host memory to find the SRB. */
                /* Only dispatch new commands, not SSB_CLEAR acks */
                if ((v & CMD_INTERRUPT_ADAPTER) && !(v & CMD_SSB_CLEAR)) {
                    tms380_handle_scb(s);
                }
            }
        }

        /* Clear system interrupt when the driver writes the low byte (ACK).
         * The driver writes CMD_SSB_CLEAR | CMD_INTERRUPT_ADAPTER (0xA0XX)
         * to acknowledge a command completion interrupt.
         * CMD_SSB_CLEAR = 0x2000, CMD_INTERRUPT_ADAPTER = 0x8000.
         * We clear the IRQ whenever CMD_SSB_CLEAR is set. */
        if (v & CMD_SSB_CLEAR) {
            pci_irq_deassert(&s->parent_obj);
            s->sifsts &= ~(STS_IRQ_MASK | 0x0080);
        }
        /* Also clear on any low-byte write without CMD_INTERRUPT_ADAPTER */
        else if ((v & 0x00FF) && !(v & CMD_INTERRUPT_ADAPTER)) {
            pci_irq_deassert(&s->parent_obj);
            s->sifsts &= ~(STS_IRQ_MASK | 0x0080);
        }
        break;

    case SIF_ACL:   /* 0x08 — adapter control */
        s->sifacl = v;
        if (v & ACL_ARESET) {
            tms380_do_reset(s);
        }
        break;

    case SIF_ADD:   /* 0x0a — DIO address (alternate set) */
        s->dio_addr = v;
        break;

    case SIF_ADX:   /* 0x0c — DIO page/extended address */
        s->dio_page = v;
        qemu_log_mask(LOG_GUEST_ERROR, "tms380: SIFADX=0x%04x\n", v);
        break;

    case SIF_DMALEN: /* 0x0e — DMA length (ignored) */
        break;

    default:
        break;
    }
}

static const MemoryRegionOps tms380_io_ops = {
    .read = tms380_io_read,
    .write = tms380_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 2,
    },
};

/* --- PCI device lifecycle --- */

static void tms380_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    TMS380PCIState *s = TMS380_PCI(pci_dev);

    /* Set PCI interrupt pin A */
    pci_dev->config[PCI_INTERRUPT_PIN] = 1;

    /* The tmspci driver doesn't call pci_set_master(), but we need bus
     * mastering for DMA. Set it as a default so the guest can use it.
     * Use wmask to allow the guest to control it if it wants to. */
    pci_dev->config[PCI_COMMAND] = PCI_COMMAND_MASTER;
    pci_dev->wmask[PCI_COMMAND] |= PCI_COMMAND_MASTER;

    /* Allocate adapter SRAM */
    s->sram = g_malloc0(TMS380_SRAM_SIZE);

    /* Register I/O BAR */
    memory_region_init_io(&s->io_bar, OBJECT(pci_dev), &tms380_io_ops, s,
                          "tms380-sif", TMS380_IO_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io_bar);

    /* Create reset timer */
    s->reset_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tms380_bud_done, s);

    /* Load backend if configured */
    tms380_load_backend(s);

    /* Start in reset */
    tms380_do_reset(s);

    qemu_log_mask(LOG_GUEST_ERROR,
                  "tms380: PCI adapter MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                  s->mac[0], s->mac[1], s->mac[2],
                  s->mac[3], s->mac[4], s->mac[5]);
}

static void tms380_pci_exit(PCIDevice *pci_dev)
{
    TMS380PCIState *s = TMS380_PCI(pci_dev);

    timer_free(s->reset_timer);

    if (s->backend && s->fn_destroy) {
        s->fn_destroy(s->backend);
        s->backend = NULL;
    }
    if (s->backend_lib) {
        dlclose(s->backend_lib);
        s->backend_lib = NULL;
    }

    g_free(s->sram);
}

static const Property tms380_pci_properties[] = {
    DEFINE_PROP_STRING("mau_path", TMS380PCIState, mau_path),
    DEFINE_PROP_STRING("backend_lib", TMS380PCIState, backend_lib_path),
    DEFINE_PROP_UINT8("mac0", TMS380PCIState, mac[0], 0x00),
    DEFINE_PROP_UINT8("mac1", TMS380PCIState, mac[1], 0x00),
    DEFINE_PROP_UINT8("mac2", TMS380PCIState, mac[2], 0xF8),
    DEFINE_PROP_UINT8("mac3", TMS380PCIState, mac[3], 0x00),
    DEFINE_PROP_UINT8("mac4", TMS380PCIState, mac[4], 0x00),
    DEFINE_PROP_UINT8("mac5", TMS380PCIState, mac[5], 0x01),
};

static void tms380_pci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = tms380_pci_realize;
    k->exit = tms380_pci_exit;
    k->vendor_id = TMS380_PCI_VENDOR_ID;
    k->device_id = TMS380_PCI_DEVICE_ID;
    k->class_id = PCI_CLASS_NETWORK_OTHER;
    k->revision = 0x01;
    device_class_set_props(dc, tms380_pci_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "TMS380 Token Ring 16/4 PCI Adapter (Compaq)";
}

static const TypeInfo tms380_pci_type_info = {
    .name          = TYPE_TMS380_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(TMS380PCIState),
    .class_init    = tms380_pci_class_init,
    .interfaces    = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void tms380_pci_register_types(void)
{
    type_register_static(&tms380_pci_type_info);
}

type_init(tms380_pci_register_types)
