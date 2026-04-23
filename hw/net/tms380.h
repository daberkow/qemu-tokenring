/*
 * TMS380 Token Ring Adapter - Register Definitions
 *
 * Emulates the TI TMS380C26 System Interface (SIF) registers
 * for use with the Linux tms380tr driver.
 */

#ifndef HW_NET_TMS380_H
#define HW_NET_TMS380_H

#include "hw/isa/isa.h"
#include "qom/object.h"

#define TYPE_TMS380 "tms380"
OBJECT_DECLARE_SIMPLE_TYPE(TMS380State, TMS380)

/* SIF Register offsets from I/O base */
#define SIF_DAT     0x00    /* Data register (16-bit) */
#define SIF_CMD     0x02    /* Command/status register */
#define SIF_ADR     0x04    /* Address register */
#define SIF_ACL     0x06    /* Adapter control/status */

/* SIF_ACL bits */
#define SIFACL_ARESET   (1 << 8)    /* Adapter reset */
#define SIFACL_INIT     (1 << 9)    /* Initializing (1=busy, 0=ready) */
#define SIFACL_SINTEN   (1 << 1)    /* System interrupt enable */

/* SIF_CMD bits */
#define SIFCMD_INTRESET (1 << 0)    /* Reset interrupt */
#define SIFCMD_ADAP_INT (1 << 2)    /* Adapter interrupt pending */

/* Device states */
typedef enum {
    TMS_STATE_RESET,
    TMS_STATE_READY,
} TMS380DevState;

/* I/O port region size: 8 bytes (4 x 16-bit registers) */
#define TMS380_IO_SIZE  8

struct TMS380State {
    ISADevice parent_obj;

    MemoryRegion io;
    qemu_irq irq;
    uint32_t iobase;
    uint32_t irq_num;

    /* SIF register values */
    uint16_t sifdat;
    uint16_t sifcmd;
    uint16_t sifadr;
    uint16_t sifacl;

    /* Device state */
    TMS380DevState dev_state;
    uint8_t mac[6];

    /* Initialization data sequence — the driver reads these from SIFDAT
     * after reset to discover adapter capabilities */
    int init_read_index;

    /* Timer for simulating reset delay */
    QEMUTimer *reset_timer;
};

#endif /* HW_NET_TMS380_H */
