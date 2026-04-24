/*
 * TMS380 Token Ring Adapter - PCI Device Model
 *
 * Emulates a Compaq 4/16 Token Ring PCI adapter (vendor 0x0E11,
 * device 0x0508) using the TMS380 SIF register interface.
 * Works with the Linux tmspci + tms380tr drivers.
 */

#ifndef HW_NET_TMS380_H
#define HW_NET_TMS380_H

#include "hw/pci/pci_device.h"
#include "qom/object.h"

#define TYPE_TMS380_PCI "tms380"
OBJECT_DECLARE_SIMPLE_TYPE(TMS380PCIState, TMS380_PCI)

/* PCI IDs — Compaq 4/16 Token Ring PCI, matched by Linux tmspci driver */
#define TMS380_PCI_VENDOR_ID    0x0E11
#define TMS380_PCI_DEVICE_ID    0x0508

/*
 * SIF Register offsets from BAR0.
 * The TMS380 has 8 registers, each 16 bits wide.
 * The Linux tms380tr driver accesses these via inw/outw.
 */
#define SIF_DAT     0x00    /* Data register */
#define SIF_INC     0x02    /* Data + auto-increment DIO address */
#define SIF_ADR     0x04    /* DIO address register (low) */
#define SIF_CMD     0x06    /* Command (write) / Status (read) */
#define SIF_ACL     0x08    /* Adapter control register */
#define SIF_ADD     0x0a    /* DIO address (alternate low) */
#define SIF_ADX     0x0c    /* DIO extended address (page/high) */
#define SIF_DMALEN  0x0e    /* DMA length */

/* SIF_ACL bits */
#define ACL_ARESET   0x0080  /* Adapter reset */
#define ACL_CPHALT   0x0040  /* Communication processor halt */
#define ACL_BOOT     0x0020  /* Bootstrap enable */
#define ACL_SINTEN   0x0008  /* System interrupt enable */
#define ACL_PEN      0x0004  /* Parity enable */

/* SIF_CMD/STS bits */
#define CMD_INTERRUPT_ADAPTER  0x8000  /* Interrupt adapter */
#define CMD_EXECUTE            0x4000  /* Execute */
#define CMD_SCB_REQUEST        0x2000  /* SCB request */
#define CMD_RCV_VALID          0x1000  /* Receive valid */
#define CMD_RCV_CONTINUE       0x0800  /* Receive continue */
#define CMD_CLEAR_SYSTEM_IRQ   0x00FF  /* Low byte: clear system IRQ mask */
#define CMD_SSB_CLEAR          0x2000  /* SSB clear (same as SCB_REQUEST) */

/* SIF_STS (read from CMD register) */
#define STS_INITIALIZE  0x0040  /* Adapter initializing */
#define STS_ERROR       0x0020  /* Error during init */
#define STS_TEST        0x0010  /* Self-test in progress */
#define STS_MASK        0x00F0  /* Status mask */
#define STS_IRQ_MASK    0x000F  /* Interrupt type */

/* Interrupt types (in STS low nibble) */
#define STS_IRQ_ADAPTER_CHECK  0x0000
#define STS_IRQ_RING_STATUS    0x0004
#define STS_IRQ_SCB_CLEAR      0x0006
#define STS_IRQ_COMMAND_STATUS 0x0008
#define STS_IRQ_RECEIVE_STATUS 0x000A
#define STS_IRQ_TRANSMIT_STATUS 0x000C

/* SRB Command codes */
#define SRB_CMD_OPEN            0x03
#define SRB_CMD_CLOSE           0x04
#define SRB_CMD_SET_GROUP_ADDR  0x06
#define SRB_CMD_SET_FUNCT_ADDR  0x07
#define SRB_CMD_READ_ERROR_LOG  0x08

/* Device states */
typedef enum {
    TMS_STATE_RESET,      /* Hardware reset in progress */
    TMS_STATE_BUD,        /* Bring-Up Diagnostics / firmware load */
    TMS_STATE_INIT,       /* Initializing (IPB written, executing) */
    TMS_STATE_READY,      /* Ready for OPEN */
    TMS_STATE_OPEN,       /* Adapter open on ring */
} TMS380DevState;

/* Adapter RAM size for DIO access */
#define TMS380_SRAM_SIZE    (128 * 1024)  /* 128KB adapter RAM (two pages) */

/* I/O BAR size: 32 bytes (tmspci driver requests TMS_PCI_IO_EXTENT = 32) */
#define TMS380_IO_SIZE      32

struct TMS380PCIState {
    PCIDevice parent_obj;

    MemoryRegion io_bar;
    TMS380DevState dev_state;

    /* SIF register values */
    uint16_t sifdat;
    uint16_t sifacl;
    uint16_t sifsts;      /* Status register (read from CMD offset) */
    uint16_t sifcmd;      /* Last command written */

    /* DIO (Direct I/O) addressing */
    uint16_t dio_addr;    /* Current DIO address (low 16 bits) */
    uint16_t dio_page;    /* DIO page (high bits via SIFADX) */

    /* MAC address */
    uint8_t mac[6];

    /* Timer for simulating reset/init delays */
    QEMUTimer *reset_timer;

    /* Adapter shared RAM — DIO reads/writes go here.
     * The MAC address is placed at address 0x0000.
     * SRB/SCB/SSB/ARB areas are at driver-configured offsets. */
    uint8_t *sram;

    /* SCB/SSB/SRB/ARB addresses (set during init via IPB) */
    uint32_t scb_addr;
    uint32_t ssb_addr;

    /* TPL/RPL chain addresses (set by TRANSMIT/RECEIVE commands) */
    uint32_t tpl_addr;    /* Head of transmit parameter list chain */
    uint32_t rpl_addr;    /* Current receive parameter list pointer */

    /* RX polling timer */
    QEMUTimer *rx_timer;
    /* Deferred interrupt timers */
    QEMUTimer *scb_clear_timer;
    QEMUTimer *cmd_status_timer;
    QEMUTimer *tx_status_timer;

    /* tr_backend FFI — loaded via dlopen */
    void *backend_lib;
    void *backend;
    char *mau_path;
    char *backend_lib_path;

    /* Raw backend function pointers (tr_raw_* from libtr_backend.so) */
    void *(*fn_raw_create)(const char *mau_path, const uint8_t *mac);
    int (*fn_raw_send)(void *backend, const uint8_t *data, uint32_t len);
    int (*fn_raw_get_recv_fd)(void *backend);
    int (*fn_raw_recv)(void *backend, uint8_t *buf, uint32_t buf_len);
    void (*fn_raw_destroy)(void *backend);
};

#endif /* HW_NET_TMS380_H */
