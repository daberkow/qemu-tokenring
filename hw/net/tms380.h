/*
 * TMS380 Token Ring Adapter - PCI Device Model
 *
 * Emulates a Compaq 4/16 Token Ring PCI adapter (vendor 0x0E11,
 * device 0x0046) using the TMS380 SIF register interface.
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
#define TMS380_PCI_DEVICE_ID    0x0046

/* SIF Register offsets from BAR0 */
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
#define SIFCMD_EXECUTE  (1 << 1)    /* Execute SRB command */
#define SIFCMD_ADAP_INT (1 << 2)    /* Adapter interrupt pending */

/* SRB Command codes */
#define SRB_CMD_OPEN            0x03
#define SRB_CMD_CLOSE           0x04
#define SRB_CMD_SET_GROUP_ADDR  0x06
#define SRB_CMD_SET_FUNCT_ADDR  0x07
#define SRB_CMD_READ_ERROR_LOG  0x08

/* SRB offsets */
#define SRB_CMD_OFFSET      0
#define SRB_RETCODE_OFFSET  2

/* SRB return codes */
#define SRB_SUCCESS         0x00

/* Device states */
typedef enum {
    TMS_STATE_RESET,
    TMS_STATE_READY,
    TMS_STATE_OPEN,
} TMS380DevState;

/* Adapter shared memory size */
#define TMS380_SRAM_SIZE    256

/* I/O BAR size: 8 bytes (4 x 16-bit registers) */
#define TMS380_IO_SIZE      8

struct TMS380PCIState {
    PCIDevice parent_obj;

    MemoryRegion io_bar;
    TMS380DevState dev_state;

    /* SIF register values */
    uint16_t sifdat;
    uint16_t sifcmd;
    uint16_t sifadr;
    uint16_t sifacl;

    /* MAC address */
    uint8_t mac[6];

    /* Initialization data read sequence */
    int init_read_index;

    /* Timer for simulating reset delay */
    QEMUTimer *reset_timer;

    /* Adapter shared memory (SRB buffer) */
    uint8_t sram[TMS380_SRAM_SIZE];

    /* tr_backend FFI — loaded via dlopen */
    void *backend_lib;      /* dlopen handle */
    void *backend;          /* TrBackend* from tr_backend_create */
    char *mau_path;         /* vmau socket path */
    char *backend_lib_path; /* path to libtr_backend.so */

    /* Function pointers resolved from backend_lib */
    void *(*fn_create)(const char *mau_path, const uint8_t *mac);
    int (*fn_insert)(void *backend);
    int (*fn_send)(void *backend, const uint8_t *dst, const uint8_t *data, uint32_t len);
    int (*fn_get_recv_fd)(void *backend);
    int (*fn_recv)(void *backend, uint8_t *src_mac, uint8_t *buf, uint32_t buf_len);
    void (*fn_destroy)(void *backend);
};

#endif /* HW_NET_TMS380_H */
