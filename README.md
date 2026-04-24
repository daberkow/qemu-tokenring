# QEMU Token Ring Fork

A fork of [QEMU 11.0.0](https://www.qemu.org/) that adds a **TMS380 Token Ring PCI NIC** device model. Emulates a Compaq 4/16 Token Ring PCI adapter (vendor `0x0E11`, device `0x0508`) that works with the Linux `tms380tr` + `tmspci` kernel drivers.

Two QEMU guests on the same virtual ring can ping, TCP, and exchange arbitrary traffic over Token Ring — just like it's 1995.

## Branch Layout

| Branch | Purpose |
|--------|---------|
| `tms380-dev` | **Default.** Token Ring device model + all development work |
| `master` | Tracks upstream QEMU — synced periodically, no Token Ring code |

## Requirements

- [vmau](https://github.com/daberkow/tokenring) — the virtual MAU (Multistation Access Unit) that provides ring topology
- `libtr_backend.so` — built from the same tokenring repo (`cargo build --release`, output in `target/release/`)
- A guest disk image with Token Ring drivers (Linux 2.6.x kernels have `tms380tr`; Debian 6 works well)
- KVM-capable host (recommended)

## Building

```bash
# Standard QEMU build — TMS380 device is included automatically
mkdir build && cd build
../configure --target-list=x86_64-softmmu --enable-kvm --enable-slirp
make -j$(nproc)
```

## Usage

```bash
# Terminal 1: start the virtual MAU
vmau --socket-path /tmp/vmau.sock

# Terminal 2: VM1 (default MAC 00:00:F8:00:00:01)
./build/qemu-system-x86_64 -m 2G -hda vm1.qcow2 -enable-kvm \
  -device "tms380,mau_path=/tmp/vmau.sock,backend_lib=/path/to/libtr_backend.so" \
  -device e1000,netdev=net0 -netdev user,id=net0,hostfwd=tcp::2222-:22 \
  -vnc :1

# Terminal 3: VM2 (different MAC)
./build/qemu-system-x86_64 -m 2G -hda vm2.qcow2 -enable-kvm \
  -device "tms380,mau_path=/tmp/vmau.sock,backend_lib=/path/to/libtr_backend.so,mac0=0x00,mac1=0x00,mac2=0xF8,mac3=0x00,mac4=0x00,mac5=0x02" \
  -device e1000,netdev=net0 -netdev user,id=net0,hostfwd=tcp::2223-:22 \
  -vnc :2
```

Inside the guests (once tr0 is up with IPs):
```
ping 10.0.0.2    # from VM1
ping 10.0.0.1    # from VM2
```

## Device Properties

| Property | Description | Default |
|----------|-------------|---------|
| `mau_path` | Path to vmau Unix domain socket | (required) |
| `backend_lib` | Path to `libtr_backend.so` | (required) |
| `mac0`..`mac5` | MAC address bytes | `00:00:F8:00:00:01` |

## Guest OS Requirements

The guest kernel needs the `tms380tr` and `tmspci` modules. These were removed from Linux around kernel 4.x. Known working distros:

- **Debian 6** (kernel 2.6.32) — tested, `tr0` appears on boot
- **Debian 7** (kernel 3.2)
- **Ubuntu 10.04** (kernel 2.6.32)
- **Ubuntu 12.04** (kernel 3.2)

Any Linux distribution with a 2.6.x or 3.x kernel should work. The adapter requires firmware (`tms380tr.bin`) which Debian includes in the `firmware-misc-nonfree` package or can be extracted from any TMS380 adapter's flash.

## Architecture

The device model implements the TMS380 SIF (System Interface) register set:

| Register | Offset | Function |
|----------|--------|----------|
| SIFDAT | 0x00 | DIO data read/write |
| SIFINC | 0x02 | DIO data + auto-increment |
| SIFADR | 0x04 | DIO address (low) |
| SIFCMD/STS | 0x06 | Command write / status read |
| SIFACL | 0x08 | Adapter control |
| SIFADD | 0x0A | DIO address (alternate) |
| SIFADX | 0x0C | DIO extended address (page) |
| SIFDMALEN | 0x0E | DMA length |

The guest driver communicates via an SRB/SCB/SSB command protocol in shared DMA memory. The device model intercepts SCB commands (OPEN, TRANSMIT, RECEIVE, CLOSE, etc.) and translates them into raw 802.5 frame I/O through `libtr_backend.so`, which connects to vmau over a Unix domain socket.

Key files:
- `hw/net/tms380.c` — device model implementation (~900 lines)
- `hw/net/tms380.h` — register defines and device state
- `hw/net/Kconfig` — build integration
- `hw/net/meson.build` — build integration

## Status

Working:
- PCI device probe and initialization
- Firmware load (BUD sequence)
- OPEN / CLOSE commands
- TX path (TPL chain processing, frame DMA, completion interrupts)
- RX path (frame DMA to RPL buffers, receive interrupts)
- ARP resolution between guests
- ICMP ping between guests
- TCP connections (tested with netcat)

## Related

- [tokenring](https://github.com/daberkow/tokenring) — vmau, tr-station MAC library, and tr-backend FFI bridge
- [Design document](https://github.com/daberkow/tokenring/blob/main/docs/vtr-design.md) — full architecture and protocol specification
