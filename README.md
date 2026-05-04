![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Tiny Tapeout Verilog Project Template

- [Read the documentation for project](docs/info.md)

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Set up your Verilog project

1. Add your Verilog files to the `src` folder.
2. Edit the [info.yaml](info.yaml) and update information about your project, paying special attention to the `source_files` and `top_module` properties. If you are upgrading an existing Tiny Tapeout project, check out our [online info.yaml migration tool](https://tinytapeout.github.io/tt-yaml-upgrade-tool/).
3. Edit [docs/info.md](docs/info.md) and add a description of your project.
4. Adapt the testbench to your design. See [test/README.md](test/README.md) for more information.

The GitHub action will automatically build the ASIC files using [LibreLane](https://www.zerotoasiccourse.com/terminology/librelane/).

## Enable GitHub actions to build the results page

- [Enabling GitHub Pages](https://tinytapeout.com/faq/#my-github-action-is-failing-on-the-pages-part)

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)

## What next?

- [Submit your design to the next shuttle](https://app.tinytapeout.com/).
- Edit [this README](README.md) and explain your design, how it works, and how to test it.
- Share your project on your social network of choice:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [@TinyTapeout](https://www.linkedin.com/company/100708654/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - X (formerly Twitter) [#tinytapeout](https://twitter.com/hashtag/tinytapeout) [@tinytapeout](https://twitter.com/tinytapeout)
  - Bluesky [@tinytapeout.com](https://bsky.app/profile/tinytapeout.com)


  # CRC_FIFO — CRC-32 Engine with 8-Byte FIFO and VGA Display

A Tiny Tapeout submission implementing a hardware CRC-32 integrity verification engine
with real-time VGA visualization, designed for edge AI systems.

[![Tiny Tapeout](https://img.shields.io/badge/Tiny%20Tapeout-Submitted-blue)](https://tinytapeout.com)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)

## What does it do?

This chip computes CRC-32 checksums (IEEE 802.3, polynomial 0xEDB88320 — the same standard
used in Ethernet, ZIP, and PNG) and displays the engine state in real time on a VGA monitor.

A host microcontroller feeds data bytes into an 8-byte FIFO via a simple 8-bit data bus.
The CRC engine processes one bit per clock cycle and raises an interrupt when the result is
ready. The result is then read back byte by byte through the same bus.

Simultaneously, a 640×480 VGA display shows:

- A **FIFO occupancy bar** that grows as bytes are written
- A **color-coded FSM panel** showing the current engine state
- A **live CRC bit display** updating as the engine runs

## Project details

| Field | Value |
|-------|-------|
| **Author** | Jorge Luis Chuquimia Parra |
| **GitHub** | [27jorge05](https://github.com/27jorge05) |
| **Module name** | `tt_um_27jorge05_crc_fifo` |
| **Language** | Verilog |
| **Clock** | 25 MHz |
| **Tile size** | 1 × 1 |
| **CRC standard** | IEEE 802.3 (0xEDB88320) |
| **FIFO depth** | 8 bytes |

## Quick start

### Prerequisites

- Tiny Tapeout demo board (or FPGA with bitstream)
- TinyVGA PMOD connected to `uo_out`
- Microcontroller (Arduino, RP2040, ESP32, etc.)

### Write data and read CRC
1.  Assert enable   → ui_in[6] = 1
2.  Write byte      → place byte on uio_in, pulse ui_in[0] (wr)
3.  Wait for IRQ    → poll ui_in read addr=0, bit 3; or watch VGA IRQ block turn red
4.  Read result     → rd=1, addr=1..4, collect 4 bytes from uio_out (LSB first)
5.  Reset for next  → pulse ui_in[7] (rst_crc)

### Verify with Python

```python
import binascii
message = b"Hello"
expected = binascii.crc32(message) & 0xFFFFFFFF
print(f"Expected CRC-32: 0x{expected:08X}")
```

## Pin summary

| Pin | Function |
|-----|----------|
| `ui_in[0]` | `wr` — write byte into FIFO |
| `ui_in[1]` | `rd` — read register onto data bus |
| `ui_in[5:2]` | `addr[3:0]` — register select |
| `ui_in[6]` | `enable` — enable engine |
| `ui_in[7]` | `rst_crc` — soft reset |
| `uo_out[7:0]` | VGA output (TinyVGA PMOD: HSync, RGB, VSync) |
| `uio[7:0]` | Bidirectional data bus |

## VGA display regions

| Screen rows | Content |
|-------------|---------|
| 0–79 | Solid blue header |
| 90–149 | FIFO occupancy bar (green, scales with byte count) |
| 160–219 | FSM state / IRQ / enable / rst_crc status blocks |
| 230–309 | CRC[7:0] bit display (orange = 1, dark blue = 0) |
| Remaining | Black background |

## Repository structure
├── src/
│   ├── project.v           # Top-level module (CRC engine + VGA)
│   └── hvsync_generator.v  # VGA sync signal generator
├── test/
│   ├── test.py             # CocoTB testbench (VGA frame capture)
│   ├── tb.v                # Verilog testbench wrapper
│   └── Makefile
├── docs/
│   └── info.md             # Full project documentation
├── info.yaml               # Tiny Tapeout project metadata
└── README.md               # This file

## Running the simulation

```bash
cd test
make test
```

On the first run, reference VGA frames are generated automatically. Subsequent runs compare
against those references pixel-by-pixel.

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that makes it affordable to get digital designs
manufactured on a real chip. Learn more at [tinytapeout.com](https://tinytapeout.com).

## License

[Apache 2.0](LICENSE) — Copyright (c) 2024 Jorge Luis Chuquimia Parra
