<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

# CRC_FIFO: Motor CRC-32 con FIFO de 2 KiB

## How it works

This project implements a CRC-32 integrity verification engine with a 2 KiB internal FIFO buffer,
designed for edge AI systems where data reliability is critical.

### Core architecture

The design processes incoming data bytes through a parallel CRC-32 engine (polynomial 0x04C11DB7,
the same standard used in Ethernet, ZIP, and PNG). Instead of computing one bit per clock cycle,
the engine operates on 32-bit words assembled from the FIFO, delivering results in a fraction of
the time required by serial implementations.

The internal blocks are:

- **2 KiB FIFO buffer** (2048 bytes): stores incoming data frames while the CRC engine processes
  them. Implemented as a circular register array with write/read pointers. This allows the host
  processor to write a complete data block and then trigger verification without holding the bus.
- **Parallel CRC-32 engine**: accepts 32-bit words and updates the CRC register in a single clock
  cycle using a combinational XOR tree derived from the standard polynomial equations.
- **Control FSM**: orchestrates the flow — waiting for data, assembling 4-byte words from the FIFO,
  feeding them to the CRC engine, and finalizing the result (bit-inverted per the Ethernet standard).
- **Register interface**: exposes internal state (FIFO occupancy, CRC result bytes, channel status)
  through a simple 4-bit address / 8-bit data bus accessible from an external microcontroller.
- **IRQ output**: signals when the CRC result is ready or when the FIFO is full, allowing
  interrupt-driven operation without polling.

### Data flow

1. The host writes data bytes to the FIFO via `ui_in` (write strobe + address 0).
2. Once at least 4 bytes are available, the FSM assembles a 32-bit word and feeds it to the CRC engine.
3. The engine updates its internal register in one clock cycle.
4. Steps 2–3 repeat until the FIFO is drained.
5. The final CRC value is bit-inverted and stored in result registers.
6. The `irq` output goes high to notify the host processor.
7. The host reads the 4-byte CRC result from registers via `ui_in` (read strobe + addresses 1–4).

### Why CRC-32 matters for edge AI

An intelligent agent operating on a noisy edge environment (wireless links, long cables, cosmic
ray bit flips in SRAM) must verify that sensor data and model weights are intact before using them
for inference. A hardware CRC engine offloads this task from the main processor, running at wire
speed without consuming CPU cycles.

## How to test

### Simulation

Run the included CocoTB testbench:

```bash
cd test
make test
```

A passing result confirms the module compiles and the interface is functional.

### Hardware testing (once the chip is fabricated)

**Required equipment:**
- Tiny Tapeout demo board (or any FPGA with the bitstream loaded)
- Microcontroller (Arduino, RP2040, or similar) connected to the chip's I/O pins
- Optional: logic analyzer for signal inspection

**Step-by-step procedure:**

1. Connect the microcontroller to the chip according to the pinout table below.
2. Assert `enable` (ui_in[6] = 1) and release reset.
3. Write a test message byte by byte:
   - Set `addr` = 0 (ui_in[5:2] = 0000)
   - Place the byte on the data bus
   - Pulse `wr` (ui_in[0] = 1 for one clock cycle)
   - Repeat for each byte of the message
4. Wait for the `irq` output (uo_out[0]) to go high — this means the CRC is ready.
5. Read the 4-byte result:
   - Set `rd` = 1 (ui_in[1] = 1) and cycle through `addr` = 1, 2, 3, 4
   - Collect the 4 bytes returned on the data bus
6. Assemble the 32-bit CRC (byte 1 = LSB, byte 4 = MSB).
7. Compare with the expected CRC calculated offline (e.g., using Python's `binascii.crc32()`).

**Quick Python reference to compute expected CRC:**

```python
import binascii
message = b"Hello"
expected_crc = binascii.crc32(message) & 0xFFFFFFFF
print(f"Expected CRC-32: 0x{expected_crc:08X}")
```

If the chip returns the same value, the engine is working correctly.

## External hardware

- A microcontroller (Arduino Uno, Raspberry Pi Pico, ESP32, or similar) to drive the input bus
  and read CRC results.
- Optional LEDs on `uo_out[1:2]` (error flags) to visually indicate data integrity errors.
- Optional logic analyzer (Saleae or equivalent) for debugging the bus timing.

No additional hardware is required to use the module on the Tiny Tapeout demo FPGA board.

## Pin description

| Pin | Direction | Function |
|-----|-----------|----------|
| `ui_in[0]` | Input | `wr` — write strobe: pulse high to write `data` into FIFO (when addr=0) |
| `ui_in[1]` | Input | `rd` — read strobe: pulse high to read register at `addr` |
| `ui_in[5:2]` | Input | `addr[3:0]` — register address (0=FIFO write, 1–4=CRC result bytes, 9=status) |
| `ui_in[6]` | Input | `enable` — enables CRC processing; must be high during operation |
| `ui_in[7]` | Input | reserved |
| `uo_out[0]` | Output | `irq` — goes high when CRC result is ready or FIFO is full |
| `uo_out[2:1]` | Output | `error_flags[1:0]` — one bit per channel, high if a mismatch is detected |
| `uo_out[7:3]` | Output | reserved (tied low) |
| `uio[7:0]` | Bidir | `data[7:0]` — data bus: input when writing, output when reading |

## Performance and area

| Parameter | Value |
|-----------|-------|
| Logic cells (estimated) | ~20,000 (~40% of Tiny Tapeout limit) |
| Maximum clock frequency | 50 MHz |
| Operating clock | 25 MHz |
| Throughput | 32 bits/cycle → 800 Mbps at 25 MHz |
| FIFO capacity | 2,048 bytes |
| CRC polynomial | 0x04C11DB7 (Ethernet/IEEE 802.3) |
| Estimated power | < 10 mW active, < 1 µW idle |

## Author

**Jorge Luis Chuquimia Parra**  
GitHub: [27jorge05](https://github.com/27jorge05)
