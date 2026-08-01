# STM32WBA65RI TFT Display 3D Graphics Demo (Embassy + embedded-3dgfx)

An asynchronous Rust project for the **STM32WBA65RI** (Cortex-M33 with FPU) running **Embassy** and rendering real-time 3D animations using **`embedded-3dgfx`** on a 2.8" ILI9341 TFT display (DM-TFT28-116 from `dmtftlibrary`).

---

## 🛠️ Project Structure & Hardware Setup

### Microcontroller Target
- **MCU**: STM32WBA65RI (ARM Cortex-M33 with FPU, 1MB Flash, 128KB SRAM)
- **Target Triple**: `thumbv8m.main-none-eabihf`
- **Framework**: [Embassy](https://embassy.dev) (using local path dependency `../../embassy-project/embassy`)

### Display & Wiring Pinout (Arduino Shield Layout)

The **DM-TFT28-116** TFT module (ILI9341 controller) connects to the NUCLEO-WBA65RI Arduino Uno V3 headers as follows:

| Function | Arduino Pin | STM32WBA Pin | Embassy Signal |
| :--- | :--- | :--- | :--- |
| **SPI SCK** | D13 | `PB10` | `SPI2` SCK |
| **SPI MOSI** | D11 | `PC3` | `SPI2` MOSI |
| **SPI MISO** | D12 | `PA9` | `SPI2` MISO |
| **TFT CS** | D10 | `PB9` | Output GPIO (Chip Select) |
| **TFT DC** | D9 | `PB11` | Output GPIO (Data/Command) |
| **TFT RESET** | D8 | `PA10` | Output GPIO (Reset) |
| **Power** | 5V / 3.3V & GND | 5V / 3.3V & GND | Power Supply Pins |

---

## 🎨 3D Engine & Features

- **3D Renderer**: `embedded-3dgfx` v0.4.1 (with `row_width_240` and `depth-u16` features optimized for 128KB SRAM)
- **Display Driver**: `mipidsi` v0.9 + `embedded-hal-bus::spi::ExclusiveDevice`
- **Math Library**: `nalgebra` v0.34
- **Animations**:
  - Continuous 3D rotation (Roll, Pitch, Yaw) across multiple 3D meshes (Cube and Octahedron).
  - Dynamic palette color cycling (Cyan, Magenta, Yellow, Green, Red, White).
  - Render mode transitions (Wireframe Lines, Point Cloud, Solid Triangles).
  - Real-time 2D HUD text overlay using `embedded-graphics`.

---

## 🚀 Building & Flashing

### 1. Build Debug / Release Binaries
```bash
# Debug build
cargo build

# Optimized Release build
cargo build --release
```

### 2. Run & Flash with Probe-rs
```bash
cargo run --release
```
or directly via `probe-rs`:
```bash
probe-rs run --chip STM32WBA65RI target/thumbv8m.main-none-eabihf/release/stm32wba-tftdisplay
```
