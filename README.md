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

### Adafruit ICM-20948 9-DoF IMU Wiring (SPI3 Interface)

Connect the **Adafruit TDK InvenSense ICM-20948 9-DoF IMU** breakout to the NUCLEO-WBA65RI board using the dedicated `SPI3` peripheral:

| IMU Breakout Pin | STM32WBA Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **VIN** | `3.3V` / `5V` | Power Supply | Adafruit board has onboard regulator |
| **GND** | `GND` | Ground | Common Ground |
| **SCK / SCL** | `PA0` | `SPI3` SCK (AF6) | 1 MHz SPI Init Clock |
| **MOSI / SDA** | `PD5` | `SPI3` MOSI (AF5) | SPI Data In (Replaces PB8 to avoid LED3 loading conflict) |
| **MISO / SDO** | `PA1` | `SPI3` MISO (AF6) | SPI Data Out |
| **CS** | `PA4` | Output GPIO (`PA4`) | Active-Low Chip Select |

#### 🎯 IMU Auto-Steer Controls in DOOM Demo (`doom_demo`)
- **Activation**: **Triple-press B2 (center button)** within 1.25s (75 frames) to toggle IMU Auto-Steer mode on/off. When active, `[ IMU AUTO-STEER: ON ]` toast banner pops up.
- **Auto-Forward Walk**: In IMU mode, the player walks forward through the 3D maze automatically at a constant, comfortable pace—keeping the LCD screen pointed directly at your eyes at all times without wrist strain.
- **Proportional Steering (Yaw)**: **Spin/rotate the board horizontally left or right (Yaw)**. Fused Gyroscope Z-axis angular rate + Accel X-axis lateral tilt steers the camera through hallways.
- **Pitch Brake / Reverse**: **Tilt the board backward / up toward your face**. Acts as a brake to pause forward movement or slowly reverse. Tilting slightly down gives a sprint boost.
- **Additive Control**: Button, Touchscreen, and USB HID keyboard/mouse/gamepad controls remain fully active alongside the IMU.

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

---

## 🎮 Executable Demos

### 1. 3D Physics & Wireframe Mesh Demo (`stm32wba-tftdisplay`)
- Real-time 3D rigid-body physics, mesh rotations, color cycling, and HUD metrics running at ~85 FPS.
- Flash & Run:
  ```bash
  cargo run --release --bin stm32wba-tftdisplay
  ```

### 2. DOOM E1M1-Inspired 3D Level Walkthrough Demo (`doom_demo`)
- DDA Fast Raycasting 3D engine with 16x16 E1M1 map layout, wall height projection, distance lighting attenuation, depth-sorted 3D billboarded sprites (Barrels, Health Kits, Imp Enemies), animated DOOM Guy status avatar, radar minimap, and weapon recoil.
- Flash & Run:
  ```bash
  cargo run --release --bin doom_demo
  ```

---

## 🚀 Building & Flashing

### Build Release Binaries
```bash
cargo build --release --bins
```

### Flash with Probe-rs
```bash
probe-rs run --chip STM32WBA65RI target/thumbv8m.main-none-eabihf/release/doom_demo
```
