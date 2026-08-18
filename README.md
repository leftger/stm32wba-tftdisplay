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

#### 🎯 6-DoF Madgwick AHRS Controls in DOOM Demo (`doom_demo`)
- **Activation**: **Triple-press B2 (center button)** within 1.25s (75 frames) to toggle Madgwick AHRS mode on/off. When active, `[ MADGWICK AHRS: ON ]` toast banner pops up.
- **Madgwick Quaternion Orientation**: Fuses Accelerometer gravity vector and Gyroscope 3D rotational rates into 3D Quaternions ($q_w, q_x, q_y, q_z$) in real-time at 60 FPS using hardware Cortex-M33 FPU.
- **1-to-1 Absolute Steering**: Turning the physical board 45° physically turns the in-game 3D viewport **exactly 45°** with zero gyro drift!
- **Auto-Forward Walk**: Player walks forward through the 3D maze automatically at a steady, comfortable pace—keeping the LCD display facing your eyes cleanly at all times.
- **Additive Control**: Button, Touchscreen, and USB HID keyboard/mouse/gamepad controls remain fully active alongside the IMU.

---

## 🎨 3D Engine & Features

- **3D Renderer**: `embedded-3dgfx` v0.5 (slim features: `row_width_320` / `depth-u16`, plus per-bin gates)
- **Display Driver**: `mipidsi` v0.9 + `embedded-hal-bus::spi::ExclusiveDevice`
- **Math Library**: `nalgebra` v0.35
- **Animations**:
  - Continuous 3D rotation (Roll, Pitch, Yaw) across multiple 3D meshes (Cube and Octahedron).
  - Dynamic palette color cycling (Cyan, Magenta, Yellow, Green, Red, White).
  - Render mode transitions (Wireframe Lines, Point Cloud, Solid Triangles).
  - Real-time 2D HUD text overlay using `embedded-graphics`.

### Mesh / normal input contract

Lit render modes in `embedded-3dgfx` expect **face normals** in `Geometry::normals` (one per triangle) and optional **vertex normals** in `Geometry::vertex_normals` (one per vertex, for Gouraud). Feeding only vertex normals leaves solid/lit modes blank while wireframe still works.

See **[docs/geometry.md](docs/geometry.md)** for field meanings, winding, double-sided meshes, and per-`RenderMode` requirements.

## 🎮 Executable Demos

### 1. 3D Physics & Wireframe Mesh Demo (`stm32wba-tftdisplay`)
- Real-time 3D rigid-body physics, mesh rotations, color cycling, and HUD metrics running at ~85 FPS.
- Flash & Run:
  ```bash
  cargo run --release --bin stm32wba-tftdisplay --features physics
  ```

### 2. DOOM E1M1-Inspired 3D Level Walkthrough Demo (`doom_demo`)
- DDA Fast Raycasting 3D engine with 16x16 E1M1 map layout, wall height projection, distance lighting attenuation, depth-sorted 3D billboarded sprites (Barrels, Health Kits, Imp Enemies), animated DOOM Guy status avatar, radar minimap, and weapon recoil.
- Flash & Run:
  ```bash
  cargo run --release --bin doom_demo --features doom
  ```

### 4. Embedded GUI Studio Live Display Agent (`studio_agent`)
- Flash-once base firmware that turns the board into a live remote display for [`embedded-gui-studio`](../embedded-gui/crates/embedded-gui-studio). It enumerates as a **vendor-specific USB-HS bulk device (PD6 = DP, PD7 = DM)** with 512-byte IN/OUT endpoints and paints RGB565 rectangles pushed by Studio straight to the ILI9341 panel — no reflash as the host GUI changes.
- Wire format and codec: [`embedded-gui-live`](../embedded-gui/crates/embedded-gui-live). Studio renders a screen through the real `embedded-gui` `GuiContext`, diffs it, and streams changed 40x40 tiles. The agent copies decoded tiles into a two-slot queue, byte-swaps RGB565 once, and sends them from a dedicated asynchronous GPDMA SPI task. USB reception therefore overlaps the current panel write and naturally backpressures if SPI gets more than two tiles behind.
- **Connect the user USB-HS port to the PC** (not the ST-Link connector). ST-Link stays attached for flashing/RTT.
- Flash & Run:
  ```bash
  cargo run --release --bin studio_agent --features studio-agent
  ```
- Then in Studio: pick `wba65-live` under **Connect USB**, hit **Connect**, and edit KDL — the panel updates in real time (leave **Live** checked). Press Play to stream active busy-wheel and plotter animation phases at up to 30 FPS using dirty tiles. No `/dev/cu.*` port is created; Studio claims the native bulk interface directly with `nusb`.

### 3. Madgwick / VQF Orientation Sync Demo (`orientation_demo`)
- ICM-20948-driven spacecraft mesh with VQF (default) or Madgwick AHRS, lit render-mode cycle, and toon ink overlay.
- Geometry for this demo follows the face-normal contract in [docs/geometry.md](docs/geometry.md).
- **Controls**: **B1** cycles render mode, **B2** re-zeros attitude (**hold ~1 s** to start/stop magnetometer calibration), **B3** resets the filter (hold to swap AHRS backend).
- **Heading drift**: yaw is the one axis gravity cannot constrain, so it depends on magnetometer quality and gyro Z bias.
  - Hold **B2** for ~1 s, rotate the board through all orientations until the sample counter passes the minimum, then hold **B2** again to apply. The HUD shows `MCAL` once a calibration is active (`RAWMAG` means uncorrected).
  - Calibration removes hard-iron offset and equalizes soft-iron axis gain; afterwards samples whose field magnitude strays too far are rejected as disturbances (see `mag_rejects` in the defmt log).
  - Keep the board still for a few seconds now and then so VQF's rest-phase gyro bias estimation can run (`REST` on the HUD, `online_bias` in the log).
- Flash & Run:
  ```bash
  cargo run --release --bin orientation_demo --features lighting
  ```

---

## 🚀 Building & Flashing

### Build Release Binaries
```bash
# Each bin declares required-features; build them individually:
cargo build --release --bin stm32wba-tftdisplay --features physics
cargo build --release --bin orientation_demo --features lighting
cargo build --release --bin doom_demo --features doom
cargo build --release --bin studio_agent --features studio-agent
```

### Flash with Probe-rs
```bash
probe-rs run --chip STM32WBA65RI target/thumbv8m.main-none-eabihf/release/doom_demo
```
