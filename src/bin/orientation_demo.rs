#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::fmt::Write;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::rcc::*;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::{Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;

use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::{Baseline, Text};

use mipidsi::interface::SpiInterface;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::Builder;

use icm20948::{AccelConfig, AccelFullScale, GyroConfig, GyroFullScale, Icm20948Driver, SpiInterface as ImuSpiInterface};
use libm::{asinf, atan2f, sqrtf};

use embedded_3dgfx::command_buffer::CommandBuffer;
use embedded_3dgfx::config::apply_default_caps;
use embedded_3dgfx::mesh::{Geometry, K3dMesh, RenderMode};
use embedded_3dgfx::renderer::FrameCtx;
use embedded_3dgfx::K3dengine;
use nalgebra::{Point3, Vector3};

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL0 => InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => InterruptHandler<peripherals::GPDMA1_CH1>;
});

const VIEW_WIDTH: usize = 320;
const VIEW_HEIGHT: usize = 240;
const VIEW_PIXELS: usize = VIEW_WIDTH * VIEW_HEIGHT;

struct FrameBuffer([Rgb565; VIEW_PIXELS]);
struct SafeFrameBuf(UnsafeCell<FrameBuffer>);
unsafe impl Sync for SafeFrameBuf {}
static RAW_FRAMEBUF: SafeFrameBuf =
    SafeFrameBuf(UnsafeCell::new(FrameBuffer([Rgb565::BLACK; VIEW_PIXELS])));

struct ZBuffer([u16; VIEW_PIXELS]);
struct SafeZBuf(UnsafeCell<ZBuffer>);
unsafe impl Sync for SafeZBuf {}
static RAW_ZBUF: SafeZBuf = SafeZBuf(UnsafeCell::new(ZBuffer([u16::MAX; VIEW_PIXELS])));

struct MicroDelay;
impl embedded_hal::delay::DelayNs for MicroDelay {
    #[inline(always)]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * 100) / 1000;
        cortex_m::asm::delay(cycles as u32);
    }
}

struct TxBuf([u8; 32768]);
struct SafeTxBuf(UnsafeCell<TxBuf>);
unsafe impl Sync for SafeTxBuf {}
static RAW_TX_BUF: SafeTxBuf = SafeTxBuf(UnsafeCell::new(TxBuf([0u8; 32768])));

#[derive(Clone, Copy)]
struct DirtyRect {
    min_x: usize,
    min_y: usize,
    max_x: usize,
    max_y: usize,
}

impl DirtyRect {
    fn empty() -> Self {
        Self {
            min_x: VIEW_WIDTH,
            min_y: VIEW_HEIGHT,
            max_x: 0,
            max_y: 0,
        }
    }

    #[inline(always)]
    fn add_point(&mut self, x: usize, y: usize) {
        if x < self.min_x {
            self.min_x = x;
        }
        if y < self.min_y {
            self.min_y = y;
        }
        if x > self.max_x {
            self.max_x = x;
        }
        if y > self.max_y {
            self.max_y = y;
        }
    }
}

struct OffscreenBuffer<'a> {
    pixels: &'a mut [Rgb565; VIEW_PIXELS],
    dirty: DirtyRect,
}

impl<'a> OriginDimensions for OffscreenBuffer<'a> {
    fn size(&self) -> Size {
        Size::new(VIEW_WIDTH as u32, VIEW_HEIGHT as u32)
    }
}

impl<'a> DrawTarget for OffscreenBuffer<'a> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    #[inline]
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < VIEW_WIDTH as i32
                && coord.y >= 0
                && coord.y < VIEW_HEIGHT as i32
            {
                let x = coord.x as usize;
                let y = coord.y as usize;
                let idx = y * VIEW_WIDTH + x;
                self.pixels[idx] = color;
                self.dirty.add_point(x, y);
            }
        }
        Ok(())
    }
}

// ----------------------------------------------------------------------------
// 6-DoF Madgwick AHRS Orientation Filter (no_std, Cortex-M33 FPU Optimized)
// ----------------------------------------------------------------------------
pub struct MadgwickFilter {
    pub beta: f32,
    pub q: [f32; 4], // Quaternion [w, x, y, z]
}

impl MadgwickFilter {
    pub const fn new(beta: f32) -> Self {
        Self {
            beta,
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn reset(&mut self) {
        self.q = [1.0, 0.0, 0.0, 0.0];
    }

    pub fn update_6dof(&mut self, gx: f32, gy: f32, gz: f32, ax: f32, ay: f32, az: f32, dt: f32) {
        let (q1, q2, q3, q4) = (self.q[0], self.q[1], self.q[2], self.q[3]);

        let norm_a = sqrtf(ax * ax + ay * ay + az * az);
        if norm_a < 0.001 {
            return;
        }
        let ax = ax / norm_a;
        let ay = ay / norm_a;
        let az = az / norm_a;

        let _2q1 = 2.0 * q1;
        let _2q2 = 2.0 * q2;
        let _2q3 = 2.0 * q3;
        let _2q4 = 2.0 * q4;
        let _4q1 = 4.0 * q1;
        let _4q2 = 4.0 * q2;
        let _4q3 = 4.0 * q3;
        let _8q2 = 8.0 * q2;
        let _8q3 = 8.0 * q3;
        let q1q1 = q1 * q1;
        let q2q2 = q2 * q2;
        let q3q3 = q3 * q3;
        let q4q4 = q4 * q4;

        let s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        let s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
        let s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        let s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay;

        let norm_s = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
        let (s1, s2, s3, s4) = if norm_s > 0.00001 {
            (s1 / norm_s, s2 / norm_s, s3 / norm_s, s4 / norm_s)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        let q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1;
        let q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2;
        let q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3;
        let q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4;

        let q1_new = q1 + q_dot1 * dt;
        let q2_new = q2 + q_dot2 * dt;
        let q3_new = q3 + q_dot3 * dt;
        let q4_new = q4 + q_dot4 * dt;

        let norm_q = sqrtf(q1_new * q1_new + q2_new * q2_new + q3_new * q3_new + q4_new * q4_new);
        if norm_q > 0.00001 {
            self.q[0] = q1_new / norm_q;
            self.q[1] = q2_new / norm_q;
            self.q[2] = q3_new / norm_q;
            self.q[3] = q4_new / norm_q;
        }
    }

    pub fn euler_angles(&self) -> (f32, f32, f32) {
        let (w, x, y, z) = (self.q[0], self.q[1], self.q[2], self.q[3]);

        let roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        let sin_pitch = 2.0 * (w * y - z * x);
        let pitch = if sin_pitch.abs() >= 1.0 {
            if sin_pitch > 0.0 { core::f32::consts::FRAC_PI_2 } else { -core::f32::consts::FRAC_PI_2 }
        } else {
            asinf(sin_pitch)
        };
        let yaw = atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

        (pitch, roll, yaw)
    }
}

// ----------------------------------------------------------------------------
// 3D Object Geometry Definitions (Spacecraft Gimbal Object)
// ----------------------------------------------------------------------------
static SPACECRAFT_VERTICES: [[f32; 3]; 8] = [
    [0.0, 0.0, 3.3],    // 0: Nose Cone
    [-2.25, 0.0, -1.8], // 1: Left Wing
    [2.25, 0.0, -1.8],  // 2: Right Wing
    [0.0, 1.2, -1.5],   // 3: Canopy Top
    [0.0, -0.9, -1.5],  // 4: Fuselage Bottom
    [0.0, 2.1, -2.1],   // 5: Vertical Tail Stabilizer
    [-0.6, 0.0, -2.1],  // 6: Left Thruster
    [0.6, 0.0, -2.1],   // 7: Right Thruster
];

static SPACECRAFT_FACES: [[usize; 3]; 20] = [
    // Front Winding
    [0, 3, 1], [0, 2, 3], [0, 1, 4], [0, 4, 2],
    [3, 5, 1], [3, 2, 5], [1, 6, 4], [2, 4, 7],
    [5, 6, 1], [5, 2, 7],
    // Back Winding (Double-sided rendering across 360° orientations)
    [0, 1, 3], [0, 3, 2], [0, 4, 1], [0, 2, 4],
    [3, 1, 5], [3, 5, 2], [1, 4, 6], [2, 7, 4],
    [5, 1, 6], [5, 7, 2],
];

static SPACECRAFT_NORMALS: [[f32; 3]; 8] = [
    [0.0, 0.0, 1.0],   // 0: Nose Cone
    [-0.8, 0.0, -0.6], // 1: Left Wing
    [0.8, 0.0, -0.6],  // 2: Right Wing
    [0.0, 0.9, -0.4],  // 3: Canopy Top
    [0.0, -0.9, -0.4], // 4: Fuselage Bottom
    [0.0, 1.0, -0.2],  // 5: Vertical Tail Stabilizer
    [-0.5, 0.0, -0.8], // 6: Left Thruster
    [0.5, 0.0, -0.8],  // 7: Right Thruster
];

struct ArrayString<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> ArrayString<N> {
    fn new() -> Self {
        Self { buf: [0; N], len: 0 }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl<const N: usize> Write for ArrayString<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let rem = N - self.len;
        let to_copy = bytes.len().min(rem);
        self.buf[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul25,
        divp: Some(PllDiv::Div25),
        divq: None,
        divr: Some(PllDiv::Div4), // 100 MHz SYSCLK
        frac: Some(0),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div1;
    config.rcc.apb2_pre = APBPrescaler::Div1;
    config.rcc.voltage_scale = VoltageScale::Range1;
    let p = embassy_stm32::init(config);

    defmt::info!("============================================");
    defmt::info!("3D Madgwick AHRS Orientation Sync Demo");
    defmt::info!("100MHz Cortex-M33 + Hardware FPU + 6-DoF AHRS");
    defmt::info!("============================================");

    let mut display_spi_config = SpiConfig::default();
    display_spi_config.frequency = Hertz(25_000_000);
    let spi = Spi::new_blocking_txonly(p.SPI2, p.PB10, p.PC3, display_spi_config);
    let cs_display = Output::new(p.PB9, Level::High, Speed::VeryHigh);
    let dc = Output::new(p.PB11, Level::Low, Speed::VeryHigh);
    let rst = Output::new(p.PA10, Level::High, Speed::VeryHigh);

    let tx_buf = unsafe { &mut (*RAW_TX_BUF.0.get()).0 };
    let spi_device = ExclusiveDevice::new(spi, cs_display, MicroDelay).unwrap();
    let di = SpiInterface::new(spi_device, dc, tx_buf.as_mut_slice());

    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .init(&mut embassy_time::Delay)
        .unwrap();

    let btn1 = Input::new(p.PC13, Pull::Up); // Mode Toggle
    let btn2 = Input::new(p.PC5, Pull::Up);  // Zero-Point Calibrate
    let btn3 = Input::new(p.PB4, Pull::Up);  // Reset Filter

    cortex_m::asm::delay(1_000_000);

    let mut imu_spi_config = SpiConfig::default();
    imu_spi_config.frequency = Hertz(1_000_000);
    let imu_spi = Spi::new_blocking(
        p.SPI3,
        p.PA0, // SCK
        p.PD5, // MOSI
        p.PA1, // MISO
        imu_spi_config,
    );
    let imu_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let imu_spi_dev = ExclusiveDevice::new(imu_spi, imu_cs, MicroDelay).unwrap();
    let imu_interface = ImuSpiInterface::new(imu_spi_dev);
    let mut imu = Icm20948Driver::new(imu_interface);

    let imu_ok = (|| -> Result<(), &'static str> {
        imu.init(&mut embassy_time::Delay).map_err(|_| "imu.init() failed")?;
        imu.enable_spi_mode().map_err(|_| "imu.enable_spi_mode() failed")?;
        let accel_cfg = AccelConfig {
            full_scale: AccelFullScale::G2,
            ..AccelConfig::default()
        };
        imu.configure_accelerometer(accel_cfg).map_err(|_| "configure_accelerometer() failed")?;
        let gyro_cfg = GyroConfig {
            full_scale: GyroFullScale::Dps500,
            ..GyroConfig::default()
        };
        imu.configure_gyroscope(gyro_cfg).map_err(|_| "configure_gyroscope() failed")?;
        defmt::info!("IMU: ICM-20948 initialized successfully (SPI3)");
        Ok(())
    })().is_ok();

    if !imu_ok {
        defmt::warn!("IMU initialization failed! Check SPI3 wiring (PA0, PD5, PA1, PA4).");
    }

    let mut madgwick = MadgwickFilter::new(0.45);

    let mut engine = K3dengine::new(VIEW_WIDTH as u16, VIEW_HEIGHT as u16);
    apply_default_caps(&mut engine);
    engine.camera.set_position(Point3::new(0.0, 0.5, 6.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));

    let craft_geo = Geometry {
        vertices: &SPACECRAFT_VERTICES,
        faces: &SPACECRAFT_FACES,
        colors: &[],
        lines: &[],
        normals: &[],
        vertex_normals: &SPACECRAFT_NORMALS,
        uvs: &[],
        texture_id: None,
    };
    let light_dir = Vector3::new(0.5f32, 0.8f32, -0.7f32).normalize();
    let mut craft_mesh = K3dMesh::new(craft_geo);
    craft_mesh.set_render_mode(RenderMode::GouraudLightDir(light_dir));
    craft_mesh.set_color(Rgb565::CYAN);

    let mut commands = CommandBuffer::<512>::new();

    let render_modes = [
        RenderMode::GouraudLightDir(light_dir),
        RenderMode::BlinnPhong {
            light_dir,
            specular_intensity: 0.6,
            shininess: 16.0,
        },
        RenderMode::SolidLightDir(light_dir),
        RenderMode::Lines,
    ];
    let mut render_mode_idx = 0usize;

    let mut zero_pitch = 0.0f32;
    let mut zero_roll = 0.0f32;
    let mut zero_yaw = 0.0f32;

    let mut b1_was_pressed = false;
    let mut b2_was_pressed = false;
    let mut b3_was_pressed = false;

    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
    let val_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);

    let mut frame_count: u32 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(16)); // Target 60 FPS

    defmt::info!("Entering 3D Orientation Render Loop...");

    loop {
        let framebuf = unsafe { &mut (*RAW_FRAMEBUF.0.get()).0 };
        let zbuf = unsafe { &mut (*RAW_ZBUF.0.get()).0 };

        framebuf.fill(Rgb565::BLACK);
        zbuf.fill(u16::MAX);

        // 1. Buttons
        let b1_pressed = btn1.is_low();
        if b1_pressed && !b1_was_pressed {
            render_mode_idx = (render_mode_idx + 1) % render_modes.len();
            craft_mesh.set_render_mode(render_modes[render_mode_idx].clone());
            defmt::info!("Render Mode changed to {:?}", render_mode_idx);
        }
        b1_was_pressed = b1_pressed;

        let b2_pressed = btn2.is_low();
        if b2_pressed && !b2_was_pressed {
            let (p, r, y) = madgwick.euler_angles();
            zero_pitch = p;
            zero_roll = r;
            zero_yaw = y;
            defmt::info!("Zero calibrated: P={=f32} R={=f32} Y={=f32}", p, r, y);
        }
        b2_was_pressed = b2_pressed;

        let b3_pressed = btn3.is_low();
        if b3_pressed && !b3_was_pressed {
            madgwick.reset();
            zero_pitch = 0.0;
            zero_roll = 0.0;
            zero_yaw = 0.0;
            defmt::info!("Madgwick filter reset");
        }
        b3_was_pressed = b3_pressed;

        // 2. Poll IMU & Execute Madgwick AHRS Filter Update
        if imu_ok {
            match (imu.read_accelerometer(), imu.read_gyroscope_radians()) {
                (Ok(accel), Ok(gyro)) => {
                    madgwick.update_6dof(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, 0.01667);
                }
                (Err(_e_a), _) => {
                    if frame_count % 60 == 0 {
                        defmt::warn!("IMU Accel Read Error on SPI3!");
                    }
                }
                (_, Err(_e_g)) => {
                    if frame_count % 60 == 0 {
                        defmt::warn!("IMU Gyro Read Error on SPI3!");
                    }
                }
            }
        }

        let (raw_pitch, raw_roll, raw_yaw) = madgwick.euler_angles();
        let pitch = raw_pitch - zero_pitch;
        let roll = raw_roll - zero_roll;
        let yaw = raw_yaw - zero_yaw;

        let pitch_deg = (pitch * (180.0 / core::f32::consts::PI)) as i32;
        let roll_deg = (roll * (180.0 / core::f32::consts::PI)) as i32;
        let yaw_deg = (yaw * (180.0 / core::f32::consts::PI)) as i32;

        if frame_count % 60 == 0 {
            defmt::info!(
                "3D AHRS | Pitch={}° Roll={}° Yaw={}° | imu_ok={}",
                pitch_deg, roll_deg, yaw_deg, imu_ok
            );
        }

        // 3. Update 3D Object Transformation Matrix (X=Pitch, Y=Yaw, Z=Roll)
        craft_mesh.set_attitude(pitch, yaw, roll);

        // 4. Render 3D Scene to Offscreen Buffer
        let mut offscreen = OffscreenBuffer {
            pixels: framebuf,
            dirty: DirtyRect::empty(),
        };
        commands.clear();
        if let Ok(()) = engine.record([&craft_mesh].into_iter(), &mut commands, None) {
            let mut frame = FrameCtx {
                zbuffer: zbuf,
                width: VIEW_WIDTH,
                height: VIEW_HEIGHT,
            };
            let _ = engine.execute::<_, 512>(&mut offscreen, &mut frame, &commands, None);
        }

        // 5. Render Real-Time On-Screen Telemetry HUD Text
        {
            let mut hud_str = ArrayString::<128>::new();
            let _ = write!(
                hud_str,
                "MADGWICK 3D AHRS\nPITCH:{:4}° ROLL:{:4}° YAW:{:4}°",
                pitch_deg, roll_deg, yaw_deg
            );
            let _ = Text::with_baseline(
                hud_str.as_str(),
                Point::new(8, 8),
                text_style,
                Baseline::Top,
            )
            .draw(&mut offscreen);

            let mut bot_str = ArrayString::<64>::new();
            let _ = write!(bot_str, "B1:MODE B2:CALIB B3:RESET");
            let _ = Text::with_baseline(
                bot_str.as_str(),
                Point::new(8, 222),
                val_style,
                Baseline::Top,
            )
            .draw(&mut offscreen);
        }

        // 6. Blit Framebuffer to ILI9341 LCD via SPI DMA
        let view_area = Rectangle::new(
            Point::new(0, 0),
            Size::new(VIEW_WIDTH as u32, VIEW_HEIGHT as u32),
        );
        let _ = display.fill_contiguous(
            &view_area,
            framebuf.iter().copied(),
        );

        frame_count += 1;
        ticker.next().await;
    }
}
