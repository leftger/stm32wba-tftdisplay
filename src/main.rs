#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::fmt::Write;

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;

use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;

use mipidsi::interface::SpiInterface;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::Builder;

use embedded_3dgfx::command_buffer::CommandBuffer;
use embedded_3dgfx::config::apply_default_caps;
use embedded_3dgfx::mesh::{Geometry, K3dMesh, RenderMode};
use embedded_3dgfx::physics::{PhysicsWorld, RigidBody};
use embedded_3dgfx::renderer::FrameCtx;
use embedded_3dgfx::K3dengine;
use nalgebra::{Point3, Vector3};

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL0 => InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => InterruptHandler<peripherals::GPDMA1_CH1>;
});

// Full 320x240 Display Coverage
const VIEW_WIDTH: usize = 320;
const VIEW_HEIGHT: usize = 240;
const VIEW_PIXELS: usize = VIEW_WIDTH * VIEW_HEIGHT;

// Off-screen FrameBuffers & Z-Buffer stored safely in SRAM (~307 KB total)
struct FrameBuffer([Rgb565; VIEW_PIXELS]);
struct SafeFrameBuf(UnsafeCell<FrameBuffer>);
unsafe impl Sync for SafeFrameBuf {}
static RAW_FRAMEBUF: SafeFrameBuf = SafeFrameBuf(UnsafeCell::new(FrameBuffer([Rgb565::BLACK; VIEW_PIXELS])));

struct ZBuffer([u16; VIEW_PIXELS]);
struct SafeZBuf(UnsafeCell<ZBuffer>);
unsafe impl Sync for SafeZBuf {}
static RAW_ZBUF: SafeZBuf = SafeZBuf(UnsafeCell::new(ZBuffer([u16::MAX; VIEW_PIXELS])));

// ----------------------------------------------------------------------------
// Method A: Dirty Region Bounding Box Tracker
// ----------------------------------------------------------------------------

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

    fn add_point(&mut self, x: usize, y: usize) {
        if x < self.min_x { self.min_x = x; }
        if y < self.min_y { self.min_y = y; }
        if x > self.max_x { self.max_x = x; }
        if y > self.max_y { self.max_y = y; }
    }

    fn merge(&mut self, other: &DirtyRect) {
        if other.min_x < self.min_x { self.min_x = other.min_x; }
        if other.min_y < self.min_y { self.min_y = other.min_y; }
        if other.max_x > self.max_x { self.max_x = other.max_x; }
        if other.max_y > self.max_y { self.max_y = other.max_y; }
    }

    fn sanitize(&mut self) {
        if self.min_x >= VIEW_WIDTH { self.min_x = VIEW_WIDTH - 1; }
        if self.min_y >= VIEW_HEIGHT { self.min_y = VIEW_HEIGHT - 1; }
        if self.max_x >= VIEW_WIDTH { self.max_x = VIEW_WIDTH - 1; }
        if self.max_y >= VIEW_HEIGHT { self.max_y = VIEW_HEIGHT - 1; }
        if self.max_x < self.min_x { self.max_x = self.min_x; }
        if self.max_y < self.min_y { self.max_y = self.min_y; }
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
            if coord.x >= 0 && coord.x < VIEW_WIDTH as i32 && coord.y >= 0 && coord.y < VIEW_HEIGHT as i32 {
                let x = coord.x as usize;
                let y = coord.y as usize;
                let idx = y * VIEW_WIDTH + x;
                self.pixels[idx] = color;
                self.dirty.add_point(x, y);
            }
        }
        Ok(())
    }

    #[inline]
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let mut colors = colors.into_iter();
        let x_start = area.top_left.x.max(0) as usize;
        let y_start = area.top_left.y.max(0) as usize;
        let x_end = (area.top_left.x + area.size.width as i32).min(VIEW_WIDTH as i32) as usize;
        let y_end = (area.top_left.y + area.size.height as i32).min(VIEW_HEIGHT as i32) as usize;

        for y in y_start..y_end {
            let row_offset = y * VIEW_WIDTH;
            for x in x_start..x_end {
                if let Some(color) = colors.next() {
                    self.pixels[row_offset + x] = color;
                    self.dirty.add_point(x, y);
                }
            }
        }
        Ok(())
    }

    #[inline]
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.pixels.fill(color);
        self.dirty = DirtyRect { min_x: 0, min_y: 0, max_x: VIEW_WIDTH - 1, max_y: VIEW_HEIGHT - 1 };
        Ok(())
    }
}

// ----------------------------------------------------------------------------
// 3D Geometry Definitions
// ----------------------------------------------------------------------------

static CUBE_VERTICES: [[f32; 3]; 8] = [
    [-1.0, -1.0,  1.0],
    [ 1.0, -1.0,  1.0],
    [ 1.0,  1.0,  1.0],
    [-1.0,  1.0,  1.0],
    [-1.0, -1.0, -1.0],
    [ 1.0, -1.0, -1.0],
    [ 1.0,  1.0, -1.0],
    [-1.0,  1.0, -1.0],
];

static CUBE_FACES: [[usize; 3]; 12] = [
    [0, 1, 2], [0, 2, 3], // Front
    [5, 4, 7], [5, 7, 6], // Back
    [3, 2, 6], [3, 6, 7], // Top
    [4, 5, 1], [4, 1, 0], // Bottom
    [1, 5, 6], [1, 6, 2], // Right
    [4, 0, 3], [4, 3, 7], // Left
];

static OCTA_VERTICES: [[f32; 3]; 6] = [
    [ 0.0,  1.4,  0.0],
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  0.0, -1.0],
    [ 0.0, -1.4,  0.0],
];

static OCTA_FACES: [[usize; 3]; 8] = [
    [0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1],
    [5, 2, 1], [5, 3, 2], [5, 4, 3], [5, 1, 4],
];

static PALETTE: [Rgb565; 6] = [
    Rgb565::CYAN,
    Rgb565::MAGENTA,
    Rgb565::YELLOW,
    Rgb565::GREEN,
    Rgb565::RED,
    Rgb565::WHITE,
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
    let p = embassy_stm32::init(Config::default());
    defmt::info!("============================================");
    defmt::info!("STM32WBA65RI Full 320x240 Screen 3D Demo");
    defmt::info!("Optimizations: Full Screen 320x240 Bounding Box + 50MHz SPI");
    defmt::info!("============================================");

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(50_000_000); // 50 MHz SPI

    let spi = Spi::new(
        p.SPI2,
        p.PB10, // SCK
        p.PC3,  // MOSI
        p.PA9,  // MISO
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        Irqs,
        spi_config,
    );

    let cs = Output::new(p.PB9, Level::High, Speed::VeryHigh);
    let dc = Output::new(p.PB11, Level::Low, Speed::VeryHigh);
    let rst = Output::new(p.PA10, Level::High, Speed::VeryHigh);

    let spi_device = ExclusiveDevice::new(spi, cs, embassy_time::Delay).unwrap();
    let mut tx_buffer = [0u8; 4096];
    let di = SpiInterface::new(spi_device, dc, &mut tx_buffer);

    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .init(&mut embassy_time::Delay)
        .unwrap();

    // Initial background wipe once
    display.clear(Rgb565::new(2, 3, 6)).unwrap();

    let title_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
    let stats_style = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);

    let mut engine = K3dengine::new(VIEW_WIDTH as u16, VIEW_HEIGHT as u16);
    apply_default_caps(&mut engine);
    engine.camera.set_position(Point3::new(0.0, 1.8, 5.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));

    // Initialize 3D Physics World
    let mut physics_world = PhysicsWorld::<4>::new();
    physics_world.set_gravity(Vector3::new(0.0, -4.0, 0.0));

    let cube_body = RigidBody::new(1.0)
        .with_position(Vector3::new(-1.3, 1.0, 0.0))
        .with_velocity(Vector3::new(0.0, 1.5, 0.0));
    let cube_id = physics_world.add_body(cube_body).unwrap();

    let cube_geo = Geometry {
        vertices: &CUBE_VERTICES,
        faces: &CUBE_FACES,
        colors: &[],
        lines: &[],
        normals: &[],
        vertex_normals: &[],
        uvs: &[],
        texture_id: None,
    };
    let mut cube_mesh = K3dMesh::new(cube_geo);
    cube_mesh.set_render_mode(RenderMode::Lines);
    cube_mesh.set_color(Rgb565::CYAN);

    let octa_geo = Geometry {
        vertices: &OCTA_VERTICES,
        faces: &OCTA_FACES,
        colors: &[],
        lines: &[],
        normals: &[],
        vertex_normals: &[],
        uvs: &[],
        texture_id: None,
    };
    let mut octa_mesh = K3dMesh::new(octa_geo);
    octa_mesh.set_render_mode(RenderMode::Lines);
    octa_mesh.set_color(Rgb565::MAGENTA);

    let framebuf = unsafe { &mut (*RAW_FRAMEBUF.0.get()).0 };
    let zbuf = unsafe { &mut (*RAW_ZBUF.0.get()).0 };
    let mut commands = CommandBuffer::<512>::new();

    let mut angle: f32 = 0.0;
    let mut frame_count: u32 = 0;
    let mut palette_idx: usize = 0;

    let mut prev_dirty = DirtyRect::empty();
    let mut last_render_us: u64 = 0;
    let mut last_blit_us: u64 = 0;
    let mut last_total_ms: u64 = 0;

    let mut ticker = Ticker::every(Duration::from_millis(16)); // Target max FPS

    loop {
        let frame_start = Instant::now();

        // 1. Update 3D Physics Step
        physics_world.step::<4>(0.016);
        if let Some(body) = physics_world.body_mut(cube_id) {
            let pos = body.position;
            if pos.y < -1.5 {
                body.position = Vector3::new(-1.3, 1.5, 0.0);
                body.velocity = Vector3::new(0.0, 2.5, 0.0);
            }
            cube_mesh.set_position(pos.x, pos.y, pos.z);
        }

        angle += 0.04;
        if angle > core::f32::consts::TAU {
            angle -= core::f32::consts::TAU;
        }

        frame_count += 1;
        if frame_count % 90 == 0 {
            palette_idx = (palette_idx + 1) % PALETTE.len();
            cube_mesh.set_color(PALETTE[palette_idx]);
            octa_mesh.set_color(PALETTE[(palette_idx + 3) % PALETTE.len()]);

            let modes = [RenderMode::Lines, RenderMode::Points, RenderMode::Lines];
            let mode = modes[(frame_count / 90 % 3) as usize].clone();
            cube_mesh.set_render_mode(mode.clone());
            octa_mesh.set_render_mode(mode);
        }

        cube_mesh.set_attitude(angle * 0.8, angle, angle * 0.5);

        octa_mesh.set_position(1.3, 0.0, 0.0);
        octa_mesh.set_attitude(-angle * 0.6, angle * 1.2, -angle * 0.4);

        let render_start = Instant::now();

        // 2. Merge previous dirty rectangle to clear previous frame positions
        let mut active_dirty = DirtyRect::empty();
        active_dirty.merge(&prev_dirty);

        // Clear framebuf and zbuf ONLY in dirty region
        let bg_color = Rgb565::new(2, 3, 6);
        if prev_dirty.min_x < VIEW_WIDTH && prev_dirty.min_y < VIEW_HEIGHT {
            for y in prev_dirty.min_y..=prev_dirty.max_y {
                let row_off = y * VIEW_WIDTH;
                for x in prev_dirty.min_x..=prev_dirty.max_x {
                    framebuf[row_off + x] = bg_color;
                    zbuf[row_off + x] = u16::MAX;
                }
            }
        } else {
            framebuf.fill(bg_color);
            zbuf.fill(u16::MAX);
        }

        let mut offscreen = OffscreenBuffer {
            pixels: framebuf,
            dirty: active_dirty,
        };

        // 3. Render Title & Stats Text directly inside off-screen RAM buffer across full 320x240
        let _ = Text::new("STM32WBA65RI FULL 320x240 DEMO", Point::new(10, 15), title_style).draw(&mut offscreen);

        let mut stats_str = ArrayString::<64>::new();
        let fps = 1000 / last_total_ms.max(1);
        let _ = write!(stats_str, "FPS:{} 3D:{}us DMA-Blit:{}us", fps, last_render_us, last_blit_us);
        let _ = Text::new(stats_str.as_str(), Point::new(10, 225), stats_style).draw(&mut offscreen);

        // 4. Render 3D Physics Meshes into 320x240 Offscreen Buffer
        commands.clear();
        let meshes = [&cube_mesh, &octa_mesh];
        if let Ok(()) = engine.record(meshes.into_iter(), &mut commands, None) {
            let mut frame = FrameCtx {
                zbuffer: zbuf,
                width: VIEW_WIDTH,
                height: VIEW_HEIGHT,
            };
            let _ = engine.execute::<_, 512>(&mut offscreen, &mut frame, &commands, None);
        }

        let mut dirty_region = offscreen.dirty;
        dirty_region.sanitize();
        prev_dirty = dirty_region;

        last_render_us = render_start.elapsed().as_micros();

        // 5. Single-pass Blit of Full 320x240 3D Scene + Text over SPI DMA
        let blit_start = Instant::now();
        let width = dirty_region.max_x - dirty_region.min_x + 1;
        let height = dirty_region.max_y - dirty_region.min_y + 1;

        let dirty_rect_area = Rectangle::new(
            Point::new(dirty_region.min_x as i32, dirty_region.min_y as i32),
            Size::new(width as u32, height as u32),
        );

        let dirty_pixels = (dirty_region.min_y..=dirty_region.max_y).flat_map(|y| {
            let row_off = y * VIEW_WIDTH;
            &framebuf[row_off + dirty_region.min_x..=row_off + dirty_region.max_x]
        });

        let _ = display.fill_contiguous(&dirty_rect_area, dirty_pixels.copied());
        last_blit_us = blit_start.elapsed().as_micros();

        last_total_ms = frame_start.elapsed().as_millis();
        defmt::info!("Frame {}: Total {}ms (3D: {}us, DMA Blit: {}us, FPS: {})", 
            frame_count, last_total_ms, last_render_us, last_blit_us, 1000 / last_total_ms.max(1));

        ticker.next().await;
    }
}
