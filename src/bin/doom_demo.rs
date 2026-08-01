#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::rcc::*;
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
#[path = "../ft6236.rs"]
mod ft6236;
use ft6236::FT6236;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;

use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};

use mipidsi::interface::SpiInterface;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::Builder;

use libm::{cosf, sinf, sqrtf};

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL0 => InterruptHandler<peripherals::GPDMA1_CH0>;
    GPDMA1_CHANNEL1 => InterruptHandler<peripherals::GPDMA1_CH1>;
});

// Zero-overhead CS delay — embassy_time::Delay rounds to ~1ms per tick which
// adds ~38ms waste for 38 tx_buffer flushes. NoDelay relies on SPI peripheral
// hardware timing which is already in the nanosecond range.
struct NoDelay;
impl embedded_hal::delay::DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

// 32KB static TX buffer — keeps SPI DMA chunking to 5 transactions instead
// of 38 (which caused ~38ms of embassy_time overhead per frame).
struct TxBuf([u8; 32768]);
struct SafeTxBuf(UnsafeCell<TxBuf>);
unsafe impl Sync for SafeTxBuf {}
static RAW_TX_BUF: SafeTxBuf = SafeTxBuf(UnsafeCell::new(TxBuf([0u8; 32768])));

const VIEW_WIDTH: usize = 240;
const VIEW_HEIGHT: usize = 320;
const VIEW3D_HEIGHT: usize = 256;
const VIEW_PIXELS: usize = VIEW_WIDTH * VIEW_HEIGHT;

struct FrameBuffer([Rgb565; VIEW_PIXELS]);
struct SafeFrameBuf(UnsafeCell<FrameBuffer>);
unsafe impl Sync for SafeFrameBuf {}
static RAW_FRAMEBUF_A: SafeFrameBuf = SafeFrameBuf(UnsafeCell::new(FrameBuffer([Rgb565::BLACK; VIEW_PIXELS])));
static RAW_FRAMEBUF_B: SafeFrameBuf = SafeFrameBuf(UnsafeCell::new(FrameBuffer([Rgb565::BLACK; VIEW_PIXELS])));
// ----------------------------------------------------------------------------
// DOOM E1M1-Inspired 16x16 Level Map
// ----------------------------------------------------------------------------
const MAP_SIZE: usize = 16;
static MAP: [u8; MAP_SIZE * MAP_SIZE] = [
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 6, 0, 6, 0, 3, 0, 2, 2, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 2, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 6, 0, 6, 0, 3, 0, 2, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 1,
    1, 0, 6, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 5, 0, 3, 1,
    1, 0, 6, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 1,
    1, 0, 0, 0, 0, 0, 6, 0, 0, 0, 3, 0, 6, 0, 3, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 1,
    1, 0, 2, 2, 0, 0, 6, 0, 0, 0, 3, 0, 6, 0, 3, 1,
    1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 3, 1,
    1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3, 0, 5, 0, 3, 1,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];

// Waypoints for smooth level walkthrough patrol camera
struct Waypoint {
    x: f32,
    y: f32,
}

static PATROL_PATH: [Waypoint; 8] = [
    Waypoint { x: 2.5, y: 2.5 },
    Waypoint { x: 2.5, y: 10.5 },
    Waypoint { x: 7.5, y: 10.5 },
    Waypoint { x: 7.5, y: 3.5 },
    Waypoint { x: 12.5, y: 3.5 },
    Waypoint { x: 12.5, y: 12.5 },
    Waypoint { x: 7.5, y: 12.5 },
    Waypoint { x: 2.5, y: 12.5 },
];

// 3D World Sprites (Barrels, Health Kits, Imp Enemies, Ammo Crates)
#[derive(Clone, Copy)]
struct Sprite {
    x: f32,
    y: f32,
    kind: u8,       // 1 = Barrel, 2 = Health Kit, 3 = Imp Enemy, 4 = Ammo Crate
    active: bool,
    hp: i8,          // Enemy HP (Imps start with 2 HP)
    cooldown: u8,    // Attack cooldown timer
    hit_flash: u8,   // Flash white on hit
}

static INITIAL_SPRITES: [Sprite; 9] = [
    Sprite { x: 3.5, y: 3.5, kind: 1, active: true, hp: 1, cooldown: 0, hit_flash: 0 },   // Barrel
    Sprite { x: 7.5, y: 4.0, kind: 2, active: true, hp: 1, cooldown: 0, hit_flash: 0 },   // Health Pack (+25 HP)
    Sprite { x: 2.5, y: 13.5, kind: 3, active: true, hp: 2, cooldown: 20, hit_flash: 0 }, // Imp 1 (Far South Corridor)
    Sprite { x: 13.5, y: 12.5, kind: 3, active: true, hp: 2, cooldown: 30, hit_flash: 0 }, // Imp 2 (Far Southeast Room)
    Sprite { x: 13.5, y: 3.5, kind: 3, active: true, hp: 2, cooldown: 40, hit_flash: 0 },  // Imp 3 (Far Northeast Room)
    Sprite { x: 13.5, y: 10.5, kind: 1, active: true, hp: 1, cooldown: 0, hit_flash: 0 }, // Barrel
    Sprite { x: 8.5, y: 12.5, kind: 2, active: true, hp: 1, cooldown: 0, hit_flash: 0 },  // Health Pack (+25 HP)
    Sprite { x: 5.5, y: 10.5, kind: 4, active: true, hp: 1, cooldown: 0, hit_flash: 0 },  // Ammo Crate (+20 Ammo)
    Sprite { x: 10.5, y: 3.5, kind: 4, active: true, hp: 1, cooldown: 0, hit_flash: 0 },  // Ammo Crate (+20 Ammo)
];

// 3D World Fireball Projectile
#[derive(Clone, Copy)]
struct Fireball {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    active: bool,
}



// ----------------------------------------------------------------------------
// Color Utilities & Shading
// ----------------------------------------------------------------------------
#[inline(always)]
fn apply_shade(color: Rgb565, factor: f32) -> Rgb565 {
    let f = factor.clamp(0.05, 1.0);
    let r = ((color.r() as f32) * f) as u8;
    let g = ((color.g() as f32) * f) as u8;
    let b = ((color.b() as f32) * f) as u8;
    Rgb565::new(r, g, b)
}

#[inline(always)]
fn pack_rgb565_u32(color: Rgb565) -> u32 {
    let raw = color.into_storage() as u32;
    (raw << 16) | raw
}

// ----------------------------------------------------------------------------
// Authentic 1993 DOOM Wall Texture Generator (16x16 Bitmapped Patterns)
// ----------------------------------------------------------------------------
#[inline(always)]
fn get_wall_texture_pixel(tile: u8, tex_x: usize, tex_y: usize, side: u8) -> Rgb565 {
    let base = match tile {
        1 => { // Earthy Tan/Brown Brick Wall (Zero Purple Tint)
            let is_mortar = (tex_y % 4 == 0)
                || ((tex_y / 4) % 2 == 0 && tex_x % 8 == 0)
                || ((tex_y / 4) % 2 == 1 && (tex_x + 4) % 8 == 0);
            if is_mortar { Rgb565::new(6, 12, 6) } else { Rgb565::new(18, 14, 2) }
        }
        2 => { // Tech Blue Panel
            if tex_y == 2 || tex_y == 14 || (tex_x == 8 && tex_y > 4 && tex_y < 12) {
                Rgb565::new(0, 50, 25) // Glowing Cyan/Blue LED
            } else if tex_x == 0 || tex_x == 15 || tex_y == 0 || tex_y == 15 {
                Rgb565::new(2, 8, 12) // Dark Navy Frame
            } else {
                Rgb565::new(4, 16, 22) // Cool Tech Blue Plate
            }
        }
        3 => { // Hazard Warning Stripe
            if (tex_x + tex_y) % 6 < 3 {
                Rgb565::YELLOW
            } else {
                Rgb565::new(2, 4, 2) // Deep Black
            }
        }
        _ => { // Reinforced Slate Steel Blast Door
            if tex_x == 0 || tex_x == 15 || tex_y == 0 || tex_y == 15 {
                Rgb565::new(16, 32, 16) // Outer Steel Frame
            } else if tex_x >= 12 && tex_x <= 13 && tex_y >= 7 && tex_y <= 9 {
                Rgb565::YELLOW // Door Handle
            } else {
                Rgb565::new(10, 20, 10) // Dark Slate Grey Panel
            }
        }
    };
    if side == 1 {
        // Dim EW walls slightly for fake directional lighting
        apply_shade(base, 0.7)
    } else {
        base
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 1. High-Performance MCU Clock Setup (100 MHz SYSCLK + 100 MHz APB1 for 50 MHz SPI)
    let mut config = Config::default();
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul25,
        divp: Some(PllDiv::Div25),
        divq: None,
        divr: Some(PllDiv::Div4), // 100 MHz
        frac: Some(0),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div1;
    config.rcc.apb2_pre = APBPrescaler::Div1;
    config.rcc.voltage_scale = VoltageScale::Range1;
    let p = embassy_stm32::init(config);

    defmt::info!("============================================");
    defmt::info!("DOOM E1M1-Inspired 3D Level Walkthrough Demo");
    defmt::info!("Engine: DDA Fast Raycaster + 25MHz SPI DMA + DOOM HUD");
    defmt::info!("============================================");

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(25_000_000); // Rock-solid 25 MHz SPI bus

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

    let tx_buf = unsafe { &mut (*RAW_TX_BUF.0.get()).0 };
    let spi_device = ExclusiveDevice::new(spi, cs, NoDelay).unwrap();
    let di = SpiInterface::new(spi_device, dc, tx_buf.as_mut_slice());

    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .orientation(Orientation::new().rotate(Rotation::Deg0).flip_horizontal())
        .init(&mut embassy_time::Delay)
        .unwrap();

    let btn1 = Input::new(p.PC13, Pull::Up); // USER1 Button B1 (Leftmost): Turn Left (PC13)
    let btn2 = Input::new(p.PC5, Pull::Up);  // USER2 Button B2 (Center): Move Forward (PC5)
    let btn3 = Input::new(p.PB4, Pull::Up);  // USER3 Button B3 (Rightmost): Turn Right (PB4)

    // Touch Controller I2C1 Setup (SDA=PB1, SCL=PB2, INT=PE0) using FT6236 driver (400kHz Fast Mode)
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(400_000); // 400kHz Fast Mode I2C
    let mut i2c = I2c::new_blocking(
        p.I2C1,
        p.PB2, // SCL (Arduino D15)
        p.PB1, // SDA (Arduino D14)
        i2c_config,
    );
    let touch_int = Input::new(p.PE0, Pull::Up); // T_IRQ (Arduino D2)
    let mut touch_dev = FT6236::new(&mut i2c);

    let mut use_buf_a = true;


    let hud_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);
    let hud_val_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);

    // Initial Camera State
    let mut pos_x: f32 = PATROL_PATH[0].x;
    let mut pos_y: f32 = PATROL_PATH[0].y;
    let mut angle: f32 = 0.0;
    let mut target_wpt_idx: usize = 1;

    let mut manual_mode_timer: u32 = 0;

    let mut z_buffer = [0.0f32; VIEW_WIDTH];

    let mut frame_count: u32 = 0;
    let mut head_bob_time: f32 = 0.0;
    let mut last_raycast_us: u64;
    let mut last_blit_us: u64;
    let mut last_total_ms: u64;

    let mut last_touch_x: u16 = 0;
    let mut last_touch_y: u16 = 0;
    let mut touch_hold_counter: u8 = 0;

    let mut ammo_count: u16 = 50;
    let mut health_count: u16 = 80;
    let mut muzzle_flash_counter: u8 = 0;
    let mut pickup_flash_counter: u8 = 0;
    let mut damage_flash_counter: u8 = 0;

    let mut sprites = INITIAL_SPRITES;
    let mut fireballs = [Fireball { x: 0.0, y: 0.0, vx: 0.0, vy: 0.0, active: false }; 4];
    let mut respawn_timer: u16 = 0;

    let mut ticker = Ticker::every(Duration::from_millis(16)); // Target 60 FPS

    loop {
        let frame_start = Instant::now();

        let framebuf = unsafe {
            if use_buf_a {
                &mut (*RAW_FRAMEBUF_A.0.get()).0
            } else {
                &mut (*RAW_FRAMEBUF_B.0.get()).0
            }
        };
        use_buf_a = !use_buf_a;

        // --------------------------------------------------------------------
        // 1. Physical 3-Button FPS Controls (B1: Left, B2: Forward, B3: Right)
        // --------------------------------------------------------------------
        let dir_x = cosf(angle);
        let dir_y = sinf(angle);

        // Camera plane for 66-degree FOV
        let fov_scale = 0.66f32;
        let plane_x = -dir_y * fov_scale;
        let plane_y = dir_x * fov_scale;

        let mut b1_pressed = btn1.is_low(); // B1 (Leftmost): Turn Right
        let mut b2_pressed = btn2.is_low(); // B2 (Center): Move Forward
        let mut b3_pressed = btn3.is_low(); // B3 (Rightmost): Turn Left
        let mut move_backward = false;
        let mut shoot_pressed = false;

        // FT6236 Touch Controller Polling with Hold Debounce & LiftUp Event Handling
        if touch_int.is_low() || touch_hold_counter > 0 {
            if let Ok(Some(pt)) = touch_dev.get_point0() {
                if pt.event != ft6236::EventType::LiftUp {
                    last_touch_x = pt.x;
                    last_touch_y = pt.y;
                    touch_hold_counter = 4; // Keep hold active for up to 4 frames (60ms window)
                } else {
                    touch_hold_counter = 0; // Immediate release on LiftUp event
                }
            } else if touch_hold_counter > 0 {
                touch_hold_counter -= 1;
            }
        }

        if touch_hold_counter > 0 {
            if last_touch_x < 240 && last_touch_y < 320 {
                if last_touch_x < 80 {
                    b3_pressed = true; // Left third: Turn Left
                } else if last_touch_x > 160 {
                    b1_pressed = true; // Right third: Turn Right
                } else {
                    // Middle third split into vertical THIRDS (0..106: Backward, 106..213: SHOOT, 213..320: Forward)
                    if last_touch_y < 106 {
                        move_backward = true;
                    } else if last_touch_y < 213 {
                        shoot_pressed = true;
                    } else {
                        b2_pressed = true;
                    }
                }
            }
        }

        if shoot_pressed && muzzle_flash_counter == 0 {
            if ammo_count > 0 {
                ammo_count -= 1;
                muzzle_flash_counter = 4; // Flash for 4 frames

                // Player Raycast Shot Hit Detection vs Active Enemies
                for sprite in sprites.iter_mut() {
                    if sprite.active && sprite.kind == 3 {
                        let sprite_x = sprite.x - pos_x;
                        let sprite_y = sprite.y - pos_y;
                        let inv_det = 1.0 / (plane_x * dir_y - dir_x * plane_y);
                        let transform_x = inv_det * (dir_y * sprite_x - dir_x * sprite_y);
                        let transform_y = inv_det * (-plane_y * sprite_x + plane_x * sprite_y);
                        if transform_y > 0.3 {
                            let sprite_screen_x = ((VIEW_WIDTH as f32 / 2.0) * (1.0 - transform_x / transform_y)) as i32;
                            let sprite_width = ((VIEW3D_HEIGHT as f32 / transform_y).abs()) as i32;
                            if (120 - sprite_screen_x).abs() < sprite_width / 2 + 12 {
                                sprite.hp -= 1;
                                sprite.hit_flash = 5; // Flash white on impact
                                if sprite.hp <= 0 {
                                    sprite.active = false; // Defeated Imp
                                }
                            }
                        }
                    }
                }
            }
        } else if muzzle_flash_counter > 0 {
            muzzle_flash_counter -= 1;
        }

        if b1_pressed || b2_pressed || b3_pressed || move_backward || shoot_pressed {
            manual_mode_timer = 300; // Reset 5-second manual control timeout

            if b1_pressed {
                // B1: Rotate Camera Right
                angle += 0.05f32;
                if angle > core::f32::consts::TAU { angle -= core::f32::consts::TAU; }
            }
            if b3_pressed {
                // B3: Rotate Camera Left
                angle -= 0.05f32;
                if angle < 0.0 { angle += core::f32::consts::TAU; }
            }
            if b2_pressed {
                // B2: Move Forward in Camera Direction
                let move_speed = 0.06f32;
                let next_x = pos_x + dir_x * move_speed;
                let next_y = pos_y + dir_y * move_speed;
                if MAP[(next_y as usize) * MAP_SIZE + (pos_x as usize)] == 0 { pos_y = next_y; }
                if MAP[(pos_y as usize) * MAP_SIZE + (next_x as usize)] == 0 { pos_x = next_x; }
                head_bob_time += 0.25;
            }
            if move_backward {
                // Move Backward (Reverse direction)
                let move_speed = 0.06f32;
                let next_x = pos_x - dir_x * move_speed;
                let next_y = pos_y - dir_y * move_speed;
                if MAP[(next_y as usize) * MAP_SIZE + (pos_x as usize)] == 0 { pos_y = next_y; }
                if MAP[(pos_y as usize) * MAP_SIZE + (next_x as usize)] == 0 { pos_x = next_x; }
                head_bob_time -= 0.25;
            }
        } else if manual_mode_timer > 0 {
            manual_mode_timer -= 1;
        }

        // --------------------------------------------------------------------
        // 1.5 Flying 3D Fireballs & Enemy Attack AI
        // --------------------------------------------------------------------
        let is_dead = health_count == 0;

        // Update active flying fireballs
        for fb in fireballs.iter_mut() {
            if fb.active {
                fb.x += fb.vx;
                fb.y += fb.vy;
                let dx = pos_x - fb.x;
                let dy = pos_y - fb.y;
                let dist_sq = dx * dx + dy * dy;

                if dist_sq < 0.64 { // Hits player!
                    health_count = health_count.saturating_sub(18);
                    damage_flash_counter = 8;
                    fb.active = false;
                } else if fb.x < 0.5 || fb.x > 15.5 || fb.y < 0.5 || fb.y > 15.5 || MAP[(fb.y as usize) * MAP_SIZE + (fb.x as usize)] > 0 {
                    fb.active = false; // Hits wall
                }
            }
        }

        // Imp Attack AI (Launches visible flying fireballs)
        for sprite in sprites.iter_mut() {
            if sprite.hit_flash > 0 { sprite.hit_flash -= 1; }

            if sprite.active && sprite.kind == 3 && !is_dead {
                let dx = pos_x - sprite.x;
                let dy = pos_y - sprite.y;
                let dist_sq = dx * dx + dy * dy;
                if dist_sq < 100.0 && dist_sq > 0.4 { // Within line of sight
                    if sprite.cooldown > 0 {
                        sprite.cooldown -= 1;
                    } else {
                        // Spawn flying fireball from Imp to player
                        let dist = sqrtf(dist_sq).max(0.1);
                        let speed = 0.20f32; // Fast flying fireball
                        let dir_u_x = dx / dist;
                        let dir_u_y = dy / dist;
                        for fb in fireballs.iter_mut() {
                            if !fb.active {
                                fb.x = sprite.x + dir_u_x * 0.4;
                                fb.y = sprite.y + dir_u_y * 0.4;
                                fb.vx = dir_u_x * speed;
                                fb.vy = dir_u_y * speed;
                                fb.active = true;
                                break;
                            }
                        }
                        sprite.cooldown = 35; // Fast attack rate (~0.6s between fireballs)
                    }
                }
            }
        }

        // Tap to Respawn when Dead
        if is_dead && (b1_pressed || b2_pressed || b3_pressed || shoot_pressed || move_backward) {
            health_count = 100;
            ammo_count = 50;
            pos_x = PATROL_PATH[0].x;
            pos_y = PATROL_PATH[0].y;
            angle = 0.0;
            sprites = INITIAL_SPRITES;
            for fb in fireballs.iter_mut() { fb.active = false; }
        }

        // Pickups (Health Packs & Ammo Crates)
        let mut any_pickup_active = false;
        let mut any_enemy_active = false;
        for sprite in sprites.iter_mut() {
            if sprite.kind == 3 && sprite.active { any_enemy_active = true; }
            if sprite.kind == 2 || sprite.kind == 4 {
                if sprite.active {
                    any_pickup_active = true;
                    let dx = pos_x - sprite.x;
                    let dy = pos_y - sprite.y;
                    if dx * dx + dy * dy < 0.49 {
                        if sprite.kind == 2 && health_count < 100 {
                            health_count = (health_count + 25).min(100);
                            sprite.active = false;
                            pickup_flash_counter = 6;
                        } else if sprite.kind == 4 && ammo_count < 999 {
                            ammo_count = (ammo_count + 20).min(999);
                            sprite.active = false;
                            pickup_flash_counter = 6;
                        }
                    }
                }
            }
        }

        // Respawn all sprites (enemies + pickups) if all cleared
        if !any_pickup_active || !any_enemy_active {
            respawn_timer += 1;
            if respawn_timer > 500 { // Respawn after ~8 seconds
                for sprite in sprites.iter_mut() {
                    sprite.active = true;
                    if sprite.kind == 3 { sprite.hp = 2; sprite.cooldown = 40; }
                }
                respawn_timer = 0;
            }
        }
        if pickup_flash_counter > 0 { pickup_flash_counter -= 1; }
        if damage_flash_counter > 0 { damage_flash_counter -= 1; }

        let is_manual = manual_mode_timer > 0;
        let mut angle_diff = 0.0f32;

        if !is_manual {
            // Auto-Patrol Mode
            let target = &PATROL_PATH[target_wpt_idx];
            let dx = target.x - pos_x;
            let dy = target.y - pos_y;
            let dist_to_target = sqrtf(dx * dx + dy * dy);

            let target_angle = libm::atan2f(dy, dx);
            angle_diff = target_angle - angle;
            while angle_diff > core::f32::consts::PI { angle_diff -= core::f32::consts::TAU; }
            while angle_diff < -core::f32::consts::PI { angle_diff += core::f32::consts::TAU; }
            angle += angle_diff * 0.08;

            let move_speed = 0.045f32;

            if dist_to_target < 0.8 {
                target_wpt_idx = (target_wpt_idx + 1) % PATROL_PATH.len();
            } else {
                let next_x = pos_x + dir_x * move_speed;
                let next_y = pos_y + dir_y * move_speed;
                if MAP[(next_y as usize) * MAP_SIZE + (pos_x as usize)] == 0 { pos_y = next_y; }
                if MAP[(pos_y as usize) * MAP_SIZE + (next_x as usize)] == 0 { pos_x = next_x; }
            }
            head_bob_time += 0.15;
        }
        let head_bob = (sinf(head_bob_time) * 3.0) as i32;

        let raycast_start = Instant::now();

        // Camera plane for 66-degree FOV
        let fov_scale = 0.66f32;
        let plane_x = -dir_y * fov_scale;
        let plane_y = dir_x * fov_scale;

        // --------------------------------------------------------------------
        // --------------------------------------------------------------------
        // 2. Mode 7 True 3D Perspective Floor & Ceiling Engine (via embedded-3dgfx crate)
        // --------------------------------------------------------------------
        let framebuf_u32 = unsafe {
            core::slice::from_raw_parts_mut(framebuf.as_mut_ptr() as *mut u32, VIEW_PIXELS / 2)
        };

        let mode7_renderer = embedded_3dgfx::raycast::Mode7Renderer::new(VIEW_WIDTH, VIEW3D_HEIGHT);
        mode7_renderer.render_floor_and_ceiling(
            pos_x,
            pos_y,
            angle,
            head_bob,
            Rgb565::new(16, 12, 4), // E1M1 Tan Stone Tile A
            Rgb565::new(11, 8, 3),  // Dark Stone Tile B
            Rgb565::new(6, 12, 18), // Tech Ceiling Panel A
            Rgb565::new(4, 8, 14),  // Tech Ceiling Panel B
            framebuf_u32,
        );

        // --------------------------------------------------------------------
        // 2.5 DDA 3D Wall Column Rendering
        // --------------------------------------------------------------------
        for x in (0..VIEW_WIDTH).step_by(4) {
            // Negate camera_x to compensate for display flip_horizontal() orientation
            let camera_x = -(2.0 * (x as f32) / (VIEW_WIDTH as f32) - 1.0);
            let ray_dir_x = dir_x + plane_x * camera_x;
            let ray_dir_y = dir_y + plane_y * camera_x;

            let mut map_x = pos_x as i32;
            let mut map_y = pos_y as i32;

            let delta_dist_x = if ray_dir_x == 0.0 { 1e30 } else { libm::fabsf(1.0 / ray_dir_x) };
            let delta_dist_y = if ray_dir_y == 0.0 { 1e30 } else { libm::fabsf(1.0 / ray_dir_y) };

            let (step_x, mut side_dist_x) = if ray_dir_x < 0.0 {
                (-1, (pos_x - map_x as f32) * delta_dist_x)
            } else {
                (1, (map_x as f32 + 1.0 - pos_x) * delta_dist_x)
            };

            let (step_y, mut side_dist_y) = if ray_dir_y < 0.0 {
                (-1, (pos_y - map_y as f32) * delta_dist_y)
            } else {
                (1, (map_y as f32 + 1.0 - pos_y) * delta_dist_y)
            };

            let mut hit_wall = 0u8;
            let mut side = 0u8;
            let mut steps = 0;

            while hit_wall == 0 && steps < 24 {
                if side_dist_x < side_dist_y {
                    side_dist_x += delta_dist_x;
                    map_x += step_x;
                    side = 0;
                } else {
                    side_dist_y += delta_dist_y;
                    map_y += step_y;
                    side = 1;
                }

                if map_x >= 0 && map_x < MAP_SIZE as i32 && map_y >= 0 && map_y < MAP_SIZE as i32 {
                    let tile = MAP[(map_y as usize) * MAP_SIZE + (map_x as usize)];
                    if tile > 0 { hit_wall = tile; }
                } else {
                    hit_wall = 1;
                }
                steps += 1;
            }

            let perp_wall_dist = if side == 0 {
                side_dist_x - delta_dist_x
            } else {
                side_dist_y - delta_dist_y
            }.max(0.1);

            // Fill all 4 z-buffer slots for sprite occlusion
            z_buffer[x]     = perp_wall_dist;
            z_buffer[x + 1] = perp_wall_dist;
            z_buffer[x + 2] = perp_wall_dist;
            z_buffer[x + 3] = perp_wall_dist;

            let line_height = (VIEW3D_HEIGHT as f32 / perp_wall_dist) as i32;
            let center_y = (VIEW3D_HEIGHT / 2) as i32 + head_bob;

            let draw_start = (center_y - line_height / 2).clamp(0, VIEW3D_HEIGHT as i32 - 1) as usize;
            let draw_end   = (center_y + line_height / 2).clamp(0, VIEW3D_HEIGHT as i32 - 1) as usize;

            // Calculate exact fractional wall hit position for texture mapping
            let mut wall_x = if side == 0 {
                pos_y + perp_wall_dist * ray_dir_y
            } else {
                pos_x + perp_wall_dist * ray_dir_x
            };
            wall_x -= libm::floorf(wall_x);
            let tex_x = ((wall_x * 16.0) as usize).clamp(0, 15);

            let shade_factor = 1.0 / (1.0 + perp_wall_dist * 0.18);
            let x_u32 = x / 2;

            // Textured 3D Wall Column
            let tex_step = 16.0 / (line_height as f32).max(1.0);
            let mut tex_pos = ((draw_start as i32 - center_y + line_height / 2) as f32) * tex_step;

            for y in draw_start..=draw_end {
                let tex_y = (tex_pos as usize) & 15;
                tex_pos += tex_step;

                let tex_color = get_wall_texture_pixel(hit_wall, tex_x, tex_y, side);
                let shaded_pixel = apply_shade(tex_color, shade_factor);
                let pixel_u32 = pack_rgb565_u32(shaded_pixel);

                let idx = y * 120 + x_u32;
                framebuf_u32[idx]     = pixel_u32;
                framebuf_u32[idx + 1] = pixel_u32;
            }
        }

        // --------------------------------------------------------------------
        // 3. Render Billboarded 3D Sprites & Flying Fireballs
        // --------------------------------------------------------------------
        for sprite in sprites.iter() {
            if !sprite.active { continue; }

            let sprite_x = sprite.x - pos_x;
            let sprite_y = sprite.y - pos_y;

            let inv_det = 1.0 / (plane_x * dir_y - dir_x * plane_y);
            let transform_x = inv_det * (dir_y * sprite_x - dir_x * sprite_y);
            let transform_y = inv_det * (-plane_y * sprite_x + plane_x * sprite_y);

            if transform_y > 0.3 {
                let sprite_screen_x = ((VIEW_WIDTH as f32 / 2.0) * (1.0 - transform_x / transform_y)) as i32;
                let sprite_height = ((VIEW3D_HEIGHT as f32 / transform_y).abs()) as i32;
                let sprite_width = sprite_height;

                let center_y = (VIEW3D_HEIGHT / 2) as i32 + head_bob;
                let draw_start_y = (center_y - sprite_height / 2).clamp(0, VIEW3D_HEIGHT as i32 - 1) as usize;
                let draw_end_y = (center_y + sprite_height / 2).clamp(0, VIEW3D_HEIGHT as i32 - 1) as usize;

                let draw_start_x = (sprite_screen_x - sprite_width / 2).clamp(0, VIEW_WIDTH as i32 - 1) as usize;
                let draw_end_x = (sprite_screen_x + sprite_width / 2).clamp(0, VIEW_WIDTH as i32 - 1) as usize;

                let base_color = if sprite.hit_flash > 0 {
                    Rgb565::WHITE // White impact flash when shot!
                } else {
                    match sprite.kind {
                        1 => Rgb565::new(16, 40, 8),           // Toxic Green Barrel
                        2 => Rgb565::GREEN,                    // Health Pack (+25 HP)
                        3 => Rgb565::new(18, 10, 4),           // Classic DOOM Ochre-Brown Imp
                        _ => Rgb565::new(0, 36, 31),           // Steel Blue Ammo Crate
                    }
                };
                let shaded_sprite = apply_shade(base_color, 1.0 / (1.0 + transform_y * 0.2));

                for stripe_x in draw_start_x..draw_end_x {
                    if transform_y < z_buffer[stripe_x] {
                        for y in draw_start_y..draw_end_y {
                            let row = y * VIEW_WIDTH;
                            // Imp red eye detail near the top of the sprite head
                            let pixel_color = if sprite.kind == 3 && sprite.hit_flash == 0 && y >= draw_start_y + (draw_end_y - draw_start_y) / 6 && y <= draw_start_y + (draw_end_y - draw_start_y) / 4 {
                                Rgb565::RED
                            } else {
                                shaded_sprite
                            };
                            if (stripe_x + y) % 2 == 0 {
                                framebuf[row + stripe_x] = pixel_color;
                            }
                        }
                    }
                }
            }
        }

        // Render Flying 3D Fireballs (Glowing Orange/Yellow Orbs)
        for fb in fireballs.iter() {
            if !fb.active { continue; }
            let fb_x = fb.x - pos_x;
            let fb_y = fb.y - pos_y;

            let inv_det = 1.0 / (plane_x * dir_y - dir_x * plane_y);
            let transform_x = inv_det * (dir_y * fb_x - dir_x * fb_y);
            let transform_y = inv_det * (-plane_y * fb_x + plane_x * fb_y);

            if transform_y > 0.3 {
                let fb_screen_x = ((VIEW_WIDTH as f32 / 2.0) * (1.0 - transform_x / transform_y)) as i32;
                let fb_size = (((VIEW3D_HEIGHT as f32 / transform_y).abs()) * 0.4) as i32;

                let center_y = (VIEW3D_HEIGHT / 2) as i32 + head_bob;
                let draw_start_y = (center_y - fb_size / 2).clamp(0, VIEW3D_HEIGHT as i32 - 1) as usize;
                let draw_end_y = (center_y + fb_size / 2).clamp(0, VIEW3D_HEIGHT as i32 - 1) as usize;

                let draw_start_x = (fb_screen_x - fb_size / 2).clamp(0, VIEW_WIDTH as i32 - 1) as usize;
                let draw_end_x = (fb_screen_x + fb_size / 2).clamp(0, VIEW_WIDTH as i32 - 1) as usize;

                let fb_color = Rgb565::new(31, 32, 0); // Fiery Glowing Orange

                for stripe_x in draw_start_x..draw_end_x {
                    if transform_y < z_buffer[stripe_x] {
                        for y in draw_start_y..draw_end_y {
                            framebuf[y * VIEW_WIDTH + stripe_x] = fb_color;
                        }
                    }
                }
            }
        }

        // --------------------------------------------------------------------
        // 4. Render Crosshair & First-Person Weapon at Bottom Center
        // --------------------------------------------------------------------
        // Center Crosshair (+)
        let cx = 120usize;
        let cy = 128usize;
        let crosshair_color = Rgb565::GREEN; // Pure Lime Green (Red=0, Green=63, Blue=0)
        for offset in 1..=4 {
            framebuf[cy * VIEW_WIDTH + cx - offset] = crosshair_color;
            framebuf[cy * VIEW_WIDTH + cx + offset] = crosshair_color;
            framebuf[(cy - offset) * VIEW_WIDTH + cx] = crosshair_color;
            framebuf[(cy + offset) * VIEW_WIDTH + cx] = crosshair_color;
        }

        // Shotgun Barrel & Recoil
        let weapon_recoil = if muzzle_flash_counter > 0 { 8i32 } else { 0i32 };
        let weapon_bob = (sinf(head_bob_time * 2.0) * 4.0) as i32;
        let weapon_x_center = 120 + (cosf(head_bob_time) * 3.0) as i32;
        let weapon_y_start = (215 + weapon_bob + weapon_recoil).clamp(180, 248) as usize;

        let gun_metal = Rgb565::new(12, 14, 16);
        let gun_dark  = Rgb565::new(5, 5, 5);

        for wy in weapon_y_start..VIEW3D_HEIGHT {
            let row = wy * VIEW_WIDTH;
            let width_at_y = 12 + ((wy - weapon_y_start) / 2) as i32;
            let wx_start = (weapon_x_center - width_at_y).clamp(0, 239) as usize;
            let wx_end   = (weapon_x_center + width_at_y).clamp(0, 239) as usize;

            for wx in wx_start..wx_end {
                framebuf[row + wx] = if wx % 3 == 0 { gun_dark } else { gun_metal };
            }
        }

        // Explosive Muzzle Flash Flare when firing!
        if muzzle_flash_counter > 0 {
            let flash_center_x = weapon_x_center as usize;
            let flash_center_y = (weapon_y_start - 12).clamp(150, 220);
            let flash_yellow = Rgb565::YELLOW;
            let flash_orange = Rgb565::new(31, 32, 0);

            for dy in 0..16usize {
                let fy = flash_center_y + dy;
                if fy < VIEW3D_HEIGHT {
                    let row = fy * VIEW_WIDTH;
                    let radius = (8 - (dy as i32 - 8).abs()) as usize;
                    let fx_start = flash_center_x.saturating_sub(radius);
                    let fx_end   = (flash_center_x + radius).min(239);
                    framebuf[row + fx_start..=row + fx_end].fill(if dy % 2 == 0 { flash_yellow } else { flash_orange });
                }
            }
        }

        // --------------------------------------------------------------------
        // 4.5 Damage Flash Animation: Semi-Transparent Red Viewport Border (Not HUD)
        // --------------------------------------------------------------------
        if damage_flash_counter > 0 {
            let border_width = 12usize;
            let red_flash = Rgb565::RED;

            for y in 0..VIEW3D_HEIGHT {
                let row = y * VIEW_WIDTH;
                let is_top_bottom_border = y < border_width || y >= VIEW3D_HEIGHT - border_width;
                for x in 0..VIEW_WIDTH {
                    let is_left_right_border = x < border_width || x >= VIEW_WIDTH - border_width;
                    if is_top_bottom_border || is_left_right_border {
                        // Semi-transparent checkerboard mesh overlay over 3D viewport edges
                        if (x + y) % 2 == 0 {
                            framebuf[row + x] = red_flash;
                        }
                    }
                }
            }
        }

        // --------------------------------------------------------------------
        // 5. Render Classic DOOM HUD Status Bar (y = 256..320, full 240px width)
        // Portrait layout: [AMMO | HEALTH | FACE | MODE | MAP]
        // --------------------------------------------------------------------
        let hud_bg = Rgb565::new(4, 6, 8);
        let hud_border = Rgb565::RED;
        const HUD_Y: usize = VIEW3D_HEIGHT; // 256

        // Fill HUD background area using 32-bit word fills
        let hud_bg_u32 = pack_rgb565_u32(hud_bg);
        let framebuf_u32 = unsafe {
            core::slice::from_raw_parts_mut(framebuf.as_mut_ptr() as *mut u32, VIEW_PIXELS / 2)
        };
        for y in HUD_Y..VIEW_HEIGHT {
            let row_u32 = y * 120;
            framebuf_u32[row_u32..row_u32 + 120].fill(hud_bg_u32);
        }
        // Top HUD border line
        framebuf[HUD_Y * VIEW_WIDTH..(HUD_Y + 1) * VIEW_WIDTH].fill(hud_border);

        // A. AMMO (x=3) & HEALTH (x=52) — left side
        {
            let mut hud_offscreen = OffscreenBuffer {
                pixels: framebuf,
                dirty: DirtyRect::empty(),
            };
            let ty = (HUD_Y + 6) as i32;
            let vy = (HUD_Y + 20) as i32;

            let mut ammo_buf = [b' '; 4];
            let mut num = ammo_count;
            ammo_buf[3] = b'0' + (num % 10) as u8; num /= 10;
            ammo_buf[2] = b'0' + (num % 10) as u8; num /= 10;
            ammo_buf[1] = b'0' + (num % 10) as u8;
            let ammo_str = core::str::from_utf8(&ammo_buf).unwrap_or(" 050");

            let mut health_buf = [b' '; 5];
            let mut h_num = health_count;
            health_buf[4] = b'%';
            health_buf[3] = b'0' + (h_num % 10) as u8; h_num /= 10;
            health_buf[2] = b'0' + (h_num % 10) as u8; h_num /= 10;
            health_buf[1] = b'0' + (h_num % 10) as u8;
            let health_str = core::str::from_utf8(&health_buf).unwrap_or(" 100%");

            let val_style = if pickup_flash_counter > 0 {
                MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE)
            } else {
                hud_val_style
            };

            let _ = Text::with_baseline("AMMO",     Point::new(3,  ty), hud_text_style, Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline(ammo_str,   Point::new(3,  vy), val_style,      Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline("HEALTH",   Point::new(52, ty), hud_text_style, Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline(health_str, Point::new(52, vy), val_style,      Baseline::Top).draw(&mut hud_offscreen);
        }

        // B. DOOM Guy Face — centered at x=106 (face 28px wide)
        let face_box_x = 106usize;
        let face_box_y = HUD_Y + 4;
        let face_bg = Rgb565::new(18, 14, 12);
        for fy in face_box_y..face_box_y + 54 {
            let row = fy * VIEW_WIDTH;
            framebuf[row + face_box_x..row + face_box_x + 28].fill(face_bg);
        }
        let eye_offset = if angle_diff > 0.05 { 2i32 } else if angle_diff < -0.05 { -2 } else { 0 };
        let skin_color  = Rgb565::new(28, 20, 16);
        let eye_color   = if is_dead || damage_flash_counter > 0 || muzzle_flash_counter > 0 { Rgb565::RED } else { Rgb565::WHITE };
        let pupil_color = Rgb565::BLACK;
        for fy in (face_box_y + 8)..(face_box_y + 44) {
            let row = fy * VIEW_WIDTH;
            framebuf[row + face_box_x + 4..row + face_box_x + 24].fill(skin_color);
        }
        let eye_y   = face_box_y + 18;
        let eye_row = eye_y * VIEW_WIDTH;
        framebuf[eye_row + face_box_x + 6..eye_row + face_box_x + 10].fill(eye_color);
        framebuf[eye_row + face_box_x + 16..eye_row + face_box_x + 20].fill(eye_color);
        let px1 = (face_box_x as i32 + 7  + eye_offset) as usize;
        let px2 = (face_box_x as i32 + 17 + eye_offset) as usize;
        framebuf[eye_row + px1] = pupil_color;
        framebuf[eye_row + px2] = pupil_color;

        // Grinning teeth when shooting / Ouch expression / Dead eyes
        let mouth_row = (face_box_y + 34) * VIEW_WIDTH;
        if is_dead {
            framebuf[mouth_row + face_box_x + 4..mouth_row + face_box_x + 24].fill(Rgb565::BLACK);
        } else if muzzle_flash_counter > 0 {
            framebuf[mouth_row + face_box_x + 8..mouth_row + face_box_x + 20].fill(Rgb565::WHITE);
        } else if damage_flash_counter > 0 {
            framebuf[mouth_row + face_box_x + 6..mouth_row + face_box_x + 22].fill(Rgb565::new(31, 10, 0));
        } else {
            framebuf[mouth_row + face_box_x + 8..mouth_row + face_box_x + 20].fill(Rgb565::RED);
        }

        // YOU DIED Screen Overlay
        if is_dead {
            // Blood Red Mesh Screen Tint
            for y in 0..VIEW3D_HEIGHT {
                let row = y * VIEW_WIDTH;
                for x in 0..VIEW_WIDTH {
                    if (x + y) % 2 == 0 {
                        framebuf[row + x] = Rgb565::RED;
                    }
                }
            }

            let mut hud_offscreen = OffscreenBuffer {
                pixels: framebuf,
                dirty: DirtyRect::empty(),
            };
            let death_title_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
            let death_sub_style   = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);

            let _ = Text::with_baseline("================================", Point::new(24,  90), death_title_style, Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline("YOU DIED!",                    Point::new(93, 105), death_title_style, Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline("TAP TO RESPAWN",                Point::new(78, 122), death_sub_style,   Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline("================================", Point::new(24, 137), death_title_style, Baseline::Top).draw(&mut hud_offscreen);
        }

        // C. MODE label — right of face (x=140)
        {
            let mut hud_offscreen = OffscreenBuffer {
                pixels: framebuf,
                dirty: DirtyRect::empty(),
            };
            let mode_str = if is_manual { "MANUAL" } else { " AUTO " };
            let ty = (HUD_Y + 6) as i32;
            let vy = (HUD_Y + 20) as i32;
            let _ = Text::with_baseline("MODE",   Point::new(140, ty), hud_text_style, Baseline::Top).draw(&mut hud_offscreen);
            let _ = Text::with_baseline(mode_str, Point::new(140, vy), hud_val_style,  Baseline::Top).draw(&mut hud_offscreen);
        }

        // D. Minimap — right side of HUD (x=186..234, y=HUD_Y+4..HUD_Y+52)
        const MINI_SCALE: usize = 3;
        let mini_x = 186usize;
        let mini_y = HUD_Y + 4;

        // Black background for minimap area
        for ry in mini_y..mini_y + 16 * MINI_SCALE {
            let row = ry * VIEW_WIDTH;
            framebuf[row + mini_x..row + mini_x + 16 * MINI_SCALE].fill(Rgb565::BLACK);
        }
        // Draw wall tiles — x mirrored to compensate for flip_horizontal() display
        for my in 0..16usize {
            for mx in 0..16usize {
                let tile = MAP[my * 16 + mx];
                if tile > 0 {
                    let tile_color = match tile {
                        1 => Rgb565::new(14, 28, 14),
                        2 => Rgb565::new(20, 4, 4),
                        3 => Rgb565::new(3, 12, 22),
                        _ => Rgb565::new(16, 16, 8),
                    };
                    // Reverse mx so that after hardware flip the map reads left→right
                    let px = mini_x + (MAP_SIZE - 1 - mx) * MINI_SCALE;
                    let py = mini_y + my * MINI_SCALE;
                    for dy in 0..MINI_SCALE {
                        let row = (py + dy) * VIEW_WIDTH;
                        framebuf[row + px..row + px + MINI_SCALE].fill(tile_color);
                    }
                }
            }
        }
        // Player dot (2×2) — mirror pos_x the same way
        let player_px = (mini_x + ((MAP_SIZE as f32 - pos_x) * MINI_SCALE as f32) as usize)
            .clamp(mini_x, mini_x + 16 * MINI_SCALE - 2);
        let player_py = (mini_y + (pos_y * MINI_SCALE as f32) as usize)
            .clamp(mini_y, mini_y + 16 * MINI_SCALE - 2);
        let pr = player_py * VIEW_WIDTH + player_px;
        framebuf[pr]     = Rgb565::RED;
        framebuf[pr + 1] = Rgb565::RED;
        let pr2 = (player_py + 1) * VIEW_WIDTH + player_px;
        framebuf[pr2]     = Rgb565::RED;
        framebuf[pr2 + 1] = Rgb565::RED;
        // Direction arrow — negate dir_x contribution to match the mirrored x axis
        let dir_px = (player_px as i32 - (dir_x * 4.0) as i32)
            .clamp(mini_x as i32, (mini_x + 16 * MINI_SCALE - 1) as i32) as usize;
        let dir_py = (player_py as i32 + (dir_y * 4.0) as i32)
            .clamp(mini_y as i32, (mini_y + 16 * MINI_SCALE - 1) as i32) as usize;
        framebuf[dir_py * VIEW_WIDTH + dir_px] = Rgb565::YELLOW;

        last_raycast_us = raycast_start.elapsed().as_micros();

        // --------------------------------------------------------------------
        // 6. Full-Screen SPI DMA Blit (NoDelay + 32KB tx_buffer = ~5 transactions)
        // --------------------------------------------------------------------
        let blit_start = Instant::now();

        let full_area = Rectangle::new(Point::new(0, 0), Size::new(VIEW_WIDTH as u32, VIEW_HEIGHT as u32));
        let _ = display.fill_contiguous(&full_area, framebuf.iter().copied());

        last_blit_us = blit_start.elapsed().as_micros();

        last_total_ms = frame_start.elapsed().as_millis();
        frame_count += 1;

        if frame_count % 60 == 0 {
            defmt::info!("DOOM Demo Frame {}: Total {}ms (Raycast: {}us, DMA Blit: {}us, FPS: {})",
                frame_count, last_total_ms, last_raycast_us, last_blit_us, 1000 / last_total_ms.max(1));
        }

        ticker.next().await;
    }
}

// ----------------------------------------------------------------------------
// Offscreen Buffer Helper for embedded-graphics Text Rendering into SRAM
// ----------------------------------------------------------------------------
struct OffscreenBuffer<'a> {
    pixels: &'a mut [Rgb565; VIEW_PIXELS],
    dirty: DirtyRect,
}

#[derive(Clone, Copy)]
struct DirtyRect {
    min_x: usize,
    min_y: usize,
    max_x: usize,
    max_y: usize,
}

impl DirtyRect {
    fn empty() -> Self {
        Self { min_x: VIEW_WIDTH, min_y: VIEW_HEIGHT, max_x: 0, max_y: 0 }
    }
    fn add_point(&mut self, x: usize, y: usize) {
        if x < self.min_x { self.min_x = x; }
        if y < self.min_y { self.min_y = y; }
        if x > self.max_x { self.max_x = x; }
        if y > self.max_y { self.max_y = y; }
    }
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
}
