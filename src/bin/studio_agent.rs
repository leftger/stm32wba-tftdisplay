//! Studio live-display agent.
//!
//! Flash this once; it enumerates as a vendor-specific USB bulk device on the WBA65 USB-HS port
//! (PD6/PD7) and blits RGB565 rectangles pushed by `embedded-gui-studio` straight
//! to the ILI9341 panel. No reflash is needed as the host GUI changes.
//!
//! Protocol: [`embedded_gui_live`]. The host renders an `embedded-gui` screen to
//! RGB565, diffs it, and streams changed tiles; this agent decodes them with a
//! constant-memory [`Decoder`] and paints them. The FT6236 capacitive panel is
//! sampled between USB reads and reported back as [`Msg::Touch`], so Studio's
//! Live Interactive mode can react to on-glass touches.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master as I2cMaster};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::rcc::*;
use embassy_stm32::spi::{mode::Master, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::msos::{
    CompatibleIdFeatureDescriptor, PropertyData, RegistryPropertyFeatureDescriptor,
};
use embassy_usb::{Builder, UsbDevice};
use embedded_gui_live::{Decoder, Msg, PROTO_VERSION};
use static_cell::StaticCell;

#[path = "../ft6236.rs"]
mod ft6236;
use ft6236::{EventType, FT6236};

bind_interrupts!(struct Irqs {
    USB_OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
    GPDMA1_CHANNEL0 => InterruptHandler<peripherals::GPDMA1_CH0>;
});

const PANEL_W: u16 = 320;
const PANEL_H: u16 = 240;
/// Native high-speed bulk transport, matching Markham's proven WBA65 design.
const USB_MPS: u16 = 512;

/// Decoder capacity: `FRAME_RECT_HEADER` (12) + a 40x40 RGB565 tile (3200) with
/// headroom. Studio streams 40x40 tiles by default.
const DEC_CAP: usize = 4096;
const MAX_RECT_BYTES: usize = 40 * 40 * 2;

/// Owned rectangle handed from the USB decoder to the display task. Two queue
/// slots allow USB to receive the next tile while GPDMA transmits the current
/// one. Backpressure naturally NAKs USB if SPI falls more than two tiles behind.
struct RectJob {
    x: u16,
    y: u16,
    w: u16,
    h: u16,
    len: usize,
    pixels_be: [u8; MAX_RECT_BYTES],
}

static DISPLAY_QUEUE: Channel<CriticalSectionRawMutex, RectJob, 2> = Channel::new();
static CLEAR_CHUNK: [u8; MAX_RECT_BYTES] = [0; MAX_RECT_BYTES];

/// How often the FT6236 is polled for a new sample while the USB link is idle.
const TOUCH_POLL: Duration = Duration::from_millis(15);
/// Minimum panel-space movement (px) that forces a fresh `Touch` while held, so
/// a stationary finger doesn't flood the IN endpoint but drags stay smooth.
const TOUCH_MOVE_EPS: u16 = 3;

type UsbDriver = Driver<'static, peripherals::USB_OTG_HS>;
type DisplaySpi = Spi<'static, Async, Master>;
type TouchI2c = I2c<'static, Blocking, I2cMaster>;

/// Maps a raw FT6236 sample into panel framebuffer coordinates.
///
/// The panel is initialized landscape (MADCTL `0xE8`: 320x240), while the
/// FT6236 reports in its native portrait frame (short axis ~240, long axis
/// ~320). This rotates the portrait sample 90° into the landscape framebuffer.
/// The two flips below are the axis conventions most likely correct for this
/// glass; if a bring-up shows touch mirrored, flip the corresponding line.
fn map_touch(raw_x: u16, raw_y: u16) -> (u16, u16) {
    let px = raw_y.min(PANEL_W - 1);
    let py = (PANEL_H - 1).saturating_sub(raw_x.min(PANEL_H - 1));
    (px, py)
}

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) {
    device.run().await;
}

#[embassy_executor::task]
async fn display_task(
    mut spi: DisplaySpi,
    mut cs: Output<'static>,
    mut dc: Output<'static>,
    mut rst: Output<'static>,
) {
    init_display(&mut spi, &mut cs, &mut dc, &mut rst).await;
    clear_display(&mut spi, &mut cs, &mut dc).await;
    defmt::info!("display DMA task ready");

    loop {
        let job = DISPLAY_QUEUE.receive().await;
        paint_rect_dma(&mut spi, &mut cs, &mut dc, &job).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Same clock tree the USB-host demo uses; this powers the HS PHY.
    let mut config = Config::default();
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul30,
        divr: Some(PllDiv::Div5),  // 96 MHz sysclk
        divq: Some(PllDiv::Div10), // 48 MHz
        divp: Some(PllDiv::Div30), // 16 MHz USB_OTG_HS reference
        frac: Some(0),
    });
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div1;
    config.rcc.apb2_pre = APBPrescaler::Div1;
    config.rcc.voltage_scale = VoltageScale::Range1;
    config.rcc.mux.otghssel = mux::Otghssel::Pll1P;
    let p = embassy_stm32::init(config);

    defmt::info!("studio_agent: booting (USB bulk RGB565 display agent)");

    // ── Display: asynchronous TX-only SPI2 using GPDMA ──
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(25_000_000);
    let spi = Spi::new_txonly(p.SPI2, p.PB10, p.PC3, p.GPDMA1_CH0, Irqs, spi_config);

    let cs = Output::new(p.PB9, Level::High, Speed::VeryHigh);
    let dc = Output::new(p.PB11, Level::Low, Speed::VeryHigh);
    let rst = Output::new(p.PA10, Level::High, Speed::VeryHigh);
    spawner.spawn(display_task(spi, cs, dc, rst).expect("create display task"));

    // ── FT6236 capacitive touch on I2C1 (SCL=PB2, SDA=PB1, INT=PE0) ──
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = Hertz(400_000);
    let i2c: TouchI2c = I2c::new_blocking(p.I2C1, p.PB2, p.PB1, i2c_config);
    let touch_int = Input::new(p.PE0, Pull::Up);
    let mut touch = FT6236::new(i2c);
    if touch.init(ft6236::Config::default()).is_err() {
        defmt::warn!("studio_agent: FT6236 not detected; touch uplink disabled");
    }

    // ── Vendor-specific USB bulk device on USB-HS (PD6 = DP, PD7 = DM) ──
    // Match Markham: two 512-byte OUT slots provide enough receive buffering.
    static EP_OUT: StaticCell<[u8; 1024]> = StaticCell::new();
    let mut driver_config = embassy_stm32::usb::Config::default();
    driver_config.vbus_detection = false;
    let driver = Driver::new_hs(
        p.USB_OTG_HS,
        Irqs,
        p.PD6,
        p.PD7,
        EP_OUT.init([0u8; 1024]),
        driver_config,
    );

    let mut usb_cfg = embassy_usb::Config::new(0x1209, 0xE611);
    usb_cfg.manufacturer = Some("embedded-gui");
    usb_cfg.product = Some("studio-agent");
    usb_cfg.serial_number = Some("wba65-live");
    usb_cfg.max_power = 500;
    usb_cfg.composite_with_iads = false;
    usb_cfg.device_class = 0x00;
    usb_cfg.device_sub_class = 0x00;
    usb_cfg.device_protocol = 0x00;

    static CFG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CTRL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CFG_DESC.init([0u8; 256]),
        BOS_DESC.init([0u8; 256]),
        MSOS_DESC.init([0u8; 256]),
        CTRL_BUF.init([0u8; 128]),
    );

    // Advertise WinUSB so Windows binds without Zadig. macOS/Linux claim the
    // same vendor interface directly through libusb/nusb.
    builder.msos_descriptor(0x0600_0000, 0x20);
    builder.msos_feature(CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        PropertyData::RegMultiSz(&["{8E91C330-D0C7-4A2F-A161-65A6BE772CA1}"]),
    ));

    let mut function = builder.function(0xFF, 0xFF, 0xFF);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0xFF, 0xFF, None);
    let mut rx = alt.endpoint_bulk_out(None, USB_MPS);
    let mut tx = alt.endpoint_bulk_in(None, USB_MPS);
    drop(alt);
    drop(interface);
    drop(function);

    spawner.spawn(usb_task(builder.build()).expect("spawn usb_task"));

    static DEC: StaticCell<Decoder<DEC_CAP>> = StaticCell::new();
    let dec = DEC.init(Decoder::new());

    let mut packet = [0u8; USB_MPS as usize];
    let mut touch_poll = Ticker::every(TOUCH_POLL);

    loop {
        rx.wait_enabled().await;
        defmt::info!("studio_agent: host connected");
        // Last reported touch, so we only emit on press, release, or a real
        // move. Reset per connection so a reconnect starts from "no contact".
        let mut last_touch: Option<(u16, u16)> = None;

        loop {
            // Race an inbound USB read against the touch poll tick so the panel
            // is sampled even while the host is idle (no OUT traffic pending).
            match select(rx.read(&mut packet), touch_poll.next()).await {
                Either::First(read) => {
                    let n = match read {
                        Ok(n) => n,
                        Err(_) => {
                            defmt::warn!("studio_agent: endpoint error, awaiting reconnect");
                            break;
                        }
                    };
                    for &b in &packet[..n] {
                        match dec.push(b) {
                            Ok(true) => match dec.message() {
                                Ok(Msg::Hello { .. }) => {
                                    let ready = Msg::Ready {
                                        proto: PROTO_VERSION,
                                        fb_w: PANEL_W,
                                        fb_h: PANEL_H,
                                        max_rect_bytes: (DEC_CAP as u32)
                                            - embedded_gui_live::FRAME_RECT_HEADER as u32,
                                    };
                                    let mut out = [0u8; 32];
                                    if let Ok(len) = ready.encode(&mut out) {
                                        let _ = tx.write_transfer(&out[..len], true).await;
                                    }
                                }
                                Ok(Msg::FrameRect {
                                    x, y, w, h, pixels, ..
                                }) => {
                                    if pixels.len() <= MAX_RECT_BYTES {
                                        let mut job = RectJob {
                                            x,
                                            y,
                                            w,
                                            h,
                                            len: pixels.len(),
                                            pixels_be: [0; MAX_RECT_BYTES],
                                        };
                                        // The wire format is little-endian
                                        // RGB565; ILI9341 wants each pixel MSB
                                        // first.
                                        for (src, dst) in pixels
                                            .chunks_exact(2)
                                            .zip(job.pixels_be.chunks_exact_mut(2))
                                        {
                                            dst[0] = src[1];
                                            dst[1] = src[0];
                                        }
                                        DISPLAY_QUEUE.send(job).await;
                                    }
                                }
                                Ok(Msg::Ping) => {
                                    let mut out = [0u8; 16];
                                    if let Ok(len) = Msg::Pong.encode(&mut out) {
                                        let _ = tx.write_transfer(&out[..len], true).await;
                                    }
                                }
                                _ => {}
                            },
                            Ok(false) => {}
                            Err(_) => { /* framing fault: decoder already resynced */ }
                        }
                    }
                }
                Either::Second(_) => {
                    if let Some(msg) = poll_touch(&mut touch, &touch_int, &mut last_touch) {
                        let mut out = [0u8; 16];
                        if let Ok(len) = msg.encode(&mut out) {
                            let _ = tx.write_transfer(&out[..len], true).await;
                        }
                    }
                }
            }
        }
    }
}

/// Reads the FT6236 and returns a [`Msg::Touch`] only when the state changes:
/// a new press, a release, or a move past [`TOUCH_MOVE_EPS`] while held.
fn poll_touch(
    touch: &mut FT6236<TouchI2c>,
    touch_int: &Input<'static>,
    last: &mut Option<(u16, u16)>,
) -> Option<Msg<'static>> {
    // The controller only asserts INT (active low) while a finger is present.
    if touch_int.is_high() {
        if let Some((x, y)) = last.take() {
            return Some(Msg::Touch {
                x,
                y,
                pressed: false,
            });
        }
        return None;
    }

    match touch.get_point0() {
        Ok(Some(pt)) if pt.event != EventType::LiftUp => {
            let (x, y) = map_touch(pt.x, pt.y);
            let moved = match *last {
                Some((lx, ly)) => {
                    x.abs_diff(lx) >= TOUCH_MOVE_EPS || y.abs_diff(ly) >= TOUCH_MOVE_EPS
                }
                None => true,
            };
            if moved {
                *last = Some((x, y));
                Some(Msg::Touch {
                    x,
                    y,
                    pressed: true,
                })
            } else {
                None
            }
        }
        _ => last.take().map(|(x, y)| Msg::Touch {
            x,
            y,
            pressed: false,
        }),
    }
}

async fn write_command(
    spi: &mut DisplaySpi,
    cs: &mut Output<'static>,
    dc: &mut Output<'static>,
    command: u8,
    args: &[u8],
) {
    cs.set_low();
    dc.set_low();
    let _ = spi.write(&[command]).await;
    if !args.is_empty() {
        dc.set_high();
        let _ = spi.write(args).await;
    }
    cs.set_high();
}

async fn set_window(
    spi: &mut DisplaySpi,
    cs: &mut Output<'static>,
    dc: &mut Output<'static>,
    x: u16,
    y: u16,
    w: u16,
    h: u16,
) {
    let x_end = x + w - 1;
    let y_end = y + h - 1;
    write_command(
        spi,
        cs,
        dc,
        0x2A,
        &[(x >> 8) as u8, x as u8, (x_end >> 8) as u8, x_end as u8],
    )
    .await;
    write_command(
        spi,
        cs,
        dc,
        0x2B,
        &[(y >> 8) as u8, y as u8, (y_end >> 8) as u8, y_end as u8],
    )
    .await;
}

async fn init_display(
    spi: &mut DisplaySpi,
    cs: &mut Output<'static>,
    dc: &mut Output<'static>,
    rst: &mut Output<'static>,
) {
    use embassy_time::{Duration, Timer};

    rst.set_low();
    Timer::after(Duration::from_micros(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(5)).await;

    // Same model options previously configured through mipidsi:
    // BGR + rotate 90 degrees + horizontal flip.
    write_command(spi, cs, dc, 0x36, &[0xE8]).await; // MADCTL
    write_command(spi, cs, dc, 0xB4, &[0x00]).await; // inversion control
    write_command(spi, cs, dc, 0x20, &[]).await; // normal color mode
    write_command(spi, cs, dc, 0x3A, &[0x55]).await; // RGB565
    write_command(spi, cs, dc, 0x13, &[]).await; // normal mode
    Timer::after(Duration::from_millis(120)).await;
    write_command(spi, cs, dc, 0x11, &[]).await; // sleep out
    Timer::after(Duration::from_millis(140)).await;
    write_command(spi, cs, dc, 0x29, &[]).await; // display on
}

async fn clear_display(spi: &mut DisplaySpi, cs: &mut Output<'static>, dc: &mut Output<'static>) {
    set_window(spi, cs, dc, 0, 0, PANEL_W, PANEL_H).await;
    write_command(spi, cs, dc, 0x2C, &[]).await;
    cs.set_low();
    dc.set_high();
    for _ in 0..(PANEL_W as usize * PANEL_H as usize * 2 / MAX_RECT_BYTES) {
        let _ = spi.write(&CLEAR_CHUNK).await;
    }
    cs.set_high();
}

async fn paint_rect_dma(
    spi: &mut DisplaySpi,
    cs: &mut Output<'static>,
    dc: &mut Output<'static>,
    job: &RectJob,
) {
    if job.w == 0
        || job.h == 0
        || job.x.saturating_add(job.w) > PANEL_W
        || job.y.saturating_add(job.h) > PANEL_H
        || job.len != job.w as usize * job.h as usize * 2
    {
        return;
    }

    set_window(spi, cs, dc, job.x, job.y, job.w, job.h).await;
    write_command(spi, cs, dc, 0x2C, &[]).await;
    cs.set_low();
    dc.set_high();
    let _ = spi.write(&job.pixels_be[..job.len]).await;
    cs.set_high();
}
