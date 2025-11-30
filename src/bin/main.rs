#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
              holding buffers for the duration of a data transfer."
)]

use defmt::{Format, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Clone, Copy, Debug, PartialEq, Format)]
pub struct RGB8 {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl RGB8 {
    #[must_use] 
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

// WS2812 timing constants (in nanoseconds)
const CODE_PERIOD_NS: u32 = 1250; // 800kHz
const T0H_NS: u32 = 400;
const T0L_NS: u32 = CODE_PERIOD_NS - T0H_NS;
const T1H_NS: u32 = 850;
const T1L_NS: u32 = CODE_PERIOD_NS - T1H_NS;

// Buffer size for one RGB LED (24 pulses + 1 delimiter)
const BUFFER_SIZE: usize = 25;

const V_MIN: f32 = 661.8;
const V_MAX: f32 = 1135.5;
const POLY_A: f32 = -2.429_344_2e-6;
const POLY_B: f32 = 6.477_230_7e-3;
const POLY_C: f32 = -3.222_632_4;
const FILTER_ALPHA: f32 = 0.2;

fn led_pulses_for_clock(src_clock_mhz: u32) -> (PulseCode, PulseCode) {
    (
        PulseCode::new(
            Level::High,
            ((T0H_NS * src_clock_mhz) / 1000) as u16,
            Level::Low,
            ((T0L_NS * src_clock_mhz) / 1000) as u16,
        ),
        PulseCode::new(
            Level::High,
            ((T1H_NS * src_clock_mhz) / 1000) as u16,
            Level::Low,
            ((T1L_NS * src_clock_mhz) / 1000) as u16,
        ),
    )
}

fn ws2812_encode(
    color: RGB8,
    pulses: (PulseCode, PulseCode),
    rmt_buffer: &mut [PulseCode; BUFFER_SIZE],
) {
    let bytes = [color.g, color.r, color.b];
    let mut idx = 0;

    for &byte in &bytes {
        for bit in (0..8).rev() {
            let is_set = (byte & (1 << bit)) != 0;
            rmt_buffer[idx] = if is_set { pulses.1 } else { pulses.0 };
            idx += 1;
        }
    }
    rmt_buffer[24] = PulseCode::new(Level::Low, 0, Level::Low, 0); // Delimiter
}

fn calculate_throttle(voltage_mv: f32) -> u8 {
    // 1. Clamp input to calibrated range
    let v = voltage_mv.clamp(V_MIN, V_MAX);

    // 2. Apply Polynomial: ax^2 + bx + c
    let t = (POLY_A * v * v) + (POLY_B * v) + POLY_C;

    // 3. Clamp result to 0.0 - 1.0 (float precision safety)
    let t_safe = t.clamp(0.0, 1.0);

    (t_safe * 100.0) as u8
}

fn throttle_to_color(throttle: u8) -> RGB8 {
    let t = f32::from(throttle) / 100.0;
    // Blue for 0% (Idle), Red for 100% (Full)
    let r = (255.0 * t) as u8;
    let b = (255.0 * (1.0 - t)) as u8;
    RGB8::new(r, 0, b)
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 0.6.0
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // Initialize ADC for hall effect sensor on GPIO4
    let mut adc_config = AdcConfig::new();
    let analog_pin = peripherals.GPIO4;
    let mut adc_pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(analog_pin, Attenuation::_6dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Initialize RMT for WS2812 control
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_idle_output(true);
    let channel_creator = rmt.channel0;
    let mut channel = channel_creator
        .configure_tx(peripherals.GPIO48, tx_config)
        .unwrap();

    // Precompute pulses based on actual clock
    let src_clock_mhz = esp_hal::clock::Clocks::get().apb_clock.as_mhz();
    let pulses = led_pulses_for_clock(src_clock_mhz);

    info!("WS2812 LED initialized on GPIO48, ADC on GPIO4");

    let _ = spawner;

    let mut rmt_buffer = [PulseCode::default(); BUFFER_SIZE];

    // Initialize filter state with the resting voltage (approximate)
    let mut filtered_mv: f32 = V_MIN;

    loop {
        let raw: u16 = nb::block!(adc.read_oneshot(&mut adc_pin)).unwrap();
        let current_mv = (f32::from(raw) / 4095.0) * 3300.0;

        // Apply EMA Filter: New = Old + Alpha * (New - Old)
        filtered_mv = filtered_mv + FILTER_ALPHA * (current_mv - filtered_mv);

        let throttle = calculate_throttle(filtered_mv);
        let color = throttle_to_color(throttle);

        ws2812_encode(color, pulses, &mut rmt_buffer);

        let transaction = channel.transmit(&rmt_buffer).unwrap();
        channel = transaction.wait().unwrap();

        info!(
            "Raw: {}mV | Filtered: {}mV | Throttle: {}% | LED: R={}, B={}",
            current_mv as u32, filtered_mv as u32, throttle, color.r, color.b
        );

        Timer::after(Duration::from_millis(10)).await;
    }
}
