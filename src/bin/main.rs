#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
              holding buffers for the duration of a data transfer."
)]

use defmt::{Format, info};
use driving_wheel::esp32s3_touch_ll::TouchSensorLL; // Needed for ISR
use driving_wheel::touch::{TOUCH_LL_INTR_MASK_ACTIVE, TouchChannel, TouchPin, TouchSensor};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::handler;
use esp_hal::interrupt::{self, Priority};
use esp_hal::peripherals::Interrupt;
use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

// Global Signal to notify the async task from the Interrupt Context
static TOUCH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// ----------------------------------------------------------------------------
// Interrupt Handler
// ----------------------------------------------------------------------------
#[handler]
fn rtc_core_isr() {
    unsafe {
        // 1. Clear the interrupt status in the Touch Controller immediately
        // We use the LL driver here because we cannot access the high-level 'TouchSensor'
        // struct (which owns the state) from a static ISR context easily.
        TouchSensorLL::interrupt_clear(TOUCH_LL_INTR_MASK_ACTIVE);
    }

    // 2. Notify the main loop
    TOUCH_SIGNAL.signal(());
}

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

// Touch threshold - adjust based on your hardware calibration
const TOUCH_THRESHOLD: u32 = 40_000;

fn led_pulses_for_clock(src_clock_mhz: u32) -> (PulseCode, PulseCode) {
    (
        #[allow(clippy::cast_possible_truncation)]
        PulseCode::new(
            Level::High,
            ((T0H_NS * src_clock_mhz) / 1000) as u16,
            Level::Low,
            ((T0L_NS * src_clock_mhz) / 1000) as u16,
        ),
        #[allow(clippy::cast_possible_truncation)]
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

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let result = (t_safe * 100.0) as u8;

    result
}

fn throttle_to_color(throttle: u8) -> RGB8 {
    let t = f32::from(throttle) / 100.0;
    // Blue for 0% (Idle), Red for 100% (Full)
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let r = (255.0 * t) as u8;
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
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

    // --- Initialize Touch Sensor with Interrupts ---
    let mut touch_sensor = TouchSensor::new();

    // 1. Select Pin 14 (Touch14) and initialize it as Analog
    let touch_pin_14 = TouchPin::new(TouchChannel::Num14).into_analog();

    // 2. Configure the channel
    touch_sensor.config_channel(&touch_pin_14);

    // 3. Set Threshold for Interrupt
    // Note: "Active" interrupt triggers when value < benchmark * threshold (approx logic)
    // You might need to tune this or the 'active threshold' register depending on behavior
    touch_sensor.set_threshold(TouchChannel::Num14, TOUCH_THRESHOLD);

    // 4. Enable "Active" Interrupt (Triggers when touched)
    touch_sensor.interrupt_enable(TOUCH_LL_INTR_MASK_ACTIVE);

    // 5. Bind the Global RTC Interrupt to our handler
    interrupt::enable(Interrupt::RTC_CORE, Priority::Priority1).unwrap();

    // 6. Start the Touch FSM
    let _touch = touch_sensor.start();

    info!("Touch Sensor initialized with Interrupts on Ch14");

    // Initialize ADC
    let mut adc_config = AdcConfig::new();
    let analog_pin = peripherals.GPIO4;
    let mut adc_pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(analog_pin, Attenuation::_6dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Initialize RMT
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

    let src_clock_mhz = esp_hal::clock::Clocks::get().apb_clock.as_mhz();
    let pulses = led_pulses_for_clock(src_clock_mhz);

    let _ = spawner;
    let mut rmt_buffer = [PulseCode::default(); BUFFER_SIZE];
    let mut filtered_mv: f32 = V_MIN;
    let mut is_paused = false;

    info!("Starting Main Loop...");

    loop {
        if is_paused {
            // If paused, we ONLY wait for the touch signal to resume
            info!("System Paused. Waiting for touch to resume...");
            TOUCH_SIGNAL.wait().await; // Asynchronously sleep until ISR fires
            is_paused = false;
            info!("Touch detected: Resuming system.");
            // Debounce slightly to prevent immediate re-pause on same touch
            Timer::after(Duration::from_millis(500)).await;
        } else {
            // If running, we race two futures:
            // 1. The Update Interval Timer (10ms)
            // 2. The Touch Signal (Interrupt)
            let update_future = Timer::after(Duration::from_millis(10));
            let touch_future = TOUCH_SIGNAL.wait();

            match select(update_future, touch_future).await {
                Either::First(_) => {
                    // --- Timer expired, run update logic ---
                    let raw: u16 = nb::block!(adc.read_oneshot(&mut adc_pin)).unwrap();
                    let current_mv = (f32::from(raw) / 4095.0) * 3300.0;
                    filtered_mv = filtered_mv + FILTER_ALPHA * (current_mv - filtered_mv);

                    let throttle = calculate_throttle(filtered_mv);
                    let color = throttle_to_color(throttle);

                    ws2812_encode(color, pulses, &mut rmt_buffer);
                    let transaction = channel.transmit(&rmt_buffer).unwrap();
                    channel = transaction.wait().unwrap();

                    // Optional log reduction
                    // info!("Throttle: {}%", throttle);
                }
                Either::Second(_) => {
                    // --- Touch Interrupt fired ---
                    info!("Touch detected: Pausing system.");
                    is_paused = true;
                    // Debounce
                    Timer::after(Duration::from_millis(500)).await;
                }
            }
        }
    }
}
