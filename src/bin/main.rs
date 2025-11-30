#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]

use defmt::{Format, info, warn};
use driving_wheel::esp32s3_touch_ll::TouchSensorLL;
use driving_wheel::touch::{TOUCH_LL_INTR_MASK_ACTIVE, TouchChannel, TouchPin, TouchSensor};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::interrupt::{self, IsrCallback, Priority}; // Import IsrCallback
use esp_hal::peripherals::Interrupt;
use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

static TOUCH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[esp_hal::ram]
extern "C" fn rtc_core_isr() {
    unsafe {
        TouchSensorLL::interrupt_disable(TOUCH_LL_INTR_MASK_ACTIVE);
        TouchSensorLL::interrupt_clear(TOUCH_LL_INTR_MASK_ACTIVE);
    }
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

// WS2812 timing constants
const CODE_PERIOD_NS: u32 = 1250;
const T0H_NS: u32 = 400;
const T0L_NS: u32 = CODE_PERIOD_NS - T0H_NS;
const T1H_NS: u32 = 850;
const T1L_NS: u32 = CODE_PERIOD_NS - T1H_NS;
const BUFFER_SIZE: usize = 25;

const V_MIN: f32 = 652.9;
const V_MAX: f32 = 1068.7;
const POLY_A: f32 = -3.71488240e-06;
const POLY_B: f32 = 8.80067475e-03;
const POLY_C: f32 = -4.16248326e+00;
const FILTER_ALPHA: f32 = 0.2;
const TOUCH_THRESHOLD: u32 = 40_000;

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
    rmt_buffer[24] = PulseCode::new(Level::Low, 0, Level::Low, 0);
}

fn calculate_throttle(voltage_mv: f32) -> u8 {
    let v = voltage_mv.clamp(V_MIN, V_MAX);
    let t = (POLY_A * v * v) + (POLY_B * v) + POLY_C;
    ((t.clamp(0.0, 1.0)) * 100.0) as u8
}

fn throttle_to_color(throttle: u8) -> RGB8 {
    let t = f32::from(throttle) / 100.0;
    let r = (255.0 * t) as u8;
    let b = (255.0 * (1.0 - t)) as u8;
    RGB8::new(r, 0, b)
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let mut touch_sensor = TouchSensor::new();
    let touch_pin_14 = TouchPin::new(TouchChannel::Num14).into_analog();
    touch_sensor.config_channel(&touch_pin_14);
    touch_sensor.set_threshold(TouchChannel::Num14, TOUCH_THRESHOLD);

    touch_sensor.interrupt_enable(TOUCH_LL_INTR_MASK_ACTIVE);

    unsafe {
        esp_hal::interrupt::bind_interrupt(Interrupt::RTC_CORE, IsrCallback::new(rtc_core_isr));
        interrupt::enable(Interrupt::RTC_CORE, Priority::Priority1).unwrap();
    }

    let mut touch = touch_sensor.start();
    info!("Touch Sensor initialized and bound");

    let mut adc_config = AdcConfig::new();
    let analog_pin = peripherals.GPIO4;
    let mut adc_pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(analog_pin, Attenuation::_6dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .unwrap()
        .into_async();

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

    async fn wait_for_release_and_rearm(
        touch: &mut TouchSensor<driving_wheel::touch::sensor_state::Running>,
    ) {
        loop {
            if !touch.is_channel_active(TouchChannel::Num14) {
                break;
            }
            Timer::after(Duration::from_millis(50)).await;
        }

        unsafe {
            TouchSensorLL::interrupt_clear(TOUCH_LL_INTR_MASK_ACTIVE);
            TouchSensorLL::interrupt_enable(TOUCH_LL_INTR_MASK_ACTIVE);
        }
    }

    loop {
        if is_paused {
            info!("System Paused. Waiting for touch to resume...");
            TOUCH_SIGNAL.wait().await;
            wait_for_release_and_rearm(&mut touch).await;
            is_paused = false;
            info!("Touch detected: Resuming system.");
        } else {
            let update_future = Timer::after(Duration::from_millis(10));
            let touch_future = TOUCH_SIGNAL.wait();

            match select(update_future, touch_future).await {
                Either::First(_) => match nb::block!(adc.read_oneshot(&mut adc_pin)) {
                    Ok(raw) => {
                        let current_mv = (f32::from(raw) / 4095.0) * 3300.0;
                        filtered_mv = filtered_mv + FILTER_ALPHA * (current_mv - filtered_mv);

                        let throttle = calculate_throttle(filtered_mv);
                        let color = throttle_to_color(throttle);
                        ws2812_encode(color, pulses, &mut rmt_buffer);

                        match channel.transmit(&rmt_buffer).await {
                            Ok(_) => {}
                            Err(e) => {
                                warn!("RMT Transmit Error: {:?}", e);
                            }
                        }
                    }
                    Err(_) => {}
                },
                Either::Second(_) => {
                    info!("Touch detected: Pausing system.");
                    is_paused = true;
                    wait_for_release_and_rearm(&mut touch).await;
                }
            }
        }
    }
}
