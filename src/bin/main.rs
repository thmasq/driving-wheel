#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]
#![allow(
    clippy::missing_safety_doc,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use defmt::{info, warn};
use driving_wheel::smart_led::{RGB8, SmartLed};
use driving_wheel::touch::{
    self, TouchChannel, TouchChannelConfig, TouchInterrupts, TouchPin, TouchSensor, pin_state,
    sensor_state,
};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::interrupt::{self, IsrCallback, Priority};
use esp_hal::peripherals::Interrupt;
use esp_hal::rmt::{Rmt, TxChannelConfig};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

static TOUCH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[esp_hal::ram]
extern "C" fn rtc_core_isr() {
    touch::on_interrupt_isr(TouchInterrupts::ACTIVE);
    TOUCH_SIGNAL.signal(());
}

const V_MIN: f32 = 652.9;
const V_MAX: f32 = 1068.7;
const POLY_A: f32 = -3.714_882_5e-6;
const POLY_B: f32 = 8.800_675e-3;
const POLY_C: f32 = -4.162_483;
const FILTER_ALPHA: f32 = 0.2;
const TOUCH_THRESHOLD: u32 = 40_000;

fn calculate_throttle(voltage_mv: f32) -> u8 {
    let v = voltage_mv.clamp(V_MIN, V_MAX);
    let t = (POLY_A * v * v) + (POLY_B * v) + POLY_C;
    ((t.clamp(0.0, 1.0)) * 100.0) as u8
}

fn get_mode_color(throttle: u8, is_forward: bool) -> RGB8 {
    let t = f32::from(throttle) / 100.0;
    let intensity = (255.0 * t) as u8;

    if is_forward {
        RGB8::new(0, intensity, 0) // Green
    } else {
        RGB8::new(intensity, 0, 0) // Red
    }
}

async fn wait_for_release_and_rearm(
    touch: &mut TouchSensor<sensor_state::Running>,
    pin: &TouchPin<pin_state::Analog>,
) {
    let mut steady_count = 0;

    loop {
        if !touch.is_channel_active(pin.channel()) {
            steady_count += 1;
        } else {
            steady_count = 0;
        }

        if steady_count >= 3 {
            break;
        }
        Timer::after(Duration::from_millis(20)).await;
    }

    touch.clear_interrupts(TouchInterrupts::ACTIVE);
    touch.enable_interrupts(TouchInterrupts::ACTIVE);
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    Timer::after(Duration::from_millis(100)).await;
    info!("Embassy initialized!");

    let mut touch_sensor = TouchSensor::new();

    let touch_pin_14 = TouchPin::new(TouchChannel::Touch14).into_analog();
    let channel_config = TouchChannelConfig {
        threshold: TOUCH_THRESHOLD,
        ..Default::default()
    };
    touch_sensor.config_channel(&touch_pin_14, channel_config);

    touch_sensor.enable_interrupts(TouchInterrupts::ACTIVE);

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

    let src_clock_mhz = esp_hal::clock::Clocks::get().apb_clock.as_mhz();
    let led_driver_uninit = SmartLed::new(rmt.channel0, peripherals.GPIO48);
    let mut led_driver = led_driver_uninit
        .initialize(tx_config, src_clock_mhz)
        .unwrap();

    let _ = spawner;
    let mut filtered_mv: f32 = V_MIN;

    let mut is_forward = true;

    info!("Starting Main Loop...");

    loop {
        let update_future = Timer::after(Duration::from_millis(10));
        let touch_future = TOUCH_SIGNAL.wait();

        match select(update_future, touch_future).await {
            Either::First(()) => {
                if let Ok(raw) = nb::block!(adc.read_oneshot(&mut adc_pin)) {
                    let current_mv = (f32::from(raw) / 4095.0) * 3300.0;
                    filtered_mv = filtered_mv + FILTER_ALPHA * (current_mv - filtered_mv);

                    let throttle = calculate_throttle(filtered_mv);
                    let color = get_mode_color(throttle, is_forward);

                    match led_driver.write(color).await {
                        Ok(()) => {}
                        Err(e) => {
                            warn!("RMT Transmit Error: {:?}", e);
                        }
                    }
                }
            }
            Either::Second(()) => {
                is_forward = !is_forward;

                if is_forward {
                    info!("Switched to FORWARD");
                } else {
                    info!("Switched to REVERSE");
                }

                wait_for_release_and_rearm(&mut touch, &touch_pin_14).await;
            }
        }
    }
}
