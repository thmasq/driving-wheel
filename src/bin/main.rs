#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]
#![allow(
    clippy::missing_safety_doc,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use defmt::{error, info, warn};
use driving_wheel::smart_led::{RGB8, SmartLed};
use driving_wheel::touch::{
    self, TouchChannel, TouchChannelConfig, TouchInterrupts, TouchPin, TouchSensor, pin_state,
    sensor_state,
};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::Level;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::interrupt::{self, IsrCallback, Priority};
use esp_hal::peripherals::Interrupt;
use esp_hal::rmt::{Rmt, TxChannelConfig};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

static TOUCH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[esp_hal::ram]
extern "C" fn rtc_core_isr() {
    touch::on_interrupt_isr(TouchInterrupts::ACTIVE);
    TOUCH_SIGNAL.signal(());
}

// =============================================================================
// Constants
// =============================================================================

const V_MIN: f32 = 652.9;
const V_MAX: f32 = 1068.7;
const POLY_A: f32 = -3.714_882_5e-6;
const POLY_B: f32 = 8.800_675e-3;
const POLY_C: f32 = -4.162_483;
const FILTER_ALPHA: f32 = 0.2;
const TOUCH_THRESHOLD: u32 = 40_000;

// Steering Configuration
const STEERING_DEADZONE: f32 = 0.05; // 5% center deadzone
const STEERING_CURVE: f32 = 1.5; // Sensitivity curve (1.0 = linear)
const STEERING_SMOOTH: f32 = 0.2; // Smoothing factor
const FULL_LOCK_RADS: f32 = 1.57; // ~90 degrees is full lock

// =============================================================================
// Helpers & Algorithms
// =============================================================================

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

struct SteeringProcessor {
    center_offset: f32,
    deadzone: f32,
    sensitivity: f32,
    smoothing: f32,
    last_output: f32,
}

impl SteeringProcessor {
    pub fn new(deadzone: f32, sensitivity: f32, smoothing: f32) -> Self {
        Self {
            center_offset: 0.0,
            deadzone,
            sensitivity,
            smoothing,
            last_output: 0.0,
        }
    }

    #[allow(dead_code)]
    pub fn set_center(&mut self, current_angle: f32) {
        self.center_offset = current_angle;
    }

    pub fn process(&mut self, raw_angle_rads: f32) -> f32 {
        let centered = raw_angle_rads - self.center_offset;
        let mut normalized = (centered / FULL_LOCK_RADS).clamp(-1.0, 1.0);

        let val_abs = libm::fabsf(normalized);
        if val_abs < self.deadzone {
            normalized = 0.0;
        } else {
            let sign = if normalized > 0.0 { 1.0 } else { -1.0 };
            normalized = sign * ((val_abs - self.deadzone) / (1.0 - self.deadzone));
        }

        if normalized != 0.0 {
            let sign = if normalized > 0.0 { 1.0 } else { -1.0 };
            normalized = sign * libm::powf(libm::fabsf(normalized), self.sensitivity);
        }

        let alpha = 1.0 - self.smoothing;
        let output = (normalized * alpha) + (self.last_output * self.smoothing);

        self.last_output = output;
        output
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

    // --- 1. Init Touch Sensor ---
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
    info!("Touch Sensor initialized");

    // --- 3. Init I2C (MPU6050) ---
    let sda = peripherals.GPIO1;
    let scl = peripherals.GPIO2;

    let clocks = esp_hal::clock::Clocks::get();

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(sda)
    .with_scl(scl);

    let mut mpu = Mpu6050::new(i2c, Address::default()).unwrap();
    let mut delay = Delay::new();

    info!("Initializing MPU6050 DMP... (Don't move!)");
    if let Err(e) = mpu.initialize_dmp(&mut delay) {
        error!("MPU Init Failed: {:?}", defmt::Debug2Format(&e));
    } else {
        info!("MPU6050 DMP Ready!");
    }

    let mut steering_proc =
        SteeringProcessor::new(STEERING_DEADZONE, STEERING_CURVE, STEERING_SMOOTH);

    // --- 4. Init ADC (Throttle) ---
    let mut adc_config = AdcConfig::new();
    let analog_pin = peripherals.GPIO4;
    let mut adc_pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(analog_pin, Attenuation::_6dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // --- 5. Init LED (RMT) ---
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .unwrap()
        .into_async();

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_idle_output(true);

    let src_clock_mhz = clocks.apb_clock.as_mhz();
    let led_driver_uninit = SmartLed::new(rmt.channel0, peripherals.GPIO48);
    let mut led_driver = led_driver_uninit
        .initialize(tx_config, src_clock_mhz)
        .unwrap();

    let _ = spawner;
    let mut filtered_mv: f32 = V_MIN;

    let mut is_forward = true;

    info!("Starting Main Loop...");

    loop {
        Timer::after(Duration::from_millis(10)).await;

        // --- Task A: Touch Sensor (Direction) ---
        if TOUCH_SIGNAL.try_take().is_some() {
            is_forward = !is_forward;
            info!(
                "Direction: {}",
                if is_forward { "FORWARD" } else { "REVERSE" }
            );
            wait_for_release_and_rearm(&mut touch, &touch_pin_14).await;
        }

        // --- Task B: MPU6050 Polling (Steering) ---
        // We check the FIFO count directly. If >= 28 bytes, a packet is ready.
        match mpu.get_fifo_count() {
            Ok(count) => {
                if count >= 28 {
                    let mut buf = [0; 28];
                    if mpu.read_fifo(&mut buf).is_ok() {
                        if let Some(quat) = Quaternion::from_bytes(&buf[..16]) {
                            let ypr = YawPitchRoll::from(quat);
                            let steer_val = steering_proc.process(ypr.roll);
                            info!("Steering: {} (Roll: {})", steer_val, ypr.roll);
                        }
                    }
                }
            }
            Err(_) => {
                // I2C Error (Bus busy, sensor disconnected, etc)
                // We ignore it to keep the loop running
            }
        }

        // --- Task C: ADC & LED (Throttle) ---
        if let Ok(raw) = nb::block!(adc.read_oneshot(&mut adc_pin)) {
            let current_mv = (f32::from(raw) / 4095.0) * 3300.0;
            filtered_mv = filtered_mv + FILTER_ALPHA * (current_mv - filtered_mv);

            let throttle = calculate_throttle(filtered_mv);
            let color = get_mode_color(throttle, is_forward);

            if let Err(e) = led_driver.write(color).await {
                warn!("RMT Transmit Error: {:?}", e);
            }
        }
    }
}
