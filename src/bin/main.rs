#![no_std]
#![no_main]
#![deny(clippy::mem_forget)]
#![allow(
    clippy::missing_safety_doc,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]

use defmt::{error, info, warn};
use driving_wheel::smart_led::{RGB8, SmartLed};
use embassy_executor::Spawner;
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer, with_timeout};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::Level;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rmt::{Rmt, TxChannelConfig};
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

use esp_radio::wifi::{
    ClientConfig, Config as WifiDriverConfig, ModeConfig, WifiController, WifiDevice, WifiEvent,
};
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};

use esp_backtrace as _;
use esp_println as _;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

// =============================================================================
// Constants
// =============================================================================

const V_MIN: f32 = 652.9;
const V_MAX: f32 = 1068.7;
const POLY_A: f32 = -3.714_882_5e-6;
const POLY_B: f32 = 8.800_675e-3;
const POLY_C: f32 = -4.162_483;
const FILTER_ALPHA: f32 = 0.2;

// Steering Configuration
const STEERING_DEADZONE: f32 = 0.05;
const STEERING_CURVE: f32 = 1.5;
const STEERING_SMOOTH: f32 = 0.2;
const LOCK_RIGHT_RADS: f32 = 1.4567; // Max reading when turned right
const LOCK_LEFT_RADS: f32 = -1.6208; // Max reading when turned left

// Networking
const SSID: &str = "FSE-Tsunderacer";
const PASSWORD: &str = "cirnobaka9";
const REMOTE_IP: Ipv4Address = Ipv4Address::new(192, 168, 9, 9);
const REMOTE_PORT: u16 = 9999;

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

        let scale_factor = if centered >= 0.0 {
            LOCK_RIGHT_RADS
        } else {
            -LOCK_LEFT_RADS
        };

        let mut normalized = (centered / scale_factor).clamp(-1.0, 1.0);

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

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Starting Wi-Fi Connection task");
    loop {
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into()),
            );

            controller.set_config(&client_config).unwrap();
            info!("Starting Wi-Fi controller...");
            controller.start().unwrap();
        }

        info!("Commanding Wi-Fi Connect...");
        match controller.connect() {
            Ok(()) => {
                info!("Waiting for link...");
                controller.wait_for_event(WifiEvent::StaConnected).await;
                info!("Wi-Fi Link Established!");

                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                warn!("Wi-Fi Disconnected! Retrying...");
                Timer::after(Duration::from_millis(5000)).await;
            }
            Err(e) => {
                warn!("Failed to initiate connection: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.init($val);
        x
    }};
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let rng = Rng::new();
    let seed = u64::from(rng.random());

    esp_rtos::start(timg0.timer0);

    Timer::after(Duration::from_millis(100)).await;
    info!("Embassy initialized!");

    // --- 0. Init Wi-Fi & Network Stack ---
    let init = mk_static!(esp_radio::Controller<'static>, esp_radio::init().unwrap());

    let (controller, interfaces) =
        esp_radio::wifi::new(init, peripherals.WIFI, WifiDriverConfig::default()).unwrap();

    let wifi_interface = interfaces.sta;

    let controller_ip = Ipv4Address::new(192, 168, 9, 10);

    let net_config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(controller_ip, 24),
        gateway: Some(Ipv4Address::new(192, 168, 9, 1)),
        dns_servers: Default::default(),
    });

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();

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

    let mut filtered_mv: f32 = V_MIN;
    let is_forward = true;
    let mut current_steering: f32 = 0.0;
    let mut throttle_u8: u8 = 0;

    // --- 6. Prepare UDP Socket ---
    let mut rx_buffer = [0; 64];
    let mut tx_buffer = [0; 64];
    let mut rx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 4];
    let mut tx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 4];

    let mut socket = embassy_net::udp::UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    socket.bind(0).unwrap();

    info!("Starting Main Loop...");

    loop {
        Timer::after(Duration::from_millis(1)).await;

        // --- Task B: MPU6050 Polling (Steering) ---
        if let Ok(count) = mpu.get_fifo_count() {
            if count > 280 {
                warn!("MPU FIFO overflow (count: {}). Resetting...", count);
                let _ = mpu.reset_fifo();
            } else if count >= 28 {
                let packets = count / 28;
                let mut buf = [0; 28];

                for _ in 0..(packets - 1) {
                    let _ = mpu.read_fifo(&mut buf);
                }

                if mpu.read_fifo(&mut buf).is_ok()
                    && let Some(quat) = Quaternion::from_bytes(&buf[..16])
                {
                    let ypr = YawPitchRoll::from(quat);
                    let steer_val = steering_proc.process(ypr.roll);
                    current_steering = steer_val;
                }
            }
        }

        // --- Task C: ADC & LED (Throttle) ---
        if let Ok(raw) = nb::block!(adc.read_oneshot(&mut adc_pin)) {
            let current_mv = (f32::from(raw) / 4095.0) * 3300.0;
            filtered_mv = filtered_mv + FILTER_ALPHA * (current_mv - filtered_mv);

            throttle_u8 = calculate_throttle(filtered_mv);
            let color = get_mode_color(throttle_u8, is_forward);

            if let Err(e) = led_driver.write(color).await {
                warn!("RMT Transmit Error: {:?}", e);
            }
        }

        // --- Task D: Network (Send UDP) ---
        if stack.is_config_up() && stack.is_link_up() {
            let remote_endpoint =
                embassy_net::IpEndpoint::new(embassy_net::IpAddress::Ipv4(REMOTE_IP), REMOTE_PORT);

            let throttle_i8 = if is_forward {
                throttle_u8 as i8
            } else {
                -(throttle_u8 as i8)
            };

            let steering_i8 = (current_steering * 100.0) as i8;
            let payload = [throttle_i8 as u8, steering_i8 as u8];

            match with_timeout(
                Duration::from_millis(50), // Drop packet if it takes > 50ms
                socket.send_to(&payload, remote_endpoint),
            )
            .await
            {
                Ok(Ok(())) => {} // Success
                Ok(Err(e)) => warn!("UDP Send Error: {:?}", e),
                Err(_) => {
                    // Timeout occurred
                    // We don't log here to avoid flooding console,
                    // but we know we skipped a frame to keep loop alive.
                }
            }
        }
    }
}
