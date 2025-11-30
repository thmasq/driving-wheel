//! High-level, Safe Touch Sensor Driver for ESP32-S3 using Typestate Pattern.
//!
//! # Features
//! - **Pin Safety**: Enforces that a GPIO pin must be switched to Analog mode before it can be configured as a Touch Channel.
//! - **State Safety**: Distinguishes between `Config` (setup) and `Running` (measurement) states. Data reading is only allowed in `Running` state; Configuration is only allowed in `Config` state.
//! - **Interrupt Safety**: Uses bitflags to prevent invalid interrupt mask operations.

use crate::esp32s3_touch_ll::{self as ll};
use bitflags::bitflags;
use core::marker::PhantomData;

// =============================================================================
// Interrupt Flags (Safe Abstraction)
// =============================================================================

bitflags! {
    /// Safe abstraction for Touch Sensor Interrupts.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct TouchInterrupts: u32 {
        const SCAN_DONE = ll::TOUCH_LL_INTR_MASK_SCAN_DONE;
        const DONE      = ll::TOUCH_LL_INTR_MASK_DONE;
        const ACTIVE    = ll::TOUCH_LL_INTR_MASK_ACTIVE;
        const INACTIVE  = ll::TOUCH_LL_INTR_MASK_INACTIVE;
        const TIMEOUT   = ll::TOUCH_LL_INTR_MASK_TIMEOUT;
        const PROX_DONE = ll::TOUCH_LL_INTR_MASK_PROX_DONE;
    }
}

// =============================================================================
// Channel & Pin Definitions
// =============================================================================

/// Touch Sensor Channels (1-14).
///
/// Channel 0 is reserved for internal denoise and has no physical pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TouchChannel {
    Touch1 = 1,
    Touch2 = 2,
    Touch3 = 3,
    Touch4 = 4,
    Touch5 = 5,
    Touch6 = 6,
    Touch7 = 7,
    Touch8 = 8,
    Touch9 = 9,
    Touch10 = 10,
    Touch11 = 11,
    Touch12 = 12,
    Touch13 = 13,
    Touch14 = 14,
}

impl TouchChannel {
    #[must_use]
    pub const fn to_u32(self) -> u32 {
        self as u32
    }

    /// On ESP32-S3, Touch Channel N corresponds directly to GPIO N.
    #[must_use]
    pub const fn to_gpio_num(self) -> u32 {
        self as u32
    }
}

// =============================================================================
// Typestate Definitions
// =============================================================================

/// State markers for the `TouchSensor` controller.
pub mod sensor_state {
    /// Controller is stopped and ready for configuration.
    pub struct Config;
    /// Controller FSM is running continuously.
    pub struct Running;
}

/// State markers for `TouchPins`.
pub mod pin_state {
    /// Pin is in default digital mode (or uninitialized).
    pub struct Digital;
    /// Pin has been configured as Analog (High-Z, buffers disabled).
    pub struct Analog;
}

// =============================================================================
// Pin Wrapper
// =============================================================================

/// A wrapper around a `TouchChannel` representing a physical pin in a specific state.
pub struct TouchPin<S> {
    channel: TouchChannel,
    _state: PhantomData<S>,
}

impl TouchPin<pin_state::Digital> {
    /// Create a new `TouchPin` handle for a specific channel.
    /// Starts in `Digital` (uninitialized) state.
    #[must_use]
    pub fn new(channel: TouchChannel) -> Self {
        Self {
            channel,
            _state: PhantomData,
        }
    }

    /// Consume the Digital pin and transition it to Analog mode.
    ///
    /// This physically disables the digital IO buffers and pull-ups/downs on the GPIO,
    /// preparing it for capacitive sensing.
    #[must_use]
    pub fn into_analog(self) -> TouchPin<pin_state::Analog> {
        unsafe {
            ll::TouchSensorLL::gpio_set_analog(self.channel.to_gpio_num());
        }
        TouchPin {
            channel: self.channel,
            _state: PhantomData,
        }
    }
}

impl TouchPin<pin_state::Analog> {
    /// Get the underlying channel enum.
    #[must_use]
    pub fn channel(&self) -> TouchChannel {
        self.channel
    }
}

// =============================================================================
// Configuration Builders
// =============================================================================

// Re-export useful enums for configuration
pub use crate::esp32s3_touch_ll::{
    TouchChargeSpeed, TouchDenoiseCap, TouchDenoiseRes, TouchFilterMode, TouchInitChargeVolt,
    TouchSmoothMode, TouchVoltAtten, TouchVoltLimHigh, TouchVoltLimLow,
};

/// Global configuration for the Touch Sensor Controller.
#[derive(Debug, Clone)]
pub struct TouchConfig {
    /// Ticks for measurement interval (0 ~ 0xFFFF).
    pub measurement_interval_ticks: u16,
    /// Sleep cycle ticks between measurements (0 ~ 0xFF).
    pub sleep_cycle: u8,
    /// Internal denoise configuration (None to disable).
    pub denoise: Option<TouchDenoiseConfig>,
    /// Filter configuration (None to disable).
    pub filter: Option<TouchFilterConfig>,
    pub voltage_high: TouchVoltLimHigh,
    pub voltage_low: TouchVoltLimLow,
    pub voltage_atten: TouchVoltAtten,
}

impl Default for TouchConfig {
    fn default() -> Self {
        Self {
            measurement_interval_ticks: 500,
            sleep_cycle: 0xF,
            denoise: None, // Denoise disabled by default
            filter: Some(TouchFilterConfig::default()),
            voltage_high: TouchVoltLimHigh::V2_7,
            voltage_low: TouchVoltLimLow::V0_5,
            voltage_atten: TouchVoltAtten::V0_5,
        }
    }
}

/// Configuration for a specific channel.
#[derive(Debug, Clone, Copy)]
pub struct TouchChannelConfig {
    pub charge_speed: TouchChargeSpeed,
    pub init_charge_volt: TouchInitChargeVolt,
    pub threshold: u32,
}

impl Default for TouchChannelConfig {
    fn default() -> Self {
        Self {
            charge_speed: TouchChargeSpeed::Speed7,
            init_charge_volt: TouchInitChargeVolt::High,
            threshold: 40_000,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TouchFilterConfig {
    pub mode: TouchFilterMode,
    pub debounce_cnt: u32, // 0-7
    pub noise_thr: u32,    // 0-3
    pub jitter_step: u32,  // 0-15
    pub smh_lvl: TouchSmoothMode,
}

impl Default for TouchFilterConfig {
    fn default() -> Self {
        Self {
            mode: TouchFilterMode::Iir16,
            debounce_cnt: 1,
            noise_thr: 0,
            jitter_step: 4,
            smh_lvl: TouchSmoothMode::Iir2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TouchDenoiseConfig {
    pub grade: TouchDenoiseRes,
    pub cap_level: TouchDenoiseCap,
}

impl Default for TouchDenoiseConfig {
    fn default() -> Self {
        Self {
            grade: TouchDenoiseRes::Bit12,
            cap_level: TouchDenoiseCap::Cap5pf,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TouchWaterproofConfig {
    pub guard_ring_pad: TouchChannel,
    pub shield_driver: u32, // 0-7
}

// =============================================================================
// Touch Sensor Driver
// =============================================================================

/// The main driver struct.
///
/// The generic `S` parameter tracks the state of the driver (`Config` or `Running`).
pub struct TouchSensor<S> {
    _state: PhantomData<S>,
}

impl Default for TouchSensor<sensor_state::Config> {
    fn default() -> Self {
        Self::new()
    }
}

impl TouchSensor<sensor_state::Config> {
    /// Create a new `TouchSensor` driver in Config state.
    ///
    /// Performs hardware reset and default initialization matching ESP-IDF defaults.
    #[must_use]
    pub fn new() -> Self {
        unsafe {
            // 1. Stop FSM and disable interrupts
            ll::TouchSensorLL::stop_fsm_repeated_timer();
            ll::TouchSensorLL::interrupt_disable(ll::TOUCH_LL_INTR_MASK_ALL);
            ll::TouchSensorLL::interrupt_clear(ll::TOUCH_LL_INTR_MASK_ALL);

            // 2. Clear channel masks
            ll::TouchSensorLL::clear_channel_mask(0x7FFF);
            ll::TouchSensorLL::clear_active_channel_status();

            // 3. Set Default Timing
            ll::TouchSensorLL::set_measure_interval_ticks(500);
            ll::TouchSensorLL::set_power_on_wait_cycle(0xF);

            // 4. Power & Voltage Config
            ll::TouchSensorLL::set_bias_type(ll::TouchBiasType::SelfBias);
            ll::TouchSensorLL::set_voltage_high(TouchVoltLimHigh::V2_7);
            ll::TouchSensorLL::set_voltage_low(TouchVoltLimLow::V0_5);
            ll::TouchSensorLL::set_voltage_attenuation(TouchVoltAtten::V0_5);

            // 5. Idle connection
            ll::TouchSensorLL::set_idle_channel_connection(ll::TouchIdleConn::Gnd);

            // 6. Enable Clock Gate
            ll::TouchSensorLL::enable_clock_gate(true);

            // 7. Reset Benchmarks
            ll::TouchSensorLL::reset_benchmark(0); // Denoise
            for i in 1..=14 {
                ll::TouchSensorLL::reset_benchmark(i);
            }
            ll::TouchSensorLL::sleep_reset_benchmark();
        }

        TouchSensor {
            _state: PhantomData,
        }
    }

    /// Apply global configuration settings.
    pub fn apply_config(&mut self, config: &TouchConfig) {
        unsafe {
            ll::TouchSensorLL::set_measure_interval_ticks(config.measurement_interval_ticks);
            ll::TouchSensorLL::set_power_on_wait_cycle(config.sleep_cycle);
            ll::TouchSensorLL::set_voltage_high(config.voltage_high);
            ll::TouchSensorLL::set_voltage_low(config.voltage_low);
            ll::TouchSensorLL::set_voltage_attenuation(config.voltage_atten);

            if let Some(filter) = &config.filter {
                ll::TouchSensorLL::set_filter_mode(filter.mode);
                ll::TouchSensorLL::set_debounce(filter.debounce_cnt);
                ll::TouchSensorLL::set_filter_active_hysteresis(filter.noise_thr);
                ll::TouchSensorLL::set_jitter_step(filter.jitter_step);
                ll::TouchSensorLL::set_smooth_mode(filter.smh_lvl);
                ll::TouchSensorLL::filter_enable(true);
            } else {
                ll::TouchSensorLL::filter_enable(false);
            }

            if let Some(denoise) = &config.denoise {
                ll::TouchSensorLL::denoise_set_resolution(denoise.grade);
                ll::TouchSensorLL::denoise_set_cap(denoise.cap_level);
                ll::TouchSensorLL::clear_channel_mask(1); // Ensure ch0 is reserved
                ll::TouchSensorLL::denoise_enable(true);
            } else {
                ll::TouchSensorLL::denoise_enable(false);
            }
        }
    }

    /// Add and configure a channel for measurement.
    ///
    /// **Typestate Enforcement**: This method requires a `TouchPin<pin_state::Analog>`.
    /// You must call `.into_analog()` on a `TouchPin` before passing it here.
    pub fn config_channel(
        &mut self,
        pin: &TouchPin<pin_state::Analog>,
        config: TouchChannelConfig,
    ) {
        let ch_num = pin.channel.to_u32();
        unsafe {
            ll::TouchSensorLL::set_init_charge_voltage(ch_num, config.init_charge_volt);
            ll::TouchSensorLL::set_charge_speed(ch_num, config.charge_speed);
            ll::TouchSensorLL::set_chan_active_threshold(ch_num, config.threshold);
            ll::TouchSensorLL::enable_channel_mask(1 << ch_num);
        }
    }

    /// Disable a channel.
    pub fn disable_channel(&mut self, channel: TouchChannel) {
        unsafe {
            ll::TouchSensorLL::clear_channel_mask(1 << channel.to_u32());
        }
    }

    /// Configure waterproof/shielding.
    pub fn set_waterproof_config(&mut self, config: &TouchWaterproofConfig) {
        unsafe {
            ll::TouchSensorLL::set_waterproof_guard_chan(config.guard_ring_pad.to_u32());
            ll::TouchSensorLL::set_shield_driver(config.shield_driver);
            // Remove guard pad from active scan
            ll::TouchSensorLL::clear_channel_mask(1 << config.guard_ring_pad.to_u32());
            ll::TouchSensorLL::waterproof_enable(true);
        }
    }

    /// Enable interrupts.
    /// Protected by a critical section to prevent RMW race conditions.
    pub fn enable_interrupts(&mut self, interrupts: TouchInterrupts) {
        critical_section::with(|_cs| unsafe {
            ll::TouchSensorLL::interrupt_enable(interrupts.bits());
        });
    }

    /// Disable interrupts.
    /// Protected by a critical section to prevent RMW race conditions.
    pub fn disable_interrupts(&mut self, interrupts: TouchInterrupts) {
        critical_section::with(|_cs| unsafe {
            ll::TouchSensorLL::interrupt_disable(interrupts.bits());
        });
    }

    /// Trigger a single software measurement (One-Shot) and block until done.
    ///
    /// This is only allowed in `Config` state.
    pub fn measure_oneshot_blocking(&mut self) {
        unsafe {
            ll::TouchSensorLL::trigger_oneshot_measurement();
            while !ll::TouchSensorLL::is_measure_done() {}
        }
    }

    /// Transition to `Running` state by starting the FSM repeated timer.
    #[must_use]
    pub fn start(self) -> TouchSensor<sensor_state::Running> {
        unsafe {
            ll::TouchSensorLL::enable_fsm_timer(true);
            ll::TouchSensorLL::start_fsm_repeated_timer();
        }
        TouchSensor {
            _state: PhantomData,
        }
    }
}

impl TouchSensor<sensor_state::Running> {
    /// Check if a measurement measurement cycle has completed.
    #[must_use]
    pub fn is_measure_done(&self) -> bool {
        unsafe { ll::TouchSensorLL::is_measure_done() }
    }

    /// Read raw data.
    #[must_use]
    pub fn read_raw(&self, channel: TouchChannel) -> u32 {
        unsafe { ll::TouchSensorLL::read_raw_data(channel.to_u32()) }
    }

    /// Read smoothed (filtered) data.
    #[must_use]
    pub fn read_smooth(&self, channel: TouchChannel) -> u32 {
        unsafe { ll::TouchSensorLL::read_smooth_data(channel.to_u32()) }
    }

    /// Read benchmark value.
    #[must_use]
    pub fn read_benchmark(&self, channel: TouchChannel) -> u32 {
        unsafe { ll::TouchSensorLL::read_benchmark(channel.to_u32()) }
    }

    /// Get the current proximity measurement count for a specific channel.
    #[must_use]
    pub fn get_proximity_count(&self, channel: TouchChannel) -> u32 {
        unsafe { ll::TouchSensorLL::get_proximity_curr_scan_cnt(channel.to_u32()) }
    }

    /// Get interrupt status.
    #[must_use]
    pub fn get_interrupt_status(&self) -> TouchInterrupts {
        let raw = unsafe { ll::TouchSensorLL::get_intr_status_mask() };
        TouchInterrupts::from_bits_truncate(raw)
    }

    /// Check if a specific channel is currently active (touched).
    #[must_use]
    pub fn is_channel_active(&self, channel: TouchChannel) -> bool {
        unsafe {
            let mask = ll::TouchSensorLL::get_active_channel_mask();
            (mask & (1 << channel.to_u32())) != 0
        }
    }

    /// Clear interrupt flags.
    pub fn clear_interrupts(&mut self, interrupts: TouchInterrupts) {
        critical_section::with(|_cs| unsafe {
            ll::TouchSensorLL::interrupt_clear(interrupts.bits());
        });
    }

    /// Enable interrupts (e.g., to re-arm after a pause).
    pub fn enable_interrupts(&mut self, interrupts: TouchInterrupts) {
        critical_section::with(|_cs| unsafe {
            ll::TouchSensorLL::interrupt_enable(interrupts.bits());
        });
    }

    /// Stop the FSM and return to `Config` state.
    #[must_use]
    pub fn stop(self) -> TouchSensor<sensor_state::Config> {
        unsafe {
            ll::TouchSensorLL::stop_fsm_repeated_timer();
            ll::TouchSensorLL::enable_fsm_timer(false);
        }
        TouchSensor {
            _state: PhantomData,
        }
    }
}

// =============================================================================
// Static Helpers for ISRs
// =============================================================================

/// Safe helper to be called inside an `extern "C"` ISR.
///
/// This safely handles the low-level register writes required to acknowledge
/// and silence interrupts within the ISR context.
///
/// # Example
/// ```rust,ignore
/// extern "C" fn rtc_core_isr() {
///     // Safely handle "Active" interrupt
///     driving_wheel::touch::on_interrupt_isr(TouchInterrupts::ACTIVE);
///     // Signal application...
/// }
/// ```
pub fn on_interrupt_isr(interrupts: TouchInterrupts) {
    let mask = interrupts.bits();
    critical_section::with(|_cs| unsafe {
        ll::TouchSensorLL::interrupt_disable(mask);
        ll::TouchSensorLL::interrupt_clear(mask);
    });
}
