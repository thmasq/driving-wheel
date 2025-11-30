//! High-level, Safe Touch Sensor Driver for ESP32-S3 using Typestate Pattern.
//!
//! # Features
//! - **Pin Safety**: Enforces that a GPIO pin must be switched to Analog mode before it can be configured as a Touch Channel.
//! - **State Safety**: Distinguishes between `Config` (setup) and `Running` (measurement) states. Data reading is only allowed in `Running` state; Configuration is only allowed in `Config` state.

use crate::esp32s3_touch_ll::{self as ll};
use core::marker::PhantomData;

/// Touch Sensor Channels (1-14).
///
/// Channel 0 is reserved for internal denoise and has no physical pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TouchChannel {
    Num1 = 1,
    Num2 = 2,
    Num3 = 3,
    Num4 = 4,
    Num5 = 5,
    Num6 = 6,
    Num7 = 7,
    Num8 = 8,
    Num9 = 9,
    Num10 = 10,
    Num11 = 11,
    Num12 = 12,
    Num13 = 13,
    Num14 = 14,
}

impl TouchChannel {
    #[must_use] 
    pub fn to_u32(self) -> u32 {
        self as u32
    }

    /// On ESP32-S3, Touch Channel N corresponds directly to GPIO N.
    #[must_use] 
    pub fn to_gpio_num(self) -> u32 {
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
// Configurations
// =============================================================================

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
            ll::TouchSensorLL::set_voltage_high(ll::TouchVoltLimHigh::V2_7);
            ll::TouchSensorLL::set_voltage_low(ll::TouchVoltLimLow::V0_5);
            ll::TouchSensorLL::set_voltage_attenuation(ll::TouchVoltAtten::V0_5);

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

    /// Add and configure a channel for measurement.
    ///
    /// **Typestate Enforcement**: This method requires a `TouchPin<pin_state::Analog>`.
    /// You must call `.into_analog()` on a `TouchPin` before passing it here.
    pub fn config_channel(&mut self, pin: &TouchPin<pin_state::Analog>) {
        let ch_num = pin.channel.to_u32();
        unsafe {
            // Set charge voltage and speed
            ll::TouchSensorLL::set_init_charge_voltage(ch_num, ll::TouchInitChargeVolt::High);
            ll::TouchSensorLL::set_charge_speed(ch_num, ll::TouchChargeSpeed::Speed7);
            // Enable in FSM
            ll::TouchSensorLL::enable_channel_mask(1 << ch_num);
        }
    }

    /// Disable a channel.
    pub fn disable_channel(&mut self, channel: TouchChannel) {
        unsafe {
            ll::TouchSensorLL::clear_channel_mask(1 << channel.to_u32());
        }
    }

    /// Set filter configuration.
    pub fn set_filter_config(&mut self, config: &TouchFilterConfig) {
        unsafe {
            ll::TouchSensorLL::set_filter_mode(config.mode);
            ll::TouchSensorLL::set_debounce(config.debounce_cnt);
            ll::TouchSensorLL::set_filter_active_hysteresis(config.noise_thr);
            ll::TouchSensorLL::set_jitter_step(config.jitter_step);
            ll::TouchSensorLL::set_smooth_mode(config.smh_lvl);
        }
    }

    pub fn filter_enable(&mut self) {
        unsafe { ll::TouchSensorLL::filter_enable(true) }
    }

    pub fn filter_disable(&mut self) {
        unsafe { ll::TouchSensorLL::filter_enable(false) }
    }

    /// Set internal denoise configuration.
    pub fn set_denoise_config(&mut self, config: &TouchDenoiseConfig) {
        unsafe {
            ll::TouchSensorLL::denoise_set_resolution(config.grade);
            ll::TouchSensorLL::denoise_set_cap(config.cap_level);
            // Ensure ch0 is cleared from standard scan mask
            ll::TouchSensorLL::clear_channel_mask(1);
            ll::TouchSensorLL::denoise_enable(true);
        }
    }

    /// Configure waterproof/shielding.
    ///
    /// Note: The guard pad must also be initialized as `Analog` via `TouchPin` flow separately,
    /// though this method takes the raw enum for configuration mapping.
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
    pub fn interrupt_enable(&mut self, mask: u32) {
        unsafe { ll::TouchSensorLL::interrupt_enable(mask) }
    }

    /// Disable interrupts.
    pub fn interrupt_disable(&mut self, mask: u32) {
        unsafe { ll::TouchSensorLL::interrupt_disable(mask) }
    }

    /// Clear interrupt flags.
    pub fn interrupt_clear(&mut self, mask: u32) {
        unsafe { ll::TouchSensorLL::interrupt_clear(mask) }
    }

    /// Set touch detection threshold.
    pub fn set_threshold(&mut self, channel: TouchChannel, thresh: u32) {
        unsafe { ll::TouchSensorLL::set_chan_active_threshold(channel.to_u32(), thresh) }
    }

    /// Trigger a single software measurement (One-Shot).
    ///
    /// This is only allowed in `Config` state (when continuous FSM is stopped).
    /// It returns immediately; polling `is_measure_done` is required.
    pub fn trigger_oneshot(&mut self) {
        unsafe { ll::TouchSensorLL::trigger_oneshot_measurement() }
    }

    /// Set the measurement interval (charge/discharge times) for touch channels.
    /// Range: 0 ~ 0xFFFF.
    /// Recommended: Modify to make measurement time around 1ms.
    pub fn set_measurement_interval(&mut self, interval_ticks: u16) {
        unsafe { ll::TouchSensorLL::set_measure_interval_ticks(interval_ticks) }
    }

    /// Set the sleep cycle between measurements.
    /// The timer frequency is `RTC_SLOW_CLK` (approx 150k or 32k).
    /// Range: 0 ~ 0xFF (u8 is sufficient here as per LL but let's accept u16 and cast to be safe or match LL).
    /// LL `set_power_on_wait_cycle` takes `u8`.
    pub fn set_sleep_time(&mut self, sleep_cycle: u8) {
        unsafe { ll::TouchSensorLL::set_power_on_wait_cycle(sleep_cycle) }
    }

    /// Set the reference voltages for the touch sensor.
    pub fn set_voltages(
        &mut self,
        high: TouchVoltLimHigh,
        low: TouchVoltLimLow,
        atten: TouchVoltAtten,
    ) {
        unsafe {
            ll::TouchSensorLL::set_voltage_high(high);
            ll::TouchSensorLL::set_voltage_low(low);
            ll::TouchSensorLL::set_voltage_attenuation(atten);
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
    ///
    /// This is useful for proximity sensing where touch channels are configured
    /// to accumulate measurements over time.
    #[must_use] 
    pub fn get_proximity_count(&self, channel: TouchChannel) -> u32 {
        unsafe { ll::TouchSensorLL::get_proximity_curr_scan_cnt(channel.to_u32()) }
    }

    /// Get interrupt status.
    #[must_use] 
    pub fn get_interrupt_status(&self) -> u32 {
        unsafe { ll::TouchSensorLL::get_intr_status_mask() }
    }

    /// Check if the global 'Scan Done' interrupt flag is set.
    ///
    /// This flag indicates that one complete scan of all enabled channels has finished.
    #[must_use] 
    pub fn is_scan_done(&self) -> bool {
        let status = self.get_interrupt_status();
        (status & ll::TOUCH_LL_INTR_MASK_SCAN_DONE) != 0
    }

    /// Check if the global 'Done' interrupt flag is set.
    ///
    /// This generally indicates a measurement done event for a channel.
    #[must_use] 
    pub fn is_done(&self) -> bool {
        let status = self.get_interrupt_status();
        (status & ll::TOUCH_LL_INTR_MASK_DONE) != 0
    }

    /// Check if the global 'Active' interrupt flag is set.
    ///
    /// This indicates that at least one channel has been activated (touched).
    /// You can check individual channel status via `is_channel_active`.
    #[must_use] 
    pub fn is_active(&self) -> bool {
        let status = self.get_interrupt_status();
        (status & ll::TOUCH_LL_INTR_MASK_ACTIVE) != 0
    }

    /// Check if the global 'Inactive' interrupt flag is set.
    ///
    /// This indicates that at least one channel has been released.
    #[must_use] 
    pub fn is_inactive(&self) -> bool {
        let status = self.get_interrupt_status();
        (status & ll::TOUCH_LL_INTR_MASK_INACTIVE) != 0
    }

    /// Check if the 'Timeout' interrupt flag is set.
    #[must_use] 
    pub fn is_timeout(&self) -> bool {
        let status = self.get_interrupt_status();
        (status & ll::TOUCH_LL_INTR_MASK_TIMEOUT) != 0
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
    pub fn interrupt_clear(&mut self, mask: u32) {
        unsafe { ll::TouchSensorLL::interrupt_clear(mask) }
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

// Re-export useful enums
pub use crate::esp32s3_touch_ll::{
    TOUCH_LL_INTR_MASK_ACTIVE, TOUCH_LL_INTR_MASK_INACTIVE, TOUCH_LL_INTR_MASK_SCAN_DONE,
    TOUCH_LL_INTR_MASK_TIMEOUT, TouchChargeSpeed, TouchDenoiseCap, TouchDenoiseRes,
    TouchFilterMode, TouchInitChargeVolt, TouchSmoothMode, TouchVoltAtten, TouchVoltLimHigh,
    TouchVoltLimLow,
};
