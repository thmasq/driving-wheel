//! Low-level, unsafe Touch Sensor driver for ESP32-S3.
//! Feature-complete mapping of esp-idf/components/hal/esp32s3/include/hal/touch_sensor_ll.h

#![allow(dead_code)]
#![allow(non_camel_case_types)]

use core::ptr::{read_volatile, write_volatile};

// Register Base Addresses
pub const DR_REG_RTCCNTL_BASE: u32 = 0x6000_8000;
pub const DR_REG_SENS_BASE: u32 = 0x6000_8800;
pub const DR_REG_RTCIO_BASE: u32 = 0x6000_8400;

pub const RTC_CNTL_INT_ENA_REG: u32 = DR_REG_RTCCNTL_BASE + 0x40;
pub const RTC_CNTL_INT_ST_REG: u32 = DR_REG_RTCCNTL_BASE + 0x48;
pub const RTC_CNTL_INT_CLR_REG: u32 = DR_REG_RTCCNTL_BASE + 0x4C;
pub const RTC_CNTL_TOUCH_CTRL1_REG: u32 = DR_REG_RTCCNTL_BASE + 0x108;
pub const RTC_CNTL_TOUCH_CTRL2_REG: u32 = DR_REG_RTCCNTL_BASE + 0x10C;
pub const RTC_CNTL_TOUCH_SCAN_CTRL_REG: u32 = DR_REG_RTCCNTL_BASE + 0x110;
pub const RTC_CNTL_TOUCH_FILTER_CTRL_REG: u32 = DR_REG_RTCCNTL_BASE + 0x11C;
pub const RTC_CNTL_TOUCH_TIMEOUT_CTRL_REG: u32 = DR_REG_RTCCNTL_BASE + 0x124;
pub const RTC_CNTL_TOUCH_APPROACH_REG: u32 = DR_REG_RTCCNTL_BASE + 0x118;
pub const RTC_CNTL_TOUCH_DAC_REG: u32 = DR_REG_RTCCNTL_BASE + 0x14C;
pub const RTC_CNTL_TOUCH_DAC1_REG: u32 = DR_REG_RTCCNTL_BASE + 0x150;

pub const SENS_SAR_TOUCH_CONF_REG: u32 = DR_REG_SENS_BASE + 0x5C;
pub const SENS_SAR_TOUCH_DENOISE_REG: u32 = DR_REG_SENS_BASE + 0x60;
pub const SENS_SAR_TOUCH_THRES1_REG: u32 = DR_REG_SENS_BASE + 0x64;
pub const SENS_SAR_TOUCH_CHN_ST_REG: u32 = DR_REG_SENS_BASE + 0x9C;
pub const SENS_SAR_TOUCH_STATUS0_REG: u32 = DR_REG_SENS_BASE + 0xA0;
pub const SENS_SAR_TOUCH_APPR_STATUS_REG: u32 = DR_REG_SENS_BASE + 0xE0;
pub const SENS_SAR_TOUCH_SLP_STATUS_REG: u32 = DR_REG_SENS_BASE + 0xDC;

pub const RTC_IO_TOUCH_PAD0_REG: u32 = DR_REG_RTCIO_BASE + 0x84;

// Interrupt Masks
pub const TOUCH_LL_INTR_MASK_SCAN_DONE: u32 = 1 << 4;
pub const TOUCH_LL_INTR_MASK_DONE: u32 = 1 << 6;
pub const TOUCH_LL_INTR_MASK_ACTIVE: u32 = 1 << 7;
pub const TOUCH_LL_INTR_MASK_INACTIVE: u32 = 1 << 8;
pub const TOUCH_LL_INTR_MASK_TIMEOUT: u32 = 1 << 18;
pub const TOUCH_LL_INTR_MASK_PROX_DONE: u32 = 1 << 20;
pub const TOUCH_LL_INTR_MASK_ALL: u32 = TOUCH_LL_INTR_MASK_SCAN_DONE
    | TOUCH_LL_INTR_MASK_DONE
    | TOUCH_LL_INTR_MASK_ACTIVE
    | TOUCH_LL_INTR_MASK_INACTIVE
    | TOUCH_LL_INTR_MASK_TIMEOUT
    | TOUCH_LL_INTR_MASK_PROX_DONE;

// Register Field Masks
pub const RTC_CNTL_TOUCH_CLKGATE_EN: u32 = 1 << 31;
pub const RTC_CNTL_TOUCH_RESET: u32 = 1 << 29;
pub const RTC_CNTL_TOUCH_TIMER_FORCE_DONE: u32 = 3 << 27;
pub const RTC_CNTL_TOUCH_START_FORCE: u32 = 1 << 16; // 0=Timer, 1=SW
pub const RTC_CNTL_TOUCH_START_EN: u32 = 1 << 15;
pub const RTC_CNTL_TOUCH_SLP_TIMER_EN: u32 = 1 << 13;
pub const RTC_CNTL_TOUCH_DBIAS: u32 = 1 << 17; // Bit 17 in CTRL2

pub const RTC_CNTL_TOUCH_TIMEOUT_EN: u32 = 1 << 22; // Bit 22 in TIMEOUT_CTRL

pub const RTC_CNTL_TOUCH_DENOISE_EN: u32 = 1 << 2; // Bit 2 in SCAN_CTRL
pub const RTC_CNTL_TOUCH_SHIELD_PAD_EN: u32 = 1 << 9; // Bit 9 in SCAN_CTRL
pub const RTC_CNTL_TOUCH_INACTIVE_CONNECTION: u32 = 1 << 8; // Bit 8 in SCAN_CTRL

// Filter Control Register Fields
pub const RTC_CNTL_TOUCH_FILTER_EN: u32 = 1 << 31;
pub const RTC_CNTL_TOUCH_CONFIG3: u32 = 3 << 23; // Active Hysteresis [24:23]
pub const RTC_CNTL_TOUCH_NOISE_THRES: u32 = 3 << 21; // [22:21]
pub const RTC_CNTL_TOUCH_CONFIG2: u32 = 3 << 19; // Noise Thresh duplicate [20:19]
pub const RTC_CNTL_TOUCH_CONFIG1: u32 = 0xF << 15; // Jitter / Config [18:15]
pub const RTC_CNTL_TOUCH_BYPASS_NOISE_THRES: u32 = 1 << 8;
pub const RTC_CNTL_TOUCH_BYPASS_NN_THRES: u32 = 1 << 7;

// Approach Register Fields
pub const RTC_CNTL_TOUCH_SLP_CHANNEL_CLR: u32 = 1 << 23;

pub const SENS_TOUCH_MEAS_DONE: u32 = 1 << 31;
pub const SENS_TOUCH_STATUS_CLR: u32 = 1 << 15; // Write 1 to clear status
pub const SENS_TOUCH_ACTIVE_MASK: u32 = 0x7FFF; // Lower 15 bits

pub const RTC_IO_TOUCH_PAD_XPD: u32 = 1 << 31;
pub const RTC_IO_TOUCH_PAD_TIE_OPT: u32 = 1 << 30;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchFilterMode {
    Iir4 = 0,
    Iir8 = 1,
    Iir16 = 2,
    Iir32 = 3,
    Iir64 = 4,
    Iir128 = 5,
    Iir256 = 6,
    Jitter = 7,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchSmoothMode {
    Off = 0,
    Iir2 = 1,
    Iir4 = 2,
    Iir8 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchChargeSpeed {
    Speed0 = 0,
    Speed1 = 1,
    Speed2 = 2,
    Speed3 = 3,
    Speed4 = 4,
    Speed5 = 5,
    Speed6 = 6,
    Speed7 = 7,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchIdleConn {
    HighZ = 0,
    Gnd = 1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchVoltLimHigh {
    V2_4 = 0,
    V2_5 = 1,
    V2_6 = 2,
    V2_7 = 3,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchVoltLimLow {
    V0_5 = 0,
    V0_6 = 1,
    V0_7 = 2,
    V0_8 = 3,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchVoltAtten {
    V1_5 = 0,
    V1_0 = 1,
    V0_5 = 2,
    V0_0 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchInitChargeVolt {
    Low = 0,
    High = 1,
    Float = 0xFF,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchDenoiseCap {
    Cap5pf = 0,
    Cap6_4pf = 1,
    Cap7_8pf = 2,
    Cap9_2pf = 3,
    Cap10_6pf = 4,
    Cap12_0pf = 5,
    Cap13_4pf = 6,
    Cap14_8pf = 7,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchDenoiseRes {
    Bit12 = 0,
    Bit10 = 1,
    Bit8 = 2,
    Bit4 = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TouchBiasType {
    SelfBias = 0,
    BandgapBias = 1,
}

pub struct TouchSensorLL;

impl TouchSensorLL {
    // -------------------------------------------------------------------------
    // Core Control & Interrupts
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn enable_clock_gate(enable: bool) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if enable {
            val |= RTC_CNTL_TOUCH_CLKGATE_EN;
        } else {
            val &= !RTC_CNTL_TOUCH_CLKGATE_EN;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn reset_module() {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val |= RTC_CNTL_TOUCH_RESET;
        unsafe { write_volatile(addr, val) };
        val &= !RTC_CNTL_TOUCH_RESET;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn interrupt_enable(mask: u32) {
        let addr = RTC_CNTL_INT_ENA_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val |= mask & TOUCH_LL_INTR_MASK_ALL;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn interrupt_disable(mask: u32) {
        let addr = RTC_CNTL_INT_ENA_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(mask & TOUCH_LL_INTR_MASK_ALL);
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn interrupt_clear(mask: u32) {
        unsafe {
            write_volatile(
                RTC_CNTL_INT_CLR_REG as *mut u32,
                mask & TOUCH_LL_INTR_MASK_ALL,
            );
        };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_intr_status_mask() -> u32 {
        (unsafe { read_volatile(RTC_CNTL_INT_ST_REG as *const u32) }) & TOUCH_LL_INTR_MASK_ALL
    }

    // -------------------------------------------------------------------------
    // Status Info
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_current_meas_channel() -> u32 {
        let val = unsafe { read_volatile(SENS_SAR_TOUCH_STATUS0_REG as *const u32) };
        (val >> 22) & 0xF // TOUCH_SCAN_CURR
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn is_measure_done() -> bool {
        let val = unsafe { read_volatile(SENS_SAR_TOUCH_CHN_ST_REG as *const u32) };
        (val & SENS_TOUCH_MEAS_DONE) != 0
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_active_channel_mask() -> u32 {
        let val = unsafe { read_volatile(SENS_SAR_TOUCH_CHN_ST_REG as *const u32) };
        val & SENS_TOUCH_ACTIVE_MASK
    }

    // -------------------------------------------------------------------------
    // Measurement Config
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn enable_scan_mask(chan_mask: u16, enable: bool) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        let mask = (u32::from(chan_mask) & 0x7FFF) << 10; // TOUCH_SCAN_PAD_MAP
        if enable {
            val |= mask;
        } else {
            val &= !mask;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_chan_active_threshold(touch_num: u32, thresh: u32) {
        if touch_num == 0 || touch_num > 14 {
            return;
        }
        let addr = SENS_SAR_TOUCH_THRES1_REG + (touch_num - 1) * 4;
        unsafe { write_volatile(addr as *mut u32, thresh & 0x003F_FFFF) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_chan_active_threshold(touch_num: u32) -> u32 {
        if touch_num == 0 || touch_num > 14 {
            return 0;
        }
        let addr = SENS_SAR_TOUCH_THRES1_REG + (touch_num - 1) * 4;
        (unsafe { read_volatile(addr as *const u32) }) & 0x003F_FFFF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_power_on_wait_cycle(wait_cycles: u8) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !0xFF; // TOUCH_XPD_WAIT
        val |= u32::from(wait_cycles);
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_power_on_wait_cycle() -> u8 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        (val & 0xFF) as u8
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_charge_times(charge_times: u16) {
        let addr = RTC_CNTL_TOUCH_CTRL1_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0xFFFF << 16);
        val |= u32::from(charge_times) << 16;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_charge_times() -> u16 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL1_REG as *const u32) };
        ((val >> 16) & 0xFFFF) as u16
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_measure_interval_ticks(interval: u16) {
        let addr = RTC_CNTL_TOUCH_CTRL1_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !0xFFFF;
        val |= u32::from(interval);
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_measure_interval_ticks() -> u16 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL1_REG as *const u32) };
        (val & 0xFFFF) as u16
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_charge_speed(touch_num: u32, speed: TouchChargeSpeed) {
        if touch_num > 14 {
            return;
        }
        let (reg, shift) = if touch_num < 10 {
            (RTC_CNTL_TOUCH_DAC_REG, 29 - (touch_num * 3))
        } else {
            (RTC_CNTL_TOUCH_DAC1_REG, 29 - ((touch_num - 10) * 3))
        };
        let addr = reg as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x7 << shift);
        val |= (speed as u32) << shift;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_charge_speed(touch_num: u32) -> TouchChargeSpeed {
        if touch_num > 14 {
            return TouchChargeSpeed::Speed0; // Default or Error
        }
        let (reg, shift) = if touch_num < 10 {
            (RTC_CNTL_TOUCH_DAC_REG, 29 - (touch_num * 3))
        } else {
            (RTC_CNTL_TOUCH_DAC1_REG, 29 - ((touch_num - 10) * 3))
        };
        let val = unsafe { read_volatile(reg as *const u32) };
        let speed_val = (val >> shift) & 0x7;
        #[allow(clippy::match_same_arms)]
        match speed_val {
            0 => TouchChargeSpeed::Speed0,
            1 => TouchChargeSpeed::Speed1,
            2 => TouchChargeSpeed::Speed2,
            3 => TouchChargeSpeed::Speed3,
            4 => TouchChargeSpeed::Speed4,
            5 => TouchChargeSpeed::Speed5,
            6 => TouchChargeSpeed::Speed6,
            7 => TouchChargeSpeed::Speed7,
            _ => TouchChargeSpeed::Speed0,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_voltage_high(limit: TouchVoltLimHigh) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x3 << 6);
        val |= (limit as u32) << 6;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_voltage_high() -> TouchVoltLimHigh {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        #[allow(clippy::match_same_arms)]
        match (val >> 6) & 0x3 {
            0 => TouchVoltLimHigh::V2_4,
            1 => TouchVoltLimHigh::V2_5,
            2 => TouchVoltLimHigh::V2_6,
            3 => TouchVoltLimHigh::V2_7,
            _ => TouchVoltLimHigh::V2_4,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_voltage_low(limit: TouchVoltLimLow) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x3 << 4);
        val |= (limit as u32) << 4;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_voltage_low() -> TouchVoltLimLow {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        #[allow(clippy::match_same_arms)]
        match (val >> 4) & 0x3 {
            0 => TouchVoltLimLow::V0_5,
            1 => TouchVoltLimLow::V0_6,
            2 => TouchVoltLimLow::V0_7,
            3 => TouchVoltLimLow::V0_8,
            _ => TouchVoltLimLow::V0_5,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_voltage_attenuation(atten: TouchVoltAtten) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x3 << 2);
        val |= (atten as u32) << 2;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_voltage_attenuation() -> TouchVoltAtten {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        #[allow(clippy::match_same_arms)]
        match (val >> 2) & 0x3 {
            0 => TouchVoltAtten::V1_5,
            1 => TouchVoltAtten::V1_0,
            2 => TouchVoltAtten::V0_5,
            3 => TouchVoltAtten::V0_0,
            _ => TouchVoltAtten::V1_5,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_init_charge_voltage(touch_num: u32, init_volt: TouchInitChargeVolt) {
        if touch_num > 14 {
            return;
        }
        let addr = (RTC_IO_TOUCH_PAD0_REG + (touch_num * 4)) as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        match init_volt {
            TouchInitChargeVolt::Float => val &= !RTC_IO_TOUCH_PAD_XPD,
            TouchInitChargeVolt::High => {
                val |= RTC_IO_TOUCH_PAD_XPD | RTC_IO_TOUCH_PAD_TIE_OPT;
            }
            TouchInitChargeVolt::Low => {
                val |= RTC_IO_TOUCH_PAD_XPD;
                val &= !RTC_IO_TOUCH_PAD_TIE_OPT;
            }
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_idle_channel_connection(conn: TouchIdleConn) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if conn == TouchIdleConn::Gnd {
            val |= RTC_CNTL_TOUCH_INACTIVE_CONNECTION;
        } else {
            val &= !RTC_CNTL_TOUCH_INACTIVE_CONNECTION;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn enable_channel_mask(enable_mask: u16) {
        // Enable in SENS (OUTEN)
        let sens_addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut sens_val = unsafe { read_volatile(sens_addr) };
        sens_val |= u32::from(enable_mask) & 0x7FFF;
        unsafe { write_volatile(sens_addr, sens_val) };

        // Enable in RTC (PAD_MAP)
        let rtc_addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut rtc_val = unsafe { read_volatile(rtc_addr) };
        rtc_val |= (u32::from(enable_mask) & 0x7FFF) << 10;
        unsafe { write_volatile(rtc_addr, rtc_val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn clear_channel_mask(disable_mask: u16) {
        let sens_addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut sens_val = unsafe { read_volatile(sens_addr) };
        sens_val &= !(u32::from(disable_mask) & 0x7FFF);
        unsafe { write_volatile(sens_addr, sens_val) };

        let rtc_addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut rtc_val = unsafe { read_volatile(rtc_addr) };
        rtc_val &= !((u32::from(disable_mask) & 0x7FFF) << 10);
        unsafe { write_volatile(rtc_addr, rtc_val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_timeout(timeout_cycles: u32) {
        let addr = RTC_CNTL_TOUCH_TIMEOUT_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if timeout_cycles > 0 {
            val &= !0x003F_FFFF; // 22 bits
            val |= timeout_cycles & 0x003F_FFFF;
            val |= RTC_CNTL_TOUCH_TIMEOUT_EN;
        } else {
            val &= !RTC_CNTL_TOUCH_TIMEOUT_EN;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_timeout() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_TIMEOUT_CTRL_REG as *const u32) };
        if (val & RTC_CNTL_TOUCH_TIMEOUT_EN) != 0 {
            val & 0x003F_FFFF
        } else {
            0
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn clear_active_channel_status() {
        let addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val |= SENS_TOUCH_STATUS_CLR;
        unsafe { write_volatile(addr, val) };
        val &= !SENS_TOUCH_STATUS_CLR;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_bias_type(bias_type: TouchBiasType) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if bias_type == TouchBiasType::BandgapBias {
            val &= !RTC_CNTL_TOUCH_DBIAS; // 0 for Bandgap
        } else {
            val |= RTC_CNTL_TOUCH_DBIAS; // 1 for Self
        }
        unsafe { write_volatile(addr, val) };
    }

    // -------------------------------------------------------------------------
    // FSM Operation
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn force_done_curr_measurement() {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val |= RTC_CNTL_TOUCH_TIMER_FORCE_DONE;
        unsafe { write_volatile(addr, val) };
        val &= !RTC_CNTL_TOUCH_TIMER_FORCE_DONE;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn enable_fsm_timer(enable: bool) {
        unsafe { Self::force_done_curr_measurement() };
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        // TOUCH_START_FORCE: 0 = Timer, 1 = Software
        if enable {
            val &= !RTC_CNTL_TOUCH_START_FORCE;
        } else {
            val |= RTC_CNTL_TOUCH_START_FORCE;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn start_fsm_repeated_timer() {
        unsafe { Self::force_done_curr_measurement() };
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !RTC_CNTL_TOUCH_START_FORCE; // Ensure timer mode
        val |= RTC_CNTL_TOUCH_SLP_TIMER_EN;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn stop_fsm_repeated_timer() {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !RTC_CNTL_TOUCH_SLP_TIMER_EN;
        unsafe { write_volatile(addr, val) };
        unsafe { Self::force_done_curr_measurement() };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn is_fsm_repeated_timer_enabled() -> bool {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        (val & RTC_CNTL_TOUCH_SLP_TIMER_EN) != 0
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn trigger_oneshot_measurement() {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val |= RTC_CNTL_TOUCH_START_EN;
        unsafe { write_volatile(addr, val) };
        val &= !RTC_CNTL_TOUCH_START_EN;
        unsafe { write_volatile(addr, val) };
    }

    // -------------------------------------------------------------------------
    // Benchmark / Data
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn reset_benchmark(touch_num: u32) {
        let addr = SENS_SAR_TOUCH_CHN_ST_REG as *mut u32;
        // Shift determined by user: 1 << (14 + touch_num)
        // This corresponds to TOUCH_CHANNEL_CLR in SENS_SAR_TOUCH_CHN_ST_REG
        let bit = if touch_num > 0 && touch_num <= 14 {
            1 << (14 + touch_num)
        } else {
            0
        };
        if bit != 0 {
            unsafe { write_volatile(addr, bit) };
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn sleep_reset_benchmark() {
        let addr = RTC_CNTL_TOUCH_APPROACH_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val |= RTC_CNTL_TOUCH_SLP_CHANNEL_CLR;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn read_raw_data(touch_num: u32) -> u32 {
        if touch_num == 0 || touch_num > 14 {
            return 0;
        }
        // Select RAW (0)
        let conf_addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut conf = unsafe { read_volatile(conf_addr) };
        conf &= !(3 << 16);
        unsafe { write_volatile(conf_addr, conf) };

        let status_addr = SENS_SAR_TOUCH_STATUS0_REG + (touch_num - 1) * 4;
        (unsafe { read_volatile(status_addr as *const u32) }) & 0x003F_FFFF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn read_benchmark(touch_num: u32) -> u32 {
        if touch_num == 0 || touch_num > 14 {
            return 0;
        }
        // Select Benchmark (2)
        let conf_addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut conf = unsafe { read_volatile(conf_addr) };
        conf &= !(3 << 16);
        conf |= 2 << 16;
        unsafe { write_volatile(conf_addr, conf) };

        let status_addr = SENS_SAR_TOUCH_STATUS0_REG + (touch_num - 1) * 4;
        (unsafe { read_volatile(status_addr as *const u32) }) & 0x003F_FFFF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn read_smooth_data(touch_num: u32) -> u32 {
        if touch_num == 0 || touch_num > 14 {
            return 0;
        }
        // Select Smooth (3)
        let conf_addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut conf = unsafe { read_volatile(conf_addr) };
        conf &= !(3 << 16);
        conf |= 3 << 16;
        unsafe { write_volatile(conf_addr, conf) };

        let status_addr = SENS_SAR_TOUCH_STATUS0_REG + (touch_num - 1) * 4;
        (unsafe { read_volatile(status_addr as *const u32) }) & 0x003F_FFFF
    }

    // -------------------------------------------------------------------------
    // Filter Configuration
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn filter_enable(enable: bool) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if enable {
            val |= RTC_CNTL_TOUCH_FILTER_EN;
        } else {
            val &= !RTC_CNTL_TOUCH_FILTER_EN;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_filter_mode(mode: TouchFilterMode) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x7 << 28);
        val |= (mode as u32) << 28;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_jitter_step(step: u32) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0xF << 11);
        val |= (step & 0xF) << 11;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_smooth_mode(mode: TouchSmoothMode) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x3 << 9);
        val |= (mode as u32) << 9;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_debounce(debounce: u32) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x7 << 25);
        val |= (debounce & 0x7) << 25;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_denoise_level(denoise_lvl: u8) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };

        let always_update = denoise_lvl == 0;
        // Map denoise level to coefficients: 1->2 (1/4), 2->1 (3/8), 3->0 (1/2), 4->3 (1)
        #[allow(clippy::match_same_arms)]
        let noise_thresh = match denoise_lvl {
            1 => 2,
            2 => 1,
            3 => 0,
            4 => 3,
            _ => 0,
        };

        if always_update {
            val |= RTC_CNTL_TOUCH_BYPASS_NOISE_THRES;
            val |= RTC_CNTL_TOUCH_BYPASS_NN_THRES;
            // Clean specific fields if bypassed
            val &= !RTC_CNTL_TOUCH_NOISE_THRES;
            val &= !RTC_CNTL_TOUCH_CONFIG2;
        } else {
            val &= !RTC_CNTL_TOUCH_BYPASS_NOISE_THRES;
            val &= !RTC_CNTL_TOUCH_BYPASS_NN_THRES;

            // Set TOUCH_NOISE_THRES (bits 21:22)
            val &= !RTC_CNTL_TOUCH_NOISE_THRES;
            val |= (noise_thresh & 0x3) << 21;

            // Set CONFIG2 (bits 19:20)
            val &= !RTC_CNTL_TOUCH_CONFIG2;
            val |= (noise_thresh & 0x3) << 19;
        }

        // Set CONFIG1 (bits 15:18) to 0xF as per IDF
        val |= RTC_CNTL_TOUCH_CONFIG1;

        val &= !RTC_CNTL_TOUCH_CONFIG3;
        val |= (2 & 0x3) << 23;

        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_filter_active_hysteresis(hysteresis: u32) {
        let addr = RTC_CNTL_TOUCH_FILTER_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        // CONFIG3 is bits 23:24
        val &= !RTC_CNTL_TOUCH_CONFIG3;
        val |= (hysteresis & 0x3) << 23;
        unsafe { write_volatile(addr, val) };
    }

    // -------------------------------------------------------------------------
    // Waterproof & Proximity
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn waterproof_enable(enable: bool) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if enable {
            val |= RTC_CNTL_TOUCH_SHIELD_PAD_EN;
        } else {
            val &= !RTC_CNTL_TOUCH_SHIELD_PAD_EN;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_waterproof_guard_chan(pad_num: u32) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0xF << 28);
        val |= (pad_num & 0xF) << 28;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_shield_driver(driver_level: u32) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x7 << 25);
        val |= (driver_level & 0x7) << 25;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_proximity_sensing_channel(prox_chan: u8, touch_num: u32) {
        let addr = SENS_SAR_TOUCH_CONF_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        let shift = match prox_chan {
            0 => 28,
            1 => 24,
            2 => 20,
            _ => return,
        };
        val &= !(0xF << shift);
        val |= (touch_num & 0xF) << shift;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn set_proximity_scan_times(times: u32) {
        let addr = RTC_CNTL_TOUCH_APPROACH_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0xFF << 24);
        val |= (times & 0xFF) << 24;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_proximity_total_scan_times() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_APPROACH_REG as *const u32) };
        (val >> 24) & 0xFF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_proximity_curr_scan_cnt(touch_num: u32) -> u32 {
        // Read configuration to see which Proximity Pad Slot (0,1,2) the touch_num is assigned to
        let conf_val = unsafe { read_volatile(SENS_SAR_TOUCH_CONF_REG as *const u32) };
        let pad0 = (conf_val >> 28) & 0xF;
        let pad1 = (conf_val >> 24) & 0xF;
        let pad2 = (conf_val >> 20) & 0xF;

        let status_val = unsafe { read_volatile(SENS_SAR_TOUCH_APPR_STATUS_REG as *const u32) };

        if touch_num == pad0 {
            (status_val >> 16) & 0xFF
        } else if touch_num == pad1 {
            (status_val >> 8) & 0xFF
        } else if touch_num == pad2 {
            status_val & 0xFF
        } else {
            0
        }
    }

    // -------------------------------------------------------------------------
    // Denoise Channel
    // -------------------------------------------------------------------------

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn denoise_enable(enable: bool) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        if enable {
            val |= RTC_CNTL_TOUCH_DENOISE_EN;
        } else {
            val &= !RTC_CNTL_TOUCH_DENOISE_EN;
        }
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn denoise_set_cap(cap: TouchDenoiseCap) {
        let addr = RTC_CNTL_TOUCH_CTRL2_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !(0x7 << 9);
        val |= (cap as u32) << 9;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn denoise_set_resolution(res: TouchDenoiseRes) {
        let addr = RTC_CNTL_TOUCH_SCAN_CTRL_REG as *mut u32;
        let mut val = unsafe { read_volatile(addr) };
        val &= !0x3;
        val |= res as u32;
        unsafe { write_volatile(addr, val) };
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn denoise_read_data() -> u32 {
        (unsafe { read_volatile(SENS_SAR_TOUCH_DENOISE_REG as *const u32) }) & 0x003F_FFFF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn proximity_pad_check(touch_num: u32) -> bool {
        let conf_val = unsafe { read_volatile(SENS_SAR_TOUCH_CONF_REG as *const u32) };

        // Extract the three proximity pads from SENS_SAR_TOUCH_CONF_REG
        let pad0 = (conf_val >> 28) & 0xF;
        let pad1 = (conf_val >> 24) & 0xF;
        let pad2 = (conf_val >> 20) & 0xF;

        (touch_num == pad0) || (touch_num == pad1) || (touch_num == pad2)
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_filter_mode() -> TouchFilterMode {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_FILTER_CTRL_REG as *const u32) };
        let mode = (val >> 28) & 0x7;
        #[allow(clippy::match_same_arms)]
        match mode {
            0 => TouchFilterMode::Iir4,
            1 => TouchFilterMode::Iir8,
            2 => TouchFilterMode::Iir16,
            3 => TouchFilterMode::Iir32,
            4 => TouchFilterMode::Iir64,
            5 => TouchFilterMode::Iir128,
            6 => TouchFilterMode::Iir256,
            7 => TouchFilterMode::Jitter,
            _ => TouchFilterMode::Iir4,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_smooth_mode() -> TouchSmoothMode {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_FILTER_CTRL_REG as *const u32) };
        let mode = (val >> 9) & 0x3;
        #[allow(clippy::match_same_arms)]
        match mode {
            0 => TouchSmoothMode::Off,
            1 => TouchSmoothMode::Iir2,
            2 => TouchSmoothMode::Iir4,
            3 => TouchSmoothMode::Iir8,
            _ => TouchSmoothMode::Off,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_debounce() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_FILTER_CTRL_REG as *const u32) };
        (val >> 25) & 0x7
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_noise_threshold() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_FILTER_CTRL_REG as *const u32) };
        (val >> 21) & 0x3
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_jitter_step() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_FILTER_CTRL_REG as *const u32) };
        (val >> 11) & 0xF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn denoise_get_cap() -> TouchDenoiseCap {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        let cap = (val >> 9) & 0x7;
        #[allow(clippy::match_same_arms)]
        match cap {
            0 => TouchDenoiseCap::Cap5pf,
            1 => TouchDenoiseCap::Cap6_4pf,
            2 => TouchDenoiseCap::Cap7_8pf,
            3 => TouchDenoiseCap::Cap9_2pf,
            4 => TouchDenoiseCap::Cap10_6pf,
            5 => TouchDenoiseCap::Cap12_0pf,
            6 => TouchDenoiseCap::Cap13_4pf,
            7 => TouchDenoiseCap::Cap14_8pf,
            _ => TouchDenoiseCap::Cap5pf,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn denoise_get_resolution() -> TouchDenoiseRes {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_SCAN_CTRL_REG as *const u32) };
        let res = val & 0x3;
        #[allow(clippy::match_same_arms)]
        match res {
            0 => TouchDenoiseRes::Bit12,
            1 => TouchDenoiseRes::Bit10,
            2 => TouchDenoiseRes::Bit8,
            3 => TouchDenoiseRes::Bit4,
            _ => TouchDenoiseRes::Bit12,
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn waterproof_get_guard_chan() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_SCAN_CTRL_REG as *const u32) };
        (val >> 28) & 0xF
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn waterproof_get_shield_driver() -> u32 {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_SCAN_CTRL_REG as *const u32) };
        (val >> 25) & 0x7
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_clock_gate_state() -> bool {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        (val & RTC_CNTL_TOUCH_CLKGATE_EN) != 0
    }

    /// Returns true if FSM is in Software Trigger Mode, false if Timer Trigger Mode.
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub unsafe fn get_fsm_mode() -> bool {
        let val = unsafe { read_volatile(RTC_CNTL_TOUCH_CTRL2_REG as *const u32) };
        (val & RTC_CNTL_TOUCH_START_FORCE) != 0
    }
}
