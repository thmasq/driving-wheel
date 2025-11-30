#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::missing_errors_doc
)]

use defmt::Format;
use esp_hal::Async;
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::{
    gpio::Level,
    rmt::{Channel, ChannelCreator, PulseCode, Tx, TxChannelConfig, TxChannelCreator},
};

const CODE_PERIOD_NS: u32 = 1250;
const T0H_NS: u32 = 400;
const T0L_NS: u32 = CODE_PERIOD_NS - T0H_NS;
const T1H_NS: u32 = 850;
const T1L_NS: u32 = CODE_PERIOD_NS - T1H_NS;

const BUFFER_SIZE: usize = 25;

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

pub struct SmartLed<S> {
    inner: S,
}

/// State: Driver is created but RMT channel is not yet configured.
pub struct Uninitialized<'d, const N: u8, P> {
    creator: ChannelCreator<'d, Async, N>,
    pin: P,
}

/// State: Driver is configured and ready to transmit.
pub struct Ready<'d> {
    channel: Channel<'d, Async, Tx>,
    pulses: (PulseCode, PulseCode),
    buffer: [PulseCode; BUFFER_SIZE],
}

impl<'d, const N: u8, P> SmartLed<Uninitialized<'d, N, P>>
where
    P: PeripheralOutput<'d> + 'd,
    ChannelCreator<'d, Async, N>: TxChannelCreator<'d, Async>,
{
    pub fn new(creator: ChannelCreator<'d, Async, N>, pin: P) -> Self {
        Self {
            inner: Uninitialized { creator, pin },
        }
    }

    pub fn initialize(
        self,
        config: TxChannelConfig,
        src_clock_mhz: u32,
    ) -> Result<SmartLed<Ready<'d>>, esp_hal::rmt::Error> {
        let (p0, p1) = Self::led_pulses_for_clock(src_clock_mhz);

        let channel = self.inner.creator.configure_tx(self.inner.pin, config)?;

        Ok(SmartLed {
            inner: Ready {
                channel,
                pulses: (p0, p1),
                buffer: [PulseCode::default(); BUFFER_SIZE],
            },
        })
    }

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
}

impl SmartLed<Ready<'_>> {
    pub async fn write(&mut self, color: RGB8) -> Result<(), esp_hal::rmt::Error> {
        self.encode(color);
        self.inner.channel.transmit(&self.inner.buffer).await
    }

    fn encode(&mut self, color: RGB8) {
        let (p0, p1) = self.inner.pulses;
        let bytes = [color.g, color.r, color.b];
        let mut idx = 0;
        for &byte in &bytes {
            for bit in (0..8).rev() {
                let is_set = (byte & (1 << bit)) != 0;
                self.inner.buffer[idx] = if is_set { p1 } else { p0 };
                idx += 1;
            }
        }
        self.inner.buffer[24] = PulseCode::new(Level::Low, 0, Level::Low, 0);
    }
}
