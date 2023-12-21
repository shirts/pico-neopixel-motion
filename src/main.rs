#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Input, Pull, Output};
use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    Common, Config, FifoJoin, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{bind_interrupts, clocks, into_ref, Peripheral, PeripheralRef};
use embassy_time::Timer;
use fixed::types::U24F8;
use fixed_macro::fixed;
use smart_leds::RGB8;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

pub struct Ws2812<'d, P: Instance, const S: usize, const N: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize, const N: usize> Ws2812<'d, P, S, N> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        into_ref!(dma);

        // Setup sm0

        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a: pio::Assembler<32> = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = Config::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    pub async fn write(&mut self, colors: &[RGB8; N], brightness: f32) {
        // Precompute the word bytes from the colors
        let mut words = [0u32; N];

        for i in 0..N {
          let word = (
            ((colors[i].g as f32 * brightness) as u32) << 24
            | ((colors[i].r as f32 * brightness) as u32) << 16
            | ((colors[i].b as f32 * brightness) as u32) << 8
          );

          words[i] = word;
        }

        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;
    }

}

fn brightness_decrease(brightness: &mut f32, step: f32) {
  if *brightness - step <= 0.02 {
    *brightness = 0.02;
  } else {
    *brightness -= step;
  }
}

fn brightness_increase(brightness: &mut f32, step: f32) {
  if *brightness + step >= 1.0 {
    *brightness = 1.0;
  } else {
    *brightness += step;
  }
}

fn wheel(mut wheel_pos: u8) -> RGB8 {
  wheel_pos = 255 - wheel_pos;
  if wheel_pos < 85 {
    return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
  }
  if wheel_pos < 170 {
    wheel_pos -= 85;
    return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
  }
  wheel_pos -= 170;
  (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

const NUM_LEDS: usize = 12;

const LED_BRIGHTNESS_DIM: f32 = 0.02;
const LED_BRIGHTNESS_MAX: f32 = 1.0;
const LED_BRIGHTNESS_STEP: f32 = 0.02;
const LED_BRIGHTNESS_STEP_SLEEP_MS: u64 = 20;
const LED_ON_MINS: u64 = 3;
const LED_ON_MS: u64 = LED_ON_MINS * 1000 * 60;
const LED_PULSE_MS: u64 = 10;

const LED_PULSE_TIMES: u8 = 5;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
  let p = embassy_rp::init(Default::default());

  let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);

  let mut data = [RGB8::default(); NUM_LEDS];

  let motion = Input::new(p.PIN_2, Pull::Down);
  let mut led = Ws2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_16);

  let mut brightness = 0.0;
  let white = RGB8 { r: 255, g: 255, b: 255, };

  // Loop forever making RGB values and pushing them out to the WS2812.
  loop {
    brightness = 0.0;

    if motion.is_high() {
      brightness = LED_BRIGHTNESS_DIM;

      // Turn on LEDs one by one
      for i in 0..NUM_LEDS {
        data[i] = white;

        led.write(&data, brightness).await;
        Timer::after_millis(20).await;
      }

      // Step up brightness
      while brightness < LED_BRIGHTNESS_MAX {
        brightness_increase(&mut brightness, LED_BRIGHTNESS_STEP);

        led.write(&data, brightness).await;
        Timer::after_millis(LED_BRIGHTNESS_STEP_SLEEP_MS).await;
      }

      // Keep LED on for a while
      // Timer::after_millis(LED_ON_MS).await;
      Timer::after_millis(LED_ON_MS).await;

      // Pulse LED
      for _ in 0..LED_PULSE_TIMES {
        while brightness > LED_BRIGHTNESS_DIM {
          brightness_decrease(&mut brightness, LED_BRIGHTNESS_STEP);

          led.write(&data, brightness).await;
          Timer::after_millis(LED_PULSE_MS).await;
        }

        while brightness < LED_BRIGHTNESS_MAX {
          brightness_increase(&mut brightness, LED_BRIGHTNESS_STEP);

          led.write(&data, brightness).await;
          Timer::after_millis(LED_PULSE_MS).await;
        }
      }

      // Motion is not detected, turn off
      // Turn off LEDs one by one
      for i in 0..NUM_LEDS {
        data[i] = RGB8::default();

        led.write(&data, brightness).await;
        Timer::after_millis(LED_BRIGHTNESS_STEP_SLEEP_MS).await;
      }
    }
  }
}
