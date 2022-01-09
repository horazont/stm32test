#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::{
	pac, rcc,
	rcc::{Enable, Reset},
};

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
					 // use panic_abort as _; // requires nightly
					 // use panic_itm as _; // logs messages over ITM; requires ITM support
					 // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use core::pin::Pin;

use cortex_m::peripheral::{syst, Peripherals};
// use cortex_m_semihosting::{debug, hprintln};

use stm32test::usart::{WakeSource, WriteExt};
use stm32test::{stm32, stm32_onewire};
use stm32test::{Executor, FromTicks, Monotonic, MonotonicExt, SystickClock};

type Duration = <SystickClock as Monotonic>::Duration;

fn nibble_to_hex(v: u8) -> u8 {
	match v {
		0x0..=0x9 => v + 48,
		0xa..=0xf => v - 10 + 97,
		_ => b'I',
	}
}

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
	use super::*;

	#[local]
	struct Local {
		tx: stm32::Tx<pac::USART1>,
		rx: stm32::Rx<pac::USART1>,
		onewire: stm32_onewire::OneWire<pac::USART3>,
		led1: stm32f1xx_hal::gpio::gpioc::PC9<
			stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
		>,
		led2: stm32f1xx_hal::gpio::gpioc::PC8<
			stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
		>,
	}

	#[shared]
	struct Resources {
		clock: SystickClock,
		clocks: rcc::Clocks,
	}

	#[init]
	fn init(_: init::Context) -> (Resources, Local, init::Monotonics) {
		let mut core = unsafe { Peripherals::steal() };
		core.SYST.set_clock_source(syst::SystClkSource::Core);
		core.SYST.set_reload(8000);
		core.SYST.enable_interrupt();
		core.SYST.enable_counter();

		let mut stm32 = unsafe { stm32f1xx_hal::pac::Peripherals::steal() };
		let mut flash = stm32.FLASH.constrain();
		let mut rcc = stm32.RCC.constrain();
		let clocks = rcc.cfgr.freeze(&mut flash.acr);
		let mut gpioc = stm32.GPIOC.split();
		let led1 = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);
		let led2 = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
		let mut gpioa = stm32.GPIOA.split();
		let mut afio = stm32.AFIO.constrain();

		let serial = {
			let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
			let pin_rx = gpioa.pa10;
			{
				let rcc = unsafe { &(*stm32f1xx_hal::pac::RCC::ptr()) };
				stm32f1xx_hal::pac::DMA1::enable(rcc);
			}
			let ch4 = stm32.DMA1.split().4;
			let mut serial = stm32f1xx_hal::serial::Serial::usart1(
				stm32.USART1,
				(pin_tx, pin_rx),
				&mut afio.mapr,
				stm32f1xx_hal::serial::Config::default().baudrate(115_200.bps()),
				clocks,
			);

			stm32::Serial::wrap(serial.erase())
		};

		let onewire = {
			let pin_tx = gpioc.pc10.into_alternate_open_drain(&mut gpioc.crh);
			let pin_rx = gpioc.pc11.into_pull_up_input(&mut gpioc.crh);

			let mut serial = stm32f1xx_hal::serial::Serial::usart3(
				stm32.USART3,
				(pin_tx, pin_rx),
				&mut afio.mapr,
				stm32f1xx_hal::serial::Config::default().baudrate(300.bps()),
				clocks,
			);

			stm32_onewire::OneWire::new(stm32::Serial::wrap(serial.erase()), clocks)
		};

		let (tx, rx) = serial.split();

		(
			Resources {
				clock: SystickClock::new(),
				clocks,
			},
			Local {
				led1,
				led2,
				tx,
				rx,
				onewire,
			},
			init::Monotonics(),
		)
	}

	#[idle(shared = [&clock, &clocks], local = [led1, led2, tx, rx, onewire])]
	fn idle(cx: idle::Context) -> ! {
		let led1 = cx.local.led1;
		let led2 = cx.local.led2;
		let tx1 = cx.local.tx.clone();
		let tx2 = cx.local.tx;
		let rx = cx.local.rx;
		let clock = cx.shared.clock;
		let clocks = cx.shared.clocks;
		let onewire = cx.local.onewire;
		let mut executor = Executor::new(
			async move {
				led1.set_low();
				clock.sleep_for(Duration::from_ticks(100)).await;
				led1.set_high();
				clock.sleep_for(Duration::from_ticks(100)).await;
				led1.set_low();
				clock.sleep_for(Duration::from_ticks(100)).await;
				loop {
					// tx1.write(&b"1111111111111111\n"[..]).await;
					led1.set_high();
					let mut addr = stm32_onewire::ZERO_ADDR;
					tx2.write(&b"scanning\n"[..]).await;
					loop {
						match onewire.find_next(&mut addr).await {
							Ok(stm32_onewire::OneWireStatus::Presence) => {
								led2.set_high();
								for byte in addr.iter() {
									let hi = (byte & 0xf0) >> 4;
									let lo = byte & 0xf;
									let buf = [nibble_to_hex(hi), nibble_to_hex(lo)];
									tx2.write(&buf[..]).await;
								}
								tx2.write(&b"\n"[..]).await;
								clock.sleep_for(Duration::from_ticks(100)).await;
								led2.set_low();
							}
							Ok(stm32_onewire::OneWireStatus::Empty) => {
								led2.set_high();
								tx2.write(&b"scan done\n"[..]).await;
								clock.sleep_for(Duration::from_ticks(1000)).await;
								led2.set_low();
								break;
							}
							Err(stm32_onewire::SearchError::Vanished) => {
								led1.set_low();
								tx2.write(&b"vanished\n"[..]).await;
								clock.sleep_for(Duration::from_ticks(100)).await;
								led1.set_high();
							}
						}
					}
					clock.sleep_for(Duration::from_ticks(500)).await;
				}
			},
			async move {
				let mut buf: [u8; 1] = [0u8; 1];
				loop {
					/* led2.set_high();
					clock.sleep_for(Duration::from_ticks(1000)).await;
					led2.set_low();
					clock.sleep_for(Duration::from_ticks(1000)).await; */
					rx.read(&mut buf[..]).await;
					tx1.write(&buf[..]).await;
				}
			},
		);
		// SAFETY: we can never ever access the unpinned thing again due to shadowing (this is effectively what pin_mut! in the futures crate does).
		let mut executor = unsafe { Pin::new_unchecked(&mut executor) };
		loop {
			executor.as_mut().advance();
			cortex_m::asm::wfi();
		}
	}

	#[task(binds = SysTick, shared = [&clock])]
	fn systick(cx: systick::Context) {
		cx.shared.clock.tick();
	}

	#[task(binds = USART1)]
	fn usart1(cx: usart1::Context) {
		stm32::usart1_interrupt();
	}

	#[task(binds = USART3)]
	fn usart3(cx: usart3::Context) {
		stm32::usart3_interrupt();
	}
}
