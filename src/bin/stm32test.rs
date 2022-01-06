#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::{
	pac,
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

use stm32test::stm32;
use stm32test::usart::{WakeSource, WriteExt};
use stm32test::{Executor, FromTicks, Monotonic, MonotonicExt, SystickClock};

type Duration = <SystickClock as Monotonic>::Duration;

#[rtic::app(device = stm32f1xx_hal::pac)]
const APP: () = {
	struct Resources {
		clock: SystickClock,
		tx: stm32::Tx<pac::USART1>,
		rx: stm32::Rx<pac::USART1>,
		led1: stm32f1xx_hal::gpio::gpioc::PC9<
			stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
		>,
		led2: stm32f1xx_hal::gpio::gpioc::PC8<
			stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>,
		>,
	}

	#[init]
	fn init(_: init::Context) -> init::LateResources {
		let mut core = unsafe { Peripherals::steal() };
		core.SYST.set_clock_source(syst::SystClkSource::Core);
		core.SYST.set_reload(8000);
		core.SYST.enable_interrupt();
		core.SYST.enable_counter();

		let mut stm32 = unsafe { stm32f1xx_hal::pac::Peripherals::steal() };
		let mut flash = stm32.FLASH.constrain();
		let mut rcc = stm32.RCC.constrain();
		let clocks = rcc.cfgr.freeze(&mut flash.acr);
		let mut gpioc = stm32.GPIOC.split(&mut rcc.apb2);
		let led1 = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);
		let led2 = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
		let mut gpioa = stm32.GPIOA.split(&mut rcc.apb2);
		let mut afio = stm32.AFIO.constrain(&mut rcc.apb2);
		let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
		let pin_rx = gpioa.pa10;
		{
			// let rcc = unsafe { &(*stm32f1xx_hal::pac::RCC::ptr()) };
			stm32f1xx_hal::pac::DMA1::enable(&mut rcc.ahb);
		}
		let ch4 = stm32.DMA1.split(&mut rcc.ahb).4;
		let serial = stm32f1xx_hal::serial::Serial::usart1(
			stm32.USART1,
			(pin_tx, pin_rx),
			&mut afio.mapr,
			stm32f1xx_hal::serial::Config::default().baudrate(300.bps()),
			clocks,
			&mut rcc.apb2,
		);

		let (tx, rx) = stm32::Serial::wrap(serial).split();

		init::LateResources {
			clock: SystickClock::new(),
			led1,
			led2,
			tx,
			rx,
		}
	}

	#[idle(resources = [&clock, led1, led2, tx, rx])]
	fn idle(cx: idle::Context) -> ! {
		let led1 = cx.resources.led1;
		let led2 = cx.resources.led2;
		let tx = cx.resources.tx;
		let rx = cx.resources.rx;
		let clock = cx.resources.clock;
		let mut executor = Executor::new(
			async move {
				loop {
					// tx1.write(&b"1111111111111111\n"[..]).await;
					led1.set_high();
					clock.sleep_for(Duration::from_ticks(500)).await;
					led1.set_low();
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
					led2.set_high();
					rx.read(&mut buf[..]).await;
					led2.set_low();
					tx.write(&buf[..]).await;
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

	#[task(binds = SysTick, resources = [&clock])]
	fn SysTick(cx: SysTick::Context) {
		cx.resources.clock.tick();
	}

	#[task(binds = USART1)]
	fn USART1(cx: USART1::Context) {
		stm32::usart1_interrupt();
	}
};
