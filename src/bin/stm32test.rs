#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use core::pin::Pin;

use cortex_m::peripheral::{Peripherals, syst};
use cortex_m_semihosting::{debug, hprintln};

use stm32test::{Executor, SystickClock, sleep_for, Monotonic, FromTicks, yield_once};

type Duration = <SystickClock as Monotonic>::Duration;

#[rtic::app(device = lm3s6965)]
const APP: () = {
	struct Resources {
		clock: SystickClock,
	}

    #[init]
    fn init(_: init::Context) -> init::LateResources {
		hprintln!("init").unwrap();
		let mut core = unsafe { Peripherals::steal() };
		core.SYST.set_clock_source(syst::SystClkSource::Core);
		core.SYST.set_reload(12000);
		core.SYST.enable_interrupt();
		core.SYST.enable_counter();
		init::LateResources{
			clock: SystickClock::new(),
		}
	}

	#[idle(resources = [&clock])]
	fn idle(cx: idle::Context) -> ! {
		let mut executor = Executor::new(
			cx.resources.clock,
			async {
				loop {
					hprintln!("task 1").unwrap();
					sleep_for(cx.resources.clock, Duration::from_ticks(1000)).await;
				}
			},
			async {
				loop {
					/* hprintln!("task 2").unwrap();
					sleep_for(cx.resources.clock, Duration::from_ticks(1500)).await; */
					yield_once().await;
				}
			},
		);
		hprintln!("sz: {}", core::mem::size_of_val(&executor));
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
};
