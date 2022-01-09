#![no_std]
use core::cmp;
use core::future::Future;
use core::ops::{Add, Sub};
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use pin_project::pin_project;

pub enum NoReturn {}

pub mod spincell;
pub mod stm32;
pub mod stm32_onewire;
pub mod usart;

#[repr(transparent)]
struct WakeupCondition {
	inner: AtomicBool,
}

unsafe fn raw_waker_clone(data: *const ()) -> RawWaker {
	RawWaker::new(data, &WAKER_VTABLE)
}

unsafe fn raw_waker_wake(data: *const ()) {
	let this: &AtomicBool = core::mem::transmute(data);
	this.store(true, Ordering::Release);
}

unsafe fn raw_waker_drop(_: *const ()) {}

static WAKER_VTABLE: RawWakerVTable = RawWakerVTable::new(
	raw_waker_clone,
	raw_waker_wake,
	raw_waker_wake,
	raw_waker_drop,
);

impl WakeupCondition {
	const fn new() -> Self {
		Self {
			inner: AtomicBool::new(true),
		}
	}

	#[inline(always)]
	fn take_ready(&self) -> bool {
		match self
			.inner
			.compare_exchange(true, false, Ordering::AcqRel, Ordering::Acquire)
		{
			Ok(_) => true,
			Err(_) => false,
		}
	}

	fn set_never(&self) {
		self.inner.store(false, Ordering::Release);
	}

	fn waker(self: Pin<&Self>) -> Waker {
		// SAFETY: from_raw is unsafe because terrible things™ may happen if RawWaker (and its vtable) does not adhere to the contract. To the best of my knowledge, this implementation *does* adhere to the contract and is even sync + send.
		unsafe {
			Waker::from_raw(RawWaker::new(
				core::mem::transmute(&self.inner),
				&WAKER_VTABLE,
			))
		}
	}
}

#[pin_project]
struct Task<T> {
	#[pin]
	inner: T,
	#[pin]
	wakeup_condition: WakeupCondition,
}

impl<T> Task<T> {
	const fn wrap(inner: T) -> Self {
		Self {
			inner,
			wakeup_condition: WakeupCondition::new(),
		}
	}
}

impl<T: Future> Task<T> {
	fn advance(self: Pin<&mut Self>) {
		if self.wakeup_condition.take_ready() {
			let this = self.project();
			let waker = this.wakeup_condition.as_ref().waker();
			let mut cx = Context::from_waker(&waker);
			match this.inner.poll(&mut cx) {
				Poll::Pending => (),
				Poll::Ready(_) => {
					this.wakeup_condition.set_never();
				}
			}
		}
	}
}

pub struct SystickClock {
	now: AtomicU16,
}

impl SystickClock {
	#[inline(always)]
	pub fn tick(&self) {
		self.now.fetch_add(1, Ordering::Relaxed);
	}
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Instant(u16);

impl cmp::PartialOrd<Instant> for Instant {
	fn partial_cmp(&self, other: &Self) -> Option<cmp::Ordering> {
		if self.0 == other.0 {
			Some(cmp::Ordering::Equal)
		} else if self.le(other) {
			Some(cmp::Ordering::Less)
		} else if other.le(self) {
			Some(cmp::Ordering::Greater)
		} else {
			None
		}
	}

	fn le(&self, other: &Self) -> bool {
		static THRESHOLD: u16 = 1 << 15;
		other.0.wrapping_sub(self.0) < THRESHOLD
	}
}

impl cmp::Ord for Instant {
	fn cmp(&self, other: &Instant) -> cmp::Ordering {
		self.partial_cmp(other).unwrap_or(cmp::Ordering::Less)
	}
}

impl Sub<Instant> for Instant {
	type Output = Duration;

	fn sub(self, other: Instant) -> Self::Output {
		Duration(self.0.wrapping_sub(other.0))
	}
}

impl Add<Duration> for Instant {
	type Output = Instant;

	fn add(self, other: Duration) -> Self::Output {
		Self(self.0.wrapping_add(other.0))
	}
}

impl Sub<Duration> for Instant {
	type Output = Instant;

	fn sub(self, other: Duration) -> Self::Output {
		Self(self.0.wrapping_sub(other.0))
	}
}

impl FromTicks for Instant {
	type Ticks = u16;

	fn from_ticks(ticks: u16) -> Self {
		Self(ticks)
	}
}

impl IntoTicks for Instant {
	type Ticks = u16;

	fn into_ticks(self) -> u16 {
		self.0
	}
}

#[derive(Clone, Copy, PartialEq, PartialOrd, Eq, Ord)]
#[repr(transparent)]
pub struct Duration(u16);

impl FromTicks for Duration {
	type Ticks = u16;

	fn from_ticks(ticks: Self::Ticks) -> Self {
		Self(ticks)
	}
}

pub trait IntoTicks {
	type Ticks;

	fn into_ticks(self) -> Self::Ticks;
}

pub trait FromTicks {
	type Ticks;

	fn from_ticks(ticks: Self::Ticks) -> Self;
}

pub trait Monotonic {
	type Instant: cmp::Ord
		+ Copy
		+ Add<Self::Duration, Output = Self::Instant>
		+ Sub<Self::Duration, Output = Self::Instant>
		+ Sub<Self::Instant, Output = Self::Duration>
		+ IntoTicks<Ticks = u16>
		+ FromTicks<Ticks = u16>;
	type Duration: cmp::Ord + Copy;

	fn now(&self) -> Self::Instant;
}

impl SystickClock {
	pub fn new() -> Self {
		Self {
			now: AtomicU16::new(0),
		}
	}
}

impl Monotonic for SystickClock {
	type Instant = Instant;
	type Duration = Duration;

	fn now(&self) -> Self::Instant {
		Instant(self.now.load(Ordering::Relaxed))
	}
}

impl<T: Monotonic> Monotonic for &T {
	type Instant = T::Instant;
	type Duration = T::Duration;

	fn now(&self) -> Self::Instant {
		(**self).now()
	}
}

#[pin_project]
pub struct Executor<A, B> {
	#[pin]
	f0: Task<A>,
	#[pin]
	f1: Task<B>,
}

impl<A, B> Executor<A, B> {
	pub const fn new(f0: A, f1: B) -> Self {
		Self {
			f0: Task::wrap(f0),
			f1: Task::wrap(f1),
		}
	}
}

impl<A: Future<Output = NoReturn>, B: Future<Output = NoReturn>> Executor<A, B> {
	pub fn advance(self: Pin<&mut Self>) -> () {
		let this = self.project();
		this.f0.advance();
		this.f1.advance();
	}
}

pub struct Yield(bool);

pub fn yield_once() -> Yield {
	Yield(false)
}

impl Future for Yield {
	type Output = ();

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		if self.0 {
			Poll::Ready(())
		} else {
			cx.waker().wake_by_ref();
			Poll::Pending
		}
	}
}

pub struct Sleep<'x, C: Monotonic + ?Sized> {
	clock: &'x C,
	until: C::Instant,
}

impl<'x, C: Monotonic + ?Sized> Future for Sleep<'x, C> {
	type Output = ();

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		if self.clock.now() >= self.until {
			return Poll::Ready(());
		}

		cx.waker().wake_by_ref();
		Poll::Pending
	}
}

pub trait MonotonicExt: Monotonic {
	fn sleep_for(&self, d: Self::Duration) -> Sleep<'_, Self>;
}

impl<T: Monotonic> MonotonicExt for T {
	fn sleep_for(&self, d: T::Duration) -> Sleep<'_, T> {
		Sleep {
			clock: self,
			until: self.now() + d,
		}
	}
}
