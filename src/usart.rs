use core::fmt::Debug;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use pin_project::pin_project;

pub trait AsyncWrite<Word> {
	type Error: Debug;
	type Future<'x>: Future<Output = Result<usize, Self::Error>> + 'x
	where
		Self: 'x,
		Word: 'x;

	fn write<'x>(&'x mut self, buf: &'x [Word]) -> Self::Future<'x>;
}

pub trait AsyncRead<Word> {
	type Error: Debug;
	type Future<'x>: Future<Output = Result<(), Self::Error>> + 'x
	where
		Self: 'x,
		Word: 'x;

	fn read<'x>(&'x mut self, buf: &'x mut [Word]) -> Self::Future<'x>;
}

pub trait AsyncSetBaudRate {
	type Error: Debug;
	type BaudRate;

	fn poll_set_baud_rate(
		&'_ mut self,
		rate: &Self::BaudRate,
		cx: &mut Context<'_>,
	) -> Poll<Result<(), Self::Error>>;
}

pub trait BaudRateGenerator<T> {
	fn calc_baud_rate(&self, bps: u32) -> T;
}

#[pin_project]
pub struct SetBaudRate<'x, T: AsyncSetBaudRate + ?Sized> {
	rate: T::BaudRate,
	#[pin]
	inner: &'x mut T,
}

impl<'x, T: AsyncSetBaudRate> Future for SetBaudRate<'x, T> {
	type Output = Result<(), T::Error>;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let mut this = self.project();
		this.inner.poll_set_baud_rate(this.rate, cx)
	}
}

pub trait AsyncSetBaudRateExt: AsyncSetBaudRate {
	fn set_baud_rate(&mut self, rate: Self::BaudRate) -> SetBaudRate<'_, Self>;
}

impl<T: AsyncSetBaudRate> AsyncSetBaudRateExt for T {
	fn set_baud_rate(&mut self, rate: Self::BaudRate) -> SetBaudRate<'_, Self> {
		SetBaudRate { inner: self, rate }
	}
}
