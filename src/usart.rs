use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use pin_project::pin_project;

use embedded_hal::serial;

#[pin_project]
pub struct WriteWord<T> {
	inner: T,
	value: u8,
}

impl<T: serial::Write<u8>> Future for WriteWord<T> {
	type Output = Result<(), T::Error>;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		match this.inner.write(*this.value) {
			Err(nb::Error::WouldBlock) => {
				cx.waker().wake_by_ref();
				Poll::Pending
			}
			Err(nb::Error::Other(e)) => Poll::Ready(Err(e)),
			Ok(()) => Poll::Ready(Ok(())),
		}
	}
}

#[pin_project]
pub struct WriteBuf<'x, T: ?Sized> {
	inner: &'x mut T,
	value: &'x [u8],
}

impl<'x, T: serial::Write<u8> + ?Sized> Future for WriteBuf<'x, T> {
	type Output = Result<(), T::Error>;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		while this.value.len() > 0 {
			match this.inner.write(this.value[0]) {
				Err(nb::Error::WouldBlock) => {
					cx.waker().wake_by_ref();
					return Poll::Pending;
				}
				Err(nb::Error::Other(e)) => return Poll::Ready(Err(e)),
				Ok(()) => {
					*this.value = this.value.split_at(1).1;
				}
			}
		}
		Poll::Ready(Ok(()))
	}
}

pub trait WakeSource {
	fn set_waker(&self, waker: Waker);
}

pub trait WriteExt: serial::Write<u8> {
	fn write_buf<'x>(&'x mut self, buf: &'x [u8]) -> WriteBuf<'x, Self>;
}

impl<T: serial::Write<u8>> WriteExt for T {
	fn write_buf<'x>(&'x mut self, buf: &'x [u8]) -> WriteBuf<'x, Self> {
		WriteBuf {
			inner: self,
			value: buf,
		}
	}
}
