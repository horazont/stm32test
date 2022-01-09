use core::convert::Infallible;
use core::future::Future;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use embedded_dma::{ReadBuffer, StaticReadBuffer, StaticWriteBuffer, WriteBuffer};

use pin_project::{pin_project, pinned_drop};

use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::{pac, rcc, serial};

use super::spincell::{RefMut, SpinCell, TryBorrowError};

use crate::usart;

pub struct Serial<USART> {
	tx: Tx<USART>,
	rx: Rx<USART>,
}

impl<USART: 'static> Serial<USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	pub fn wrap(inner: serial::ErasedSerial<USART>) -> Self {
		let (tx, rx) = inner.split();
		Self {
			tx: Tx::wrap(tx),
			rx: Rx::wrap(rx),
		}
	}

	pub fn split(self) -> (Tx<USART>, Rx<USART>) {
		(self.tx, self.rx)
	}
}

impl<USART: 'static> usart::AsyncWrite<u8> for Serial<USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	type Error = Infallible;
	type Future<'x> = WriteIntr<'x, Tx<USART>>;

	fn write<'x>(&'x mut self, buf: &'x [u8]) -> Self::Future<'x> {
		WriteIntr {
			tx: PhantomData,
			state: WriteState::Waiting,
			buf: buf,
		}
	}
}

#[derive(Clone, Copy)]
pub struct BaudRateConfig {
	config: serial::Config,
	clocks: rcc::Clocks,
}

pub struct BaudRateFromClocks<'x>(pub &'x rcc::Clocks);

impl<'x> usart::BaudRateGenerator<BaudRateConfig> for BaudRateFromClocks<'x> {
	fn calc_baud_rate(&self, bps: u32) -> BaudRateConfig {
		BaudRateConfig {
			config: serial::Config::default().baudrate(bps.bps()),
			clocks: *self.0,
		}
	}
}

impl<USART: 'static> usart::AsyncSetBaudRate for Serial<USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	type Error = Infallible;
	type BaudRate = BaudRateConfig;

	fn poll_set_baud_rate(
		&'_ mut self,
		rate: &Self::BaudRate,
		cx: &mut Context<'_>,
	) -> Poll<Result<(), Self::Error>> {
		let mut txslot = match <Tx<USART> as UsartTxSlot>::try_borrow_mut() {
			Ok(slot) => slot,
			Err(_) => panic!("failed to lock usart for reconfiguration"),
		};
		// there can be no Tx in progress, as polling can only happen on &mut self
		assert!(txslot.1.is_none());
		let mut rxslot = match <Rx<USART> as UsartRxSlot>::try_borrow_mut() {
			Ok(slot) => slot,
			Err(_) => panic!("failed to lock usart for reconfiguration"),
		};
		// there can be no Rx in progress, as polling can only happen on &mut self
		assert!(rxslot.1.is_none());

		let mut serial = rxslot.0.take().unwrap().reunite(txslot.0.take().unwrap());
		let result = match serial.reconfigure(rate.config, rate.clocks) {
			Ok(()) => Poll::Ready(Ok(())),
			Err(nb::Error::WouldBlock) => {
				// we have to spin on this
				cx.waker().wake_by_ref();
				Poll::Pending
			}
			Err(nb::Error::Other(_)) => unreachable!(),
		};
		let (tx, rx) = serial.split();
		rxslot.0 = Some(rx);
		txslot.0 = Some(tx);

		// as nothing is in progress, we do not need to re-enable interrupts
		result
	}
}

impl<USART: 'static> usart::AsyncRead<u8> for Serial<USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	type Error = Infallible;
	type Future<'x> = ReadIntr<'x, Rx<USART>>;

	fn read<'x>(&'x mut self, buf: &'x mut [u8]) -> Self::Future<'x> {
		ReadIntr {
			rx: PhantomData,
			state: ReadState::Waiting,
			buf: Some(buf),
		}
	}
}

pub enum SyncError {
	Locked(TryBorrowError),
	Serial(serial::Error),
}

impl<USART: 'static> embedded_hal::serial::Read<u8> for Serial<USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	type Error = SyncError;

	fn read(&mut self) -> nb::Result<u8, Self::Error> {
		let mut rxslot = match <Rx<USART> as UsartRxSlot>::try_borrow_mut() {
			Ok(slot) => slot,
			Err(e) => return Err(nb::Error::Other(SyncError::Locked(e))),
		};
		let rx = rxslot.0.as_mut().unwrap();
		match rx.read() {
			Ok(v) => Ok(v),
			Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
			Err(nb::Error::Other(e)) => Err(nb::Error::Other(SyncError::Serial(e))),
		}
	}
}

macro_rules! serial {
	($txslot:ident, $rxslot:ident, $usart:ty, $intr:ident) => {
		static $txslot: TxCell<$usart> = TxCell::new();
		static $rxslot: RxCell<$usart> = RxCell::new();

		impl UsartTxSlot for Tx<$usart> {
			type USART = $usart;

			fn try_borrow_mut() -> Result<RefMut<'static, TxCellInner<Self::USART>>, TryBorrowError>
			{
				$txslot.inner.try_borrow_mut()
			}

			fn drop_and_listen(mut borrow: RefMut<'static, TxCellInner<Self::USART>>) {
				// this is nasty! :)
				// SAFETY: enabling the interrupt should be a more-or-less safe operation, even while we don't technically own the peripherial.
				borrow.0.as_mut().unwrap().unlisten();
				let stolen = unsafe { borrow.steal() };
				stolen.0.as_mut().unwrap().listen();
			}
		}

		impl UsartRxSlot for Rx<$usart> {
			type USART = $usart;

			fn try_borrow_mut() -> Result<RefMut<'static, RxCellInner<Self::USART>>, TryBorrowError>
			{
				$rxslot.inner.try_borrow_mut()
			}

			fn drop_and_listen(mut borrow: RefMut<'static, RxCellInner<Self::USART>>) {
				// this is nasty! :)
				// SAFETY: enabling the interrupt should be a more-or-less safe operation, even while we don't technically own the peripherial.
				borrow.0.as_mut().unwrap().unlisten();
				let stolen = unsafe { borrow.steal() };
				stolen.0.as_mut().unwrap().listen();
			}
		}

		pub fn $intr() {
			// So the thing is: we *have* to do things in the interrupt handler in order to prevent interrupts from firing *continuously*.
			// In particular, we have to clear the interrupt flags.
			let usart = unsafe { &*<$usart>::ptr() };
			let sr = usart.sr.read();
			if sr.txe().bit_is_set() {
				// transmit interrupt
				let disable_interrupts = match $txslot.try_borrow_mut() {
					Ok(mut b) => match b.1.as_mut() {
						Some(mut txop_ref) => {
							if txop_ref.buf.len() > 0 {
								let ch = txop_ref.buf[0];
								txop_ref.buf = txop_ref.buf.split_at(1).1;
								usart.dr.write(|w| w.dr().bits(ch as u16));
							}
							if txop_ref.buf.len() == 0 {
								// all we can do is try to re-trigger the task... we cannot drop the op because the Future's Drop will do that eventually.
								txop_ref.waker.wake_by_ref();
								true
							} else {
								false
							}
						}
						None => {
							// disable the interrupts: there is nothing to do.
							true
						}
					},
					Err(_) => {
						// this is a problem: the main code is holding the lock while we're interrupting. this may happen rarely, so we have to disable interrupts to give the main code a chance to re-enable them.
						true
					}
				};
				if disable_interrupts {
					usart
						.cr1
						.modify(|_, w| w.tcie().clear_bit().txeie().clear_bit());
				}
			}
			if sr.rxne().bit_is_set() {
				let disable_interrupts = match $rxslot.try_borrow_mut() {
					Ok(mut b) => match b.1.as_mut() {
						Some(mut rxop_ref) => {
							if rxop_ref.offs < rxop_ref.buf.len() {
								let ch = usart.dr.read().dr().bits() as u8;
								unsafe { rxop_ref.buf[rxop_ref.offs].as_mut_ptr().write(ch) };
								rxop_ref.offs += 1;
							}
							if rxop_ref.offs >= rxop_ref.buf.len() {
								rxop_ref.waker.wake_by_ref();
								// disable interrupts, we're done with this transfer.
								true
							} else {
								// there is more to do, wait for the next rxne.
								false
							}
						}
						// disable interrupts, there is nothing to do.
						None => true,
					},
					Err(_) => {
						// this is a problem: the main code is holding the lock while we're interrupting. this may happen rarely, so we have to disable interrupts to give the main code a chance to re-enable them.
						true
					}
				};
				if disable_interrupts {
					usart.cr1.modify(|_, w| w.rxneie().clear_bit());
				}
			}
		}
	};
}

serial!(TX1_SLOT, RX1_SLOT, pac::USART1, usart1_interrupt);

serial!(TX2_SLOT, RX2_SLOT, pac::USART2, usart2_interrupt);

serial!(TX3_SLOT, RX3_SLOT, pac::USART3, usart3_interrupt);

enum WriteState {
	Waiting,
	Transmitting,
	Done,
}

pub trait UsartTxSlot {
	type USART: 'static;

	fn try_borrow_mut() -> Result<RefMut<'static, TxCellInner<Self::USART>>, TryBorrowError>;

	fn drop_and_listen(borrow: RefMut<'static, TxCellInner<Self::USART>>);
}

#[pin_project(PinnedDrop, !Unpin)]
pub struct WriteIntr<'x, T: UsartTxSlot + 'static> {
	tx: PhantomData<&'x T>,
	buf: &'x [u8],
	state: WriteState,
}

pub struct Tx<USART> {
	phantom: PhantomData<serial::Tx<USART>>,
}

impl<USART> Clone for Tx<USART> {
	fn clone(&self) -> Self {
		Self {
			phantom: PhantomData,
		}
	}
}

impl<USART> Tx<USART>
where
	Tx<USART>: UsartTxSlot,
{
	fn wrap(tx: serial::Tx<<Self as UsartTxSlot>::USART>) -> Self {
		Self::try_borrow_mut().unwrap().0 = Some(tx);
		Self {
			phantom: PhantomData,
		}
	}

	pub fn write<'x, B: Into<&'x [u8]> + 'x>(&'x self, buf: B) -> WriteIntr<'x, Self> {
		WriteIntr {
			tx: PhantomData,
			state: WriteState::Waiting,
			buf: buf.into(),
		}
	}
}

impl<'x, USART> Future for WriteIntr<'x, Tx<USART>>
where
	Tx<USART>: UsartTxSlot,
{
	type Output = Result<usize, Infallible>;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		match this.state {
			WriteState::Waiting => {
				match <Tx<USART> as UsartTxSlot>::try_borrow_mut() {
					Ok(mut txslot) => match txslot.1 {
						None => {
							let buf_slice: &'static [u8] = unsafe {
								// SAFETY: this is super-nasty, and relies on Drop being called or the reference passed into this to be valid forever.
								let len = this.buf.len();
								core::slice::from_raw_parts(this.buf.as_ptr(), len)
							};
							txslot.1 = Some(TxOp {
								waker: cx.waker().clone(),
								buf: buf_slice,
							});
							*this.state = WriteState::Transmitting;
							<Tx<USART> as UsartTxSlot>::drop_and_listen(txslot);
							// interrupt will wake us up
							Poll::Pending
						}
						Some(_) => {
							// need to wait. we also need to re-enable interrupts, because if we raced with the interrupt handler while acquiring the lock, the interrupt handler has failed to acquire it and disabled interrupts in order to let user code continue to operate.
							<Tx<USART> as UsartTxSlot>::drop_and_listen(txslot);
							cx.waker().wake_by_ref();
							Poll::Pending
						}
					},
					Err(_) => {
						// need to wait
						cx.waker().wake_by_ref();
						Poll::Pending
					}
				}
			}
			WriteState::Transmitting => {
				match <Tx<USART> as UsartTxSlot>::try_borrow_mut() {
					Ok(mut txop_ref) => {
						// we are only ever woken using the waker when the transmission is done, hence we don't have to double-check'
						txop_ref.1.take();
						*this.state = WriteState::Done;
						Poll::Ready(Ok(this.buf.len()))
					}
					// we will get notified when it's done
					Err(_) => Poll::Pending,
				}
			}
			WriteState::Done => panic!("future polled after it returned ready"),
		}
	}
}

#[pinned_drop]
impl<'x, T: UsartTxSlot + 'static> PinnedDrop for WriteIntr<'x, T> {
	fn drop(self: Pin<&mut Self>) {
		match self.state {
			// if we set the op up, we're the owner and can cancel it in this way
			WriteState::Transmitting => {
				match <T as UsartTxSlot>::try_borrow_mut() {
					Ok(mut txop) => txop.1 = None,
					// this can only happen if this drop is called from within an interrupt handler -- which is always invalid, because this is a future, after all.
					// we MUST panic here for safety: if we cannot cancel the txop, the interrupt handler will do unspeakable things, because the data we borrowed is now also invalid.
					// note that the panic doesn't really do much here to that end -- as the interrupt may still fire.
					Err(_) => panic!("failed to cancel txop in Drop"),
				}
			}
			_ => (),
		}
	}
}

pub trait UsartRxSlot {
	type USART: 'static;

	fn try_borrow_mut() -> Result<RefMut<'static, RxCellInner<Self::USART>>, TryBorrowError>;

	fn drop_and_listen(borrow: RefMut<'static, RxCellInner<Self::USART>>);
}

pub struct Rx<USART> {
	phantom: PhantomData<USART>,
}

impl<USART> Rx<USART>
where
	Rx<USART>: UsartRxSlot,
{
	fn wrap(rx: serial::Rx<<Self as UsartRxSlot>::USART>) -> Self {
		Self::try_borrow_mut().unwrap().0 = Some(rx);
		Self {
			phantom: PhantomData,
		}
	}

	pub fn read<'x, B: Into<&'x mut [u8]>>(&'x self, buf: B) -> ReadIntr<'x, Self> {
		ReadIntr {
			rx: PhantomData,
			state: ReadState::Waiting,
			buf: Some(buf.into()),
		}
	}
}

enum ReadState {
	Waiting,
	Receiving,
	Done,
}

#[pin_project(PinnedDrop, !Unpin)]
pub struct ReadIntr<'x, T: UsartRxSlot + 'static> {
	rx: PhantomData<&'x T>,
	buf: Option<&'x mut [u8]>,
	state: ReadState,
}

impl<'x, USART> Future for ReadIntr<'x, Rx<USART>>
where
	Rx<USART>: UsartRxSlot,
{
	type Output = Result<(), Infallible>;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		match this.state {
			ReadState::Waiting => {
				let buf = this.buf.as_mut().unwrap();
				match <Rx<USART> as UsartRxSlot>::try_borrow_mut() {
					Ok(mut rxslot) => match rxslot.1 {
						Some(_) => {
							// need to wait. we also need to re-enable interrupts, because if we raced with the interrupt handler while acquiring the lock, the interrupt handler has failed to acquire it and disabled interrupts in order to let user code continue to operate.
							<Rx<USART> as UsartRxSlot>::drop_and_listen(rxslot);
							cx.waker().wake_by_ref();
							Poll::Pending
						}
						None => {
							let buf_slice: &'static mut [MaybeUninit<u8>] = unsafe {
								let len = buf.len();
								core::mem::transmute(core::slice::from_raw_parts_mut(
									buf.as_mut_ptr(),
									len,
								))
							};
							rxslot.1 = Some(RxOp {
								waker: cx.waker().clone(),
								buf: buf_slice,
								offs: 0,
							});
							*this.state = ReadState::Receiving;
							<Rx<USART> as UsartRxSlot>::drop_and_listen(rxslot);
							// interrupt will wake us up when done
							Poll::Pending
						}
					},
					Err(_) => {
						// need to wait, but no need to re-enable any interrupts
						cx.waker().wake_by_ref();
						Poll::Pending
					}
				}
			}
			ReadState::Receiving => {
				match <Rx<USART> as UsartRxSlot>::try_borrow_mut() {
					Ok(mut rxslot) => {
						rxslot.1.take();
						*this.state = ReadState::Done;
						Poll::Ready(Ok(()))
					}
					// we'll get re-notified when it's done, or will we?
					Err(_) => Poll::Pending,
				}
			}
			ReadState::Done => panic!("future polled after it returned already"),
		}
	}
}

#[pinned_drop]
impl<'x, T: UsartRxSlot + 'static> PinnedDrop for ReadIntr<'x, T> {
	fn drop(self: Pin<&mut Self>) {
		match self.state {
			// if we set the op up, we're the owner and can cancel it in this way
			ReadState::Receiving => {
				match <T as UsartRxSlot>::try_borrow_mut() {
					Ok(mut rxslot) => rxslot.1 = None,
					// this can only happen if this drop is called from within an interrupt handler -- which is always invalid, because this is a future, after all.
					// we MUST panic here for safety: if we cannot cancel the txop, the interrupt handler will do unspeakable things, because the data we borrowed is now also invalid.
					// note that the panic doesn't really do much here to that end -- as the interrupt may still fire.
					Err(_) => panic!("failed to cancel rxop in Drop"),
				}
			}
			_ => (),
		}
	}
}

trait UartReadBuffer: StaticReadBuffer<Word = u8> + ReadBuffer<Word = u8> {}
trait UartWriteBuffer: StaticWriteBuffer<Word = u8> + WriteBuffer<Word = u8> {}

impl<T: StaticReadBuffer<Word = u8> + ReadBuffer<Word = u8>> UartReadBuffer for T {}
impl<T: StaticWriteBuffer<Word = u8> + WriteBuffer<Word = u8>> UartWriteBuffer for T {}

pub struct TxOp<'x> {
	waker: Waker,
	buf: &'x [u8],
}

pub struct TxCellInner<USART>(Option<serial::Tx<USART>>, Option<TxOp<'static>>);

#[repr(transparent)]
struct TxCell<USART> {
	inner: SpinCell<TxCellInner<USART>>,
}

impl<USART: 'static> TxCell<USART> {
	const fn new() -> Self {
		Self {
			inner: SpinCell::new(TxCellInner(None, None)),
		}
	}

	fn try_borrow_mut(&self) -> Result<RefMut<'_, TxCellInner<USART>>, TryBorrowError> {
		self.inner.try_borrow_mut()
	}
}

pub struct RxOp<'x> {
	waker: Waker,
	buf: &'x mut [MaybeUninit<u8>],
	offs: usize,
}

pub struct RxCellInner<USART>(Option<serial::Rx<USART>>, Option<RxOp<'static>>);

#[repr(transparent)]
struct RxCell<USART> {
	inner: SpinCell<RxCellInner<USART>>,
}

impl<USART: 'static> RxCell<USART> {
	const fn new() -> Self {
		Self {
			inner: SpinCell::new(RxCellInner(None, None)),
		}
	}

	fn try_borrow_mut(&self) -> Result<RefMut<'_, RxCellInner<USART>>, TryBorrowError> {
		self.inner.try_borrow_mut()
	}
}
