use core::future::Future;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use embedded_dma::{ReadBuffer, StaticReadBuffer, StaticWriteBuffer, WriteBuffer};

use pin_project::{pin_project, pinned_drop};

use stm32f1xx_hal;
use stm32f1xx_hal::{pac, rcc, serial};

use super::spincell::{RefMut, SpinCell, TryBorrowError};

pub struct Serial<USART> {
	tx: Tx<USART>,
	rx: Rx<USART>,
}

impl<USART> Serial<USART>
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

	pub fn reconfigure<'x>(
		&'x mut self,
		config: impl Into<serial::Config>,
		clocks: rcc::Clocks,
	) -> Reconfigure<'x, USART> {
		Reconfigure {
			r: self,
			config: config.into(),
			clocks,
			serial: None,
		}
	}

	pub fn write<'x, B: Into<&'x [u8]> + 'x>(&'x self, buf: B) -> WriteIntr<'x, Tx<USART>> {
		WriteIntr {
			tx: PhantomData,
			state: WriteState::Waiting,
			buf: buf.into(),
		}
	}

	pub fn read<'x, B: Into<&'x mut [u8]>>(&'x self, buf: B) -> ReadIntr<'x, Rx<USART>> {
		ReadIntr {
			rx: PhantomData,
			state: ReadState::Waiting,
			buf: Some(buf.into()),
		}
	}
}

impl Serial<pac::USART1> {
	pub fn split(self) -> (Tx<pac::USART1>, Rx<pac::USART1>) {
		(self.tx, self.rx)
	}
}

#[pin_project(PinnedDrop, !Unpin)]
pub struct Reconfigure<'x, USART: 'static>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	r: &'x mut Serial<USART>,
	config: serial::Config,
	clocks: rcc::Clocks,
	serial: Option<serial::ErasedSerial<USART>>,
}

impl<'x, USART: 'static> Future for Reconfigure<'x, USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	type Output = ();

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		let mut txslot = match <Tx<USART> as UsartTxSlot>::try_borrow_mut() {
			Ok(slot) => slot,
			Err(_) => panic!("failed to lock usart for reconfiguration"),
		};
		// there can be no Tx in progress, as the Reconfigure future can only be created from  a mutable reference of the Serial object
		assert!(txslot.1.is_none());
		let mut rxslot = match <Rx<USART> as UsartRxSlot>::try_borrow_mut() {
			Ok(slot) => slot,
			Err(_) => panic!("failed to lock usart for reconfiguration"),
		};
		// there can be no Rx in progress, as the Reconfigure future can only be created from  a mutable reference of the Serial object
		assert!(rxslot.1.is_none());

		if this.serial.is_none() {
			let tx = txslot.0.take().unwrap();
			let rx = rxslot.0.take().unwrap();
			let serial = tx.reunite(rx);
			*this.serial = Some(serial);
		}

		let mut serial = this.serial.as_mut().unwrap();
		match serial.reconfigure(*this.config, *this.clocks) {
			Ok(()) => {
				drop(serial);
				let (tx, rx) = this.serial.take().unwrap().split();
				txslot.0 = Some(tx);
				rxslot.0 = Some(rx);
				Poll::Ready(())
			}
			Err(nb::Error::WouldBlock) => {
				cx.waker().wake_by_ref();
				Poll::Pending
			}
			Err(nb::Error::Other(_)) => unreachable!(),
		}
	}
}

#[pinned_drop]
impl<'x, USART: 'static> PinnedDrop for Reconfigure<'x, USART>
where
	Tx<USART>: UsartTxSlot<USART = USART>,
	Rx<USART>: UsartRxSlot<USART = USART>,
	USART: serial::Instance,
{
	fn drop(self: Pin<&mut Self>) {
		let this = self.project();
		if let Some(serial) = this.serial.take() {
			let mut txslot = match <Tx<USART> as UsartTxSlot>::try_borrow_mut() {
				Ok(slot) => slot,
				Err(_) => panic!("failed to lock usart for reconfiguration"),
			};
			let mut rxslot = match <Rx<USART> as UsartRxSlot>::try_borrow_mut() {
				Ok(slot) => slot,
				Err(_) => panic!("failed to lock usart for reconfiguration"),
			};
			let (tx, rx) = serial.split();
			txslot.0 = Some(tx);
			rxslot.0 = Some(rx);
		}
	}
}

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

#[pin_project(PinnedDrop)]
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

impl UsartTxSlot for Tx<pac::USART1> {
	type USART = pac::USART1;

	fn try_borrow_mut() -> Result<RefMut<'static, TxCellInner<Self::USART>>, TryBorrowError> {
		TX1_SLOT.inner.try_borrow_mut()
	}

	fn drop_and_listen(mut borrow: RefMut<'static, TxCellInner<Self::USART>>) {
		// this is nasty! :)
		// SAFETY: enabling the interrupt should be a more-or-less safe operation, even while we don't technically own the peripherial.
		borrow.0.as_mut().unwrap().unlisten();
		let stolen = unsafe { borrow.steal() };
		stolen.0.as_mut().unwrap().listen();
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
	type Output = ();

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
						Poll::Ready(())
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

impl UsartRxSlot for Rx<pac::USART1> {
	type USART = pac::USART1;

	fn try_borrow_mut() -> Result<RefMut<'static, RxCellInner<Self::USART>>, TryBorrowError> {
		RX1_SLOT.inner.try_borrow_mut()
	}

	fn drop_and_listen(mut borrow: RefMut<'static, RxCellInner<Self::USART>>) {
		// this is nasty! :)
		// SAFETY: enabling the interrupt should be a more-or-less safe operation, even while we don't technically own the peripherial.
		borrow.0.as_mut().unwrap().unlisten();
		let stolen = unsafe { borrow.steal() };
		stolen.0.as_mut().unwrap().listen();
	}
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
			rx: self,
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

#[pin_project(PinnedDrop)]
pub struct ReadIntr<'x, T: UsartRxSlot + 'static> {
	rx: &'x T,
	buf: Option<&'x mut [u8]>,
	state: ReadState,
}

impl<'x, USART> Future for ReadIntr<'x, Rx<USART>>
where
	Rx<USART>: UsartRxSlot,
{
	type Output = ();

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
						Poll::Ready(())
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

/// Hold the waker, the buffer and the flag which indicates completion of the transfer
static TX1_SLOT: TxCell<pac::USART1> = TxCell::new();

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

static RX1_SLOT: RxCell<pac::USART1> = RxCell::new();

pub fn usart1_interrupt() {
	// So the thing is: we *have* to do things in the interrupt handler in order to prevent interrupts from firing *continuously*.
	// In particular, we have to clear the interrupt flags.
	let usart = unsafe { &*stm32f1xx_hal::pac::USART1::ptr() };
	let sr = usart.sr.read();
	if sr.txe().bit_is_set() {
		if sr.tc().bit_is_set() {
			// transfer complete, make sure we don't get interrupted because of that again
			usart.sr.modify(|_, w| w.tc().clear_bit());
		}
		// transmit interrupt
		let disable_interrupts = match TX1_SLOT.try_borrow_mut() {
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
		let disable_interrupts = match RX1_SLOT.try_borrow_mut() {
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
