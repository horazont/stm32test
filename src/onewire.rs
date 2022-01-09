use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use pin_project::pin_project;

use crate::usart;
use crate::usart::AsyncSetBaudRateExt;

pub struct OneWire<
	T: usart::AsyncSetBaudRate
		+ usart::AsyncRead<u8>
		+ usart::AsyncWrite<u8>
		+ embedded_hal::serial::Read<u8>,
> {
	inner: T,
	reset_rate: T::BaudRate,
	data_rate: T::BaudRate,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum OneWireStatus {
	Empty,
	Presence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchError {
	Vanished,
}

pub const ADDR_LEN: u8 = 8;
pub type DeviceAddress = [u8; ADDR_LEN as usize];
pub static ZERO_ADDR: DeviceAddress = [0u8; ADDR_LEN as usize];

#[repr(u8)]
pub enum Command {
	Search = 0xf0,
	Broadcast = 0xcc,
	MatchRom = 0x55,
}

macro_rules! command {
	($self:expr, $x:expr) => {
		$self.write_bytes(core::slice::from_ref(&($x as Command as u8)))
	};
}

#[inline(always)]
fn split_bit_offs(bit: u8) -> (u8, u8) {
	let byteaddr = (bit & 0xf8) >> 3;
	let bitoffs = bit & 0x07;
	(byteaddr, bitoffs)
}

#[inline(always)]
fn get_bit(addr: &DeviceAddress, bit: u8) -> bool {
	let (byteaddr, bitoffs) = split_bit_offs(bit);
	addr[byteaddr as usize] & (1 << bitoffs) != 0
}

impl<
		B: Clone + 'static,
		T: usart::AsyncSetBaudRate<BaudRate = B>
			+ usart::AsyncRead<u8>
			+ usart::AsyncWrite<u8>
			+ embedded_hal::serial::Read<u8>,
	> OneWire<T>
{
	pub fn new<V: usart::BaudRateGenerator<B>>(serial: T, gen: &V) -> Self {
		Self {
			inner: serial,
			reset_rate: gen.calc_baud_rate(9_600),
			data_rate: gen.calc_baud_rate(115_200),
		}
	}

	pub async fn reset(&mut self) -> OneWireStatus {
		self.inner
			.set_baud_rate(self.reset_rate.clone())
			.await
			.unwrap();
		let echo = self.probe(0xf0).await;
		self.inner
			.set_baud_rate(self.data_rate.clone())
			.await
			.unwrap();
		if echo & 0xf0 < 0xf0 {
			OneWireStatus::Presence
		} else {
			OneWireStatus::Empty
		}
	}

	async fn probe(&mut self, mut signal: u8) -> u8 {
		// before writing, we need to ensure that the usart rx is clear. it might not be clear if we have noise on the line.
		// we explicitly do not care about success/failure here
		let _ = embedded_hal::serial::Read::<u8>::read(&mut self.inner);
		self.inner
			.write(core::slice::from_ref(&signal))
			.await
			.unwrap();
		usart::AsyncRead::<u8>::read(&mut self.inner, core::slice::from_mut(&mut signal))
			.await
			.unwrap();
		signal
	}

	fn bit_read(&mut self) -> BitRead<impl Future<Output = u8> + '_> {
		BitRead {
			inner: self.probe(0xff),
		}
	}

	fn presence_probe(&mut self) -> PresenceProbe<impl Future<Output = u8> + '_> {
		PresenceProbe {
			inner: self.probe(0xff),
		}
	}

	fn write_bit(&mut self, bit: bool) -> impl Future<Output = u8> + '_ {
		self.probe(if bit { 0xff } else { 0x00 })
	}

	pub async fn write_bytes(&mut self, bytes: &[u8]) {
		for byte in bytes.iter() {
			let mut byte = *byte;
			for _ in 0..8 {
				let bit = byte & 0x1;
				byte = byte >> 1;
				if bit != 0 {
					self.probe(0xff).await;
				} else {
					self.probe(0x00).await;
				}
			}
		}
	}

	pub async fn read_bytes(&mut self, buf: &mut [u8]) {
		for i in 0..buf.len() {
			let mut byte_buf = 0x00u8;
			for bitoffs in 0..8 {
				let bit = self.bit_read().await;
				if bit {
					byte_buf |= 1 << bitoffs;
				}
			}
			buf[i] = byte_buf;
		}
	}

	/// Find the next device based on the previous address.
	///
	/// ## Algorithm
	///
	/// This implements the depth-first tree search in the 1-wire address
	/// space. It uses the 1-Wire search command. The search command works in
	/// the following way:
	///
	/// The primary node (this one) sends the address bit by bit down the bus.
	/// Before each bit, two reads are executed. If there are devices on the
	/// bus whose address starts with the address already given on the bus
	/// followed by a 0 bit, the first read will return a signal. If there
	/// are devices on the bus where the next address bit is a 1, the second
	/// read will return a signal.
	///
	/// This way, using two search commands, it is possible to find the next
	/// device. The first command is used to find the highest bit where a
	/// device has a 1 in a place where the current address has a 0. When this
	/// is found, a second search request is issued which repeats the address
	/// up to that location and then extends it by following the replies on
	/// the bus after each bit, preferring zeroes over ones.
	///
	/// (This preference for zeroes over ones is important because we want to
	/// find the *next* address -- we must thus go for the lower value.)
	///
	/// At the end of the search command, the full next address is known.
	///
	/// If no address bit indicates the presence of a higher-address device,
	/// the status [`OneWireStatus::Empty`] is returned.
	///
	/// If the device disappears mid-scan, [`SearchError::Vanished`] is
	/// returned. In that case, a new search can be started to find the next
	/// device, if any; the address has not been changed.
	///
	/// If no device is found during the initial reset strobe,
	/// [`OneWireStatus::Empty`] is returned.
	pub async fn find_next(
		&mut self,
		addr: &mut DeviceAddress,
	) -> Result<OneWireStatus, SearchError> {
		match self.reset().await {
			OneWireStatus::Presence => (),
			status => return Ok(status),
		};

		// Step 1: we scan through the current (previous) address and search for the highest bit where a device indicates that it has a higher address.
		command!(self, Command::Search).await;

		// We abuse 0xff as "none" here, for space efficiency; The highest possible bit is 8*8 = 64, which is less than 0xff.
		let mut alt_bit: u8 = 0xff;

		for bit in 0..(ADDR_LEN * 8) {
			// we are only interested in one bits
			let zero_bit_presence = self.presence_probe().await;
			let one_bit_presence = self.presence_probe().await;

			let prevbit = get_bit(&addr, bit);
			if !prevbit {
				if one_bit_presence == OneWireStatus::Presence {
					alt_bit = bit;
				}
				if zero_bit_presence == OneWireStatus::Empty {
					// that indicates that no other device can be in this subtree (possibly a device has vanished or we are starting with the zero address)
					// so we can exit early, we are not going to find a higher bit to toggle than this.
					break;
				}
			} else if one_bit_presence == OneWireStatus::Empty {
				// similarly, our previous address bit indicated existence of a device, which the bus now denies -> we can exit here, we're not going to find another device down this side of the tree.
				break;
			}

			self.write_bit(prevbit).await;
		}

		if alt_bit == 0xff {
			// we found no bit in the address where a device with higher address exists. the search is over
			return Ok(OneWireStatus::Empty);
		}

		match self.reset().await {
			OneWireStatus::Presence => (),
			// no devices anymore -> return vanished
			OneWireStatus::Empty => return Err(SearchError::Vanished),
		}

		command!(self, Command::Search).await;

		for bit in 0..alt_bit {
			let zero_bit_presence = self.presence_probe().await;
			let one_bit_presence = self.presence_probe().await;

			let prevbit = get_bit(&addr, bit);
			if (prevbit == false && zero_bit_presence == OneWireStatus::Empty)
				|| (prevbit == true && one_bit_presence == OneWireStatus::Empty)
			{
				// the device we were looking for has vanished in the meantime -> return error
				return Err(SearchError::Vanished);
			}

			self.write_bit(prevbit).await;
		}

		{
			// we do not care about the zero-side of the tree
			let _zero_bit_presence = self.presence_probe().await;
		}

		{
			let one_bit_presence = self.presence_probe().await;
			if one_bit_presence == OneWireStatus::Empty {
				// we were looking for a device with a one in this place of the address, we saw it earlier. it must've vanished.
				return Err(SearchError::Vanished);
			}
		}

		self.write_bit(true).await;

		{
			let (byteaddr, bitoffs) = split_bit_offs(alt_bit);
			// we know that the bit was previously zero, otherwise we wouldn't have found it. so we can just set it to 1 by or-ing
			addr[byteaddr as usize] |= 1 << bitoffs;

			// now we also zero everything after this bit in order to make the search work correctly if we exit early
			// first we zero the current byte bitwise
			for bit in (bitoffs + 1)..8 {
				addr[byteaddr as usize] &= !(1 << bit);
			}
			// and then we zero everything else bytewise
			for byte in (byteaddr + 1)..ADDR_LEN {
				addr[byte as usize] = 0u8;
			}
		}

		for bit in (alt_bit + 1)..(ADDR_LEN * 8) {
			let zero_bit_presence = self.presence_probe().await;
			let one_bit_presence = self.presence_probe().await;

			let value = if zero_bit_presence == OneWireStatus::Presence {
				// we have to go for low addresses first to do depth-first search
				// we do not have to zero anything here because we already cleared the remainder of the address earlier. just walk down the tree
				false
			} else if one_bit_presence == OneWireStatus::Presence {
				let (byteaddr, bitoffs) = split_bit_offs(bit);
				addr[byteaddr as usize] |= 1 << bitoffs;
				true
			} else {
				// device vanished -- as we cleared the address before, we do not have to clear it here and the next search will continue here.
				return Err(SearchError::Vanished);
			};

			self.write_bit(value).await;
		}

		Ok(OneWireStatus::Presence)
	}
}

#[pin_project]
struct PresenceProbe<T: Future<Output = u8>> {
	#[pin]
	inner: T,
}

impl<T: Future<Output = u8>> Future for PresenceProbe<T> {
	type Output = OneWireStatus;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		match this.inner.poll(cx) {
			Poll::Pending => Poll::Pending,
			Poll::Ready(signal) => Poll::Ready(if signal == 0xff {
				OneWireStatus::Empty
			} else {
				OneWireStatus::Presence
			}),
		}
	}
}

#[pin_project]
struct BitRead<T: Future<Output = u8>> {
	#[pin]
	inner: T,
}

impl<T: Future<Output = u8>> Future for BitRead<T> {
	type Output = bool;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
		let this = self.project();
		match this.inner.poll(cx) {
			Poll::Pending => Poll::Pending,
			Poll::Ready(signal) => Poll::Ready(signal == 0xff),
		}
	}
}

/* #[pin_project(project = ProbeStateProj)]
pub enum ProbeState<'x, USART: 'static>
	where
	stm32::Tx<USART>: stm32::UsartTxSlot<USART = USART>,
	stm32::Rx<USART>: stm32::UsartRxSlot<USART = USART>,
	USART: stm32f1xx_hal::serial::Instance
{
	Preparing,
	Writing(#[pin] stm32::WriteIntr<'x, stm32::Tx<USART>>),
	Reading(#[pin] stm32::ReadIntr<'x, stm32::Rx<USART>>),
}

#[pin_project]
pub struct Probe<'x, USART: 'static>
	where
	stm32::Tx<USART>: stm32::UsartTxSlot<USART = USART>,
	stm32::Rx<USART>: stm32::UsartRxSlot<USART = USART>,
	USART: stm32f1xx_hal::serial::Instance
{
	#[pin]
	serial: &'x mut stm32::Serial<USART>,
	signal: u8,
	#[pin]
	state: ProbeState<'x, USART>,
}


impl<'x, USART: 'static> Future for Probe<'x, USART>
	where
	stm32::Tx<USART>: stm32::UsartTxSlot<USART = USART>,
	stm32::Rx<USART>: stm32::UsartRxSlot<USART = USART>,
	USART: stm32f1xx_hal::serial::Instance
{
	type Output = u8;

	fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output>{
		let this = self.project();
		match this.state.project() {
			ProbeStateProj::Preparing => {
				let op = this.serial.write(core::slice::from_ref(this.signal));
				this.state.set(ProbeState::Writing(op));
				// return self.poll(cx);
				todo!();
			},
			ProbeStateProj::Writing(op) => match op.poll(cx) {
				Poll::Pending => return Poll::Pending,
				Poll::Ready(()) => {
					/* let op = self.serial.read(core::slice::from_mut(this.signal));
					this.state.set(ProbeState::Reading(op));
					return self.poll(cx); */
					todo!()
				},
			},
			ProbeStateProj::Reading(op) => match op.poll(cx) {
				Poll::Pending => Poll::Pending,
				Poll::Ready(()) => Poll::Ready(*this.signal),
			},
		}
	}
} */

/* enum ResetState<'x, USART: 'static>
	where
	stm32::Tx<USART>: stm32::UsartTxSlot<USART = USART>,
	stm32::Rx<USART>: stm32::UsartRxSlot<USART = USART>,
	USART: stm32f1xx_hal::serial::Instance
	{
	SwitchToControlBaud(stm32::Reconfigure<'x, USART>),
	SendResetStrobe(stm32::WriteIntr<'x, stm32::Tx<USART>>),
	SwitchToDataBaud(stm32::Reconfigure<'x, USART>, bool),
}

#[pin_project]
pub struct Reset<'x, USART: 'static>
	where
	stm32::Tx<USART>: stm32::UsartTxSlot<USART = USART>,
	stm32::Rx<USART>: stm32::UsartRxSlot<USART = USART>,
	USART: stm32f1xx_hal::serial::Instance
{
	#[pin]
	ow: &'x mut OneWire<USART>,
	state: ResetState<'x, USART>,
}

impl<'x, USART: 'static> Future for Reset<'x, USART>
	where
	stm32::Tx<USART>: stm32::UsartTxSlot<USART = USART>,
	stm32::Rx<USART>: stm32::UsartRxSlot<USART = USART>,
	USART: stm32f1xx_hal::serial::Instance
{
	type Output = ();

	fn poll(self: Poll)
} */
