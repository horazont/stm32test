pub struct Crc8<const Poly: u8>(u8);

impl<const Poly: u8> Crc8<Poly> {
	pub const fn new() -> Self {
		Self(0)
	}

	pub fn reset(&mut self) {
		self.0 = 0;
	}

	pub fn feed(&mut self, byte: u8) -> u8 {
		for bit in 0..8 {
			if ((self.0 >> 7) & 1) != ((byte >> bit) & 1) {
				self.0 = (self.0 << 1) ^ Poly;
			} else {
				self.0 = self.0 << 1;
			}
		}
		self.0
	}

	#[inline]
	pub fn feed_buf(&mut self, buf: &[u8]) -> u8 {
		for byte in buf.iter() {
			self.feed(*byte);
		}
		self.0
	}

	#[inline]
	pub fn feed_array<const N: usize>(&mut self, arr: &[u8; N]) -> u8 {
		for i in 0..N {
			self.feed(arr[i]);
		}
		self.0
	}
}
