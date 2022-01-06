use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};

pub struct SpinCell<T> {
	lock: AtomicBool,
	inner: UnsafeCell<T>,
}

#[derive(Debug)]
pub enum TryBorrowError {
	InUse,
}

pub struct RefMut<'x, B> {
	value: &'x mut B,
	flag: &'x AtomicBool,
}

impl<T> SpinCell<T> {
	pub const fn new(inner: T) -> Self {
		Self {
			lock: AtomicBool::new(false),
			inner: UnsafeCell::new(inner),
		}
	}
}

impl<T> SpinCell<T> {
	pub fn try_borrow_mut<'x>(&'x self) -> Result<RefMut<'x, T>, TryBorrowError> {
		match self
			.lock
			.compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
		{
			Err(_) => Err(TryBorrowError::InUse),
			Ok(_) => Ok(RefMut {
				flag: &self.lock,
				// SAFETY: protected by the lock, lifetime is explicitly ensured.
				value: unsafe { &mut *self.inner.get() },
			}),
		}
	}
}

// SAFETY: Any concurrent access to the data inside the SpinCell is protected by the lock.
unsafe impl<T> Send for SpinCell<T> {}

// SAFETY: Any concurrent access to the data inside the SpinCell is protected by the lock.
unsafe impl<T> Sync for SpinCell<T> {}

impl<'x, T> RefMut<'x, T> {
	pub fn map<U, F: FnOnce(&'x mut T) -> &'x mut U>(self, f: F) -> RefMut<'x, U> {
		RefMut {
			// SAFETY: protected by the lock, lifetime is explicitly ensured
			value: f(unsafe { &mut *(self.value as *mut T) }),
			flag: self.flag,
		}
	}

	/// Drop the borrow and return the inner reference. Grossly unsafe.
	pub unsafe fn steal(self) -> &'x mut T {
		self.flag.store(false, Ordering::Release);
		&mut *(self.value as *mut T)
	}
}

impl<'x, T> Deref for RefMut<'x, T> {
	type Target = T;

	fn deref(&self) -> &T {
		self.value
	}
}

impl<'x, T> DerefMut for RefMut<'x, T> {
	fn deref_mut(&mut self) -> &mut T {
		self.value
	}
}

impl<'x, T> Drop for RefMut<'x, T> {
	fn drop(&mut self) {
		self.flag.store(false, Ordering::Release);
	}
}

// SAFETY: the RefMut indicates unique and sole ownership of the data, and it can be transferred across thread boundaries because the ownership indication is behind an atomic.
unsafe impl<'x, T> Send for RefMut<'x, T> {}
