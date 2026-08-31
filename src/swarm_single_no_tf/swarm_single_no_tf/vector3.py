import math
from typing import Union, Callable, List

class Vector3:
	"""
	A class to represent a 3D vector and perform common vector operations.
	This version includes a callback system to notify listeners of changes.
	"""
	def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
		self._x = float(x)
		self._y = float(y)
		self._z = float(z)
		self._callbacks: List[Callable[['Vector3'], None]] = []

	# --- Listener/Callback Management ---
	def add_change_listener(self, callback: Callable[['Vector3'], None]):
		"""Registers a function to be called when any component changes."""
		self._callbacks.append(callback)

	def remove_change_listener(self, callback: Callable[['Vector3'], None]):
		"""Unregisters a callback function."""
		try:
			self._callbacks.remove(callback)
		except ValueError:
			# Callback was not in the list, ignore.
			pass

	def _notify_listeners(self):
		"""Calls all registered callbacks, passing self as an argument."""
		for callback in self._callbacks:
			callback(self)

	# --- Getters ---
	@property
	def x(self) -> float:
		return self._x

	@property
	def y(self) -> float:
		return self._y

	@property
	def z(self) -> float:
		return self._z

	@property
	def xyz(self) -> dict[float, float, float]:
		"""Returns all components as a tuple."""
		return [self._x, self._y, self._z]

	# --- Setters ---
	@x.setter
	def x(self, value: Union[int, float]):
		# Call the main 'set' method to consolidate logic
		self.set(float(value), self.y, self.z)

	@y.setter
	def y(self, value: Union[int, float]):
		# Call the main 'set' method to consolidate logic
		self.set(self.x, float(value), self.z)

	@z.setter
	def z(self, value: Union[int, float]):
		# Call the main 'set' method to consolidate logic
		self.set(self.x, self.y, float(value))
		
	def set(self, x: float, y: float, z: float, disable_notify: bool = False):
		"""
		Convenience method to set all components at once.
		This is the single point of modification and notification.
		"""
		if self._x != x or self._y != y or self._z != z:
			self._x = float(x)
			self._y = float(y)
			self._z = float(z)
			if not disable_notify:
				self._notify_listeners()

	def __repr__(self) -> str:
		"""String representation for printing."""
		return f"Vector3(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"

	# ... (Vector math operators like __add__, __sub__, etc
	
	def __add__(self, other: 'Vector3') -> 'Vector3': return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
	
	def __sub__(self, other: 'Vector3') -> 'Vector3': return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
	
	def __mul__(self, other: Union[int, float, 'Vector3']) -> 'Vector3':
		if isinstance(other, Vector3):
			return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
		return Vector3(self.x * other, self.y * other, self.z * other)
	
	def __rmul__(self, scalar: Union[int, float]) -> 'Vector3': return self.__mul__(scalar)
	
	def __truediv__(self, scalar: Union[int, float]) -> 'Vector3':
		if scalar == 0: raise ValueError("Cannot divide by zero.")
		return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)
	
	def magnitude(self) -> float: return math.sqrt(self.x**2 + self.y**2 + self.z**2)
	
	def squared_magnitude(self) -> float: return self.x**2 + self.y**2 + self.z**2
	
	def squared(self) -> 'Vector3': return Vector3(self.x**2, self.y**2, self.z**2)

	def inverse(self) -> 'Vector3':
		v = Vector3()
		if self.x != 0: v.x = self.x**-1
		if self.y != 0: v.y = self.y**-1
		if self.z != 0: v.z = self.z**-1
		return v

	def sign(self) -> 'Vector3':
		v = Vector3()
		if self.x != 0: v.x = self.x / abs(self.x)
		if self.y != 0: v.y = self.y / abs(self.y)
		if self.z != 0: v.z = self.z / abs(self.z)
		return v

	def normalized(self) -> 'Vector3':
		mag = self.magnitude()
		if mag == 0: return Vector3()
		return self / mag