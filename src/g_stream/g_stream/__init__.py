from typing import Callable, List

class Event:
    def __init__(self):
        self._callbacks: List[Callable] = []

    def __iadd__(self, callback: Callable):
        """Override += operator to register a callback."""
        if callable(callback):
            self._callbacks.append(callback)
        else:
            raise TypeError("Callback must be a callable function or method.")
        return self

    def fire(self, *args, **kwargs):
        """Invoke all registered callbacks with provided arguments."""
        for callback in self._callbacks:
            callback(*args, **kwargs)