from __future__ import annotations

import threading
import time

try:
    from gpiozero import DigitalOutputDevice
except ImportError:
    DigitalOutputDevice = None


class TestPoints:
    def __init__(
        self,
        udprx_gpio: int,
        cmdtx_gpio: int,
        cantxa_gpio: int,
        cantxb_gpio: int,
        dega_gpio: int,
        degb_gpio: int,
        pulse_s: float,
        enabled: bool = True,
    ) -> None:
        self.enabled = enabled and DigitalOutputDevice is not None
        self.pulse_s = max(0.0001, pulse_s)

        self.udprx = None
        self.cmdtx = None
        self.cantxa = None
        self.cantxb = None
        self.dega = None
        self.degb = None

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._deadlines: dict[object, float] = {}
        self._worker: threading.Thread | None = None
        self._closed = False

        if not self.enabled:
            if enabled and DigitalOutputDevice is None:
                print("WARNING: gpiozero unavailable; test points disabled")
            return

        try:
            self.udprx = DigitalOutputDevice(
                udprx_gpio, active_high=True, initial_value=False
            )
            self.cmdtx = DigitalOutputDevice(
                cmdtx_gpio, active_high=True, initial_value=False
            )
            self.cantxa = DigitalOutputDevice(
                cantxa_gpio, active_high=True, initial_value=False
            )
            self.cantxb = DigitalOutputDevice(
                cantxb_gpio, active_high=True, initial_value=False
            )
            self.dega = DigitalOutputDevice(
                dega_gpio, active_high=True, initial_value=False
            )
            self.degb = DigitalOutputDevice(
                degb_gpio, active_high=True, initial_value=False
            )
        except Exception:
            self.close()
            raise

        self._worker = threading.Thread(
            target=self._finish_pulses,
            name="testpoint-pulses",
            daemon=True,
        )
        self._worker.start()

    def _finish_pulses(self) -> None:
        while not self._stop.wait(0.0005):
            now = time.monotonic()

            with self._lock:
                expired = [
                    pin
                    for pin, deadline in self._deadlines.items()
                    if now >= deadline
                ]

                for pin in expired:
                    pin.off()
                    del self._deadlines[pin]

    def _pulse(self, pin) -> None:
        if not self.enabled or pin is None:
            return

        with self._lock:
            if self._closed:
                return

            pin.on()
            self._deadlines[pin] = time.monotonic() + self.pulse_s

    def pulse_udprx(self) -> None:
        self._pulse(self.udprx)

    def pulse_cmdtx(self) -> None:
        self._pulse(self.cmdtx)

    def pulse_cantxa(self) -> None:
        self._pulse(self.cantxa)

    def pulse_cantxb(self) -> None:
        self._pulse(self.cantxb)

    def cmdtx_on(self) -> None:
        if not self.enabled or self.cmdtx is None:
            return

        with self._lock:
            if not self._closed:
                self._deadlines.pop(self.cmdtx, None)
                self.cmdtx.on()

    def cmdtx_off(self) -> None:
        if not self.enabled or self.cmdtx is None:
            return

        with self._lock:
            if not self._closed:
                self._deadlines.pop(self.cmdtx, None)
                self.cmdtx.off()

    def set_degraded(
        self,
        can_a_degraded: bool,
        can_b_degraded: bool,
    ) -> None:
        if not self.enabled:
            return

        with self._lock:
            if self._closed:
                return

            if self.dega is not None:
                self.dega.value = bool(can_a_degraded)

            if self.degb is not None:
                self.degb.value = bool(can_b_degraded)

    def close(self) -> None:
        self._stop.set()

        if self._worker is not None:
            self._worker.join(timeout=1.0)

        with self._lock:
            if self._closed:
                return

            self._closed = True
            self._deadlines.clear()

            for pin in (
                self.udprx,
                self.cmdtx,
                self.cantxa,
                self.cantxb,
                self.dega,
                self.degb,
            ):
                if pin is not None:
                    try:
                        pin.off()
                    finally:
                        pin.close()

            self.enabled = False