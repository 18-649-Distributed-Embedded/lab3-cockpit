from __future__ import annotations

import threading

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
    ) -> None:
        self.enabled = DigitalOutputDevice is not None
        self.pulse_s = pulse_s
        self._timers: dict[object, threading.Timer] = {}

        self.udprx = None
        self.cmdtx = None
        self.cantxa = None
        self.cantxb = None
        self.dega = None
        self.degb = None

        if not self.enabled:
            print("WARNING: gpiozero unavailable; test points disabled")
            return

        self.udprx = DigitalOutputDevice(
            udprx_gpio,
            active_high=True,
            initial_value=False,
        )
        self.cmdtx = DigitalOutputDevice(
            cmdtx_gpio,
            active_high=True,
            initial_value=False,
        )
        self.cantxa = DigitalOutputDevice(
            cantxa_gpio,
            active_high=True,
            initial_value=False,
        )
        self.cantxb = DigitalOutputDevice(
            cantxb_gpio,
            active_high=True,
            initial_value=False,
        )
        self.dega = DigitalOutputDevice(
            dega_gpio,
            active_high=True,
            initial_value=False,
        )
        self.degb = DigitalOutputDevice(
            degb_gpio,
            active_high=True,
            initial_value=False,
        )

    def _pulse(self, pin) -> None:
        if not self.enabled or pin is None:
            return

        old_timer = self._timers.pop(pin, None)

        if old_timer is not None:
            old_timer.cancel()

        pin.on()

        timer = threading.Timer(self.pulse_s, pin.off)
        timer.daemon = True
        self._timers[pin] = timer
        timer.start()

    def pulse_udprx(self) -> None:
        self.udprx_on()

    def udprx_on(self) -> None:
        if self.enabled and self.udprx is not None:
            self.udprx.on()

    def udprx_off(self) -> None:
        if self.enabled and self.udprx is not None:
            self.udprx.off()

    def pulse_cantxa(self) -> None:
        self.cantxa_on()

    def cantxa_on(self) -> None:
        if self.enabled and self.cantxa is not None:
            self.cantxa.on()

    def cantxa_off(self) -> None:
        if self.enabled and self.cantxa is not None:
            self.cantxa.off()

    def pulse_cantxb(self) -> None:
        self.cantxb_on()

    def cantxb_on(self) -> None:
        if self.enabled and self.cantxb is not None:
            self.cantxb.on()

    def cantxb_off(self) -> None:
        if self.enabled and self.cantxb is not None:
            self.cantxb.off()

    def cmdtx_on(self) -> None:
        if self.enabled and self.cmdtx is not None:
            self.cmdtx.on()

    def cmdtx_off(self) -> None:
        if self.enabled and self.cmdtx is not None:
            self.cmdtx.off()

    def set_degraded(self, can_a_degraded: bool, can_b_degraded: bool) -> None:
        if not self.enabled:
            return

        if self.dega is not None:
            self.dega.value = can_a_degraded

        if self.degb is not None:
            self.degb.value = can_b_degraded

    def close(self) -> None:
        for timer in self._timers.values():
            timer.cancel()

        self._timers.clear()

        for pin in (
            self.udprx,
            self.cmdtx,
            self.cantxa,
            self.cantxb,
            self.dega,
            self.degb,
        ):
            if pin is not None:
                pin.off()
                pin.close()