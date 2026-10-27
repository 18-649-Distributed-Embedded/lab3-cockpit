from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from can_catalog import Control


UDP_PACKET_SIZE = 276

COUNTER_OFFSET = 0
STEER_OFFSET = 4
THROTTLE_OFFSET = 8
BRAKE_OFFSET = 24
BUTTONS_OFFSET = 48
BUTTON_COUNT = 128

STEER_MIN = -32768
STEER_CENTER = 0
STEER_MAX = 32767

THROTTLE_RELEASED = 0
THROTTLE_PRESSED = 32767
BRAKE_RELEASED = 0
BRAKE_PRESSED = 32767

STEERING_DEADBAND = 1200
PEDAL_DEADBAND = 1000

BUTTON_LEFT_BLINKER = 9
BUTTON_RIGHT_BLINKER = 8
BUTTON_HAZARD = 7
BUTTON_TEST = 6

TURN_CANCEL_THRESHOLD = 12000
TURN_CANCEL_RETURN_THRESHOLD = 4000

TEST_DEBOUNCE_S = 0.050
TEST_DOUBLE_PRESS_S = 0.500


@dataclass(frozen=True)
class WheelState:
    counter: int
    steering_raw: int
    throttle_raw: int
    brake_raw: int
    buttons: tuple[int, ...]

    def button_pressed(self, index: int) -> bool:
        return (
            0 <= index < len(self.buttons)
            and self.buttons[index] != 0
        )


@dataclass
class CockpitState:
    throttle: int = 127
    steering: int = 0
    brake: int = 1
    blinker: int = 0
    test_active: bool = False

    previous_buttons: tuple[int, ...] | None = None
    left_latched: bool = False
    right_latched: bool = False
    left_crossed: bool = False
    right_crossed: bool = False

    last_test_edge_time: float | None = None
    test_pair_start_time: float | None = None


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def map_linear(
    value: int,
    in_low: int,
    in_high: int,
    out_low: int,
    out_high: int,
) -> int:
    if in_low == in_high:
        raise ValueError("input range cannot be zero")

    value = clamp(value, min(in_low, in_high), max(in_low, in_high))

    return round(
        out_low
        + (value - in_low)
        * (out_high - out_low)
        / (in_high - in_low)
    )


def parse_packet(packet: bytes) -> WheelState:
    if len(packet) != UDP_PACKET_SIZE:
        raise ValueError(
            f"expected {UDP_PACKET_SIZE}-byte UDP packet, "
            f"got {len(packet)}"
        )

    return WheelState(
        counter=struct.unpack_from("<I", packet, COUNTER_OFFSET)[0],
        steering_raw=struct.unpack_from("<i", packet, STEER_OFFSET)[0],
        throttle_raw=struct.unpack_from("<i", packet, THROTTLE_OFFSET)[0],
        brake_raw=struct.unpack_from("<i", packet, BRAKE_OFFSET)[0],
        buttons=tuple(
            packet[BUTTONS_OFFSET:BUTTONS_OFFSET + BUTTON_COUNT]
        ),
    )


def normalize_steering(raw: int) -> int:
    if abs(raw - STEER_CENTER) <= STEERING_DEADBAND:
        return 0

    if raw >= STEER_CENTER:
        return clamp(
            map_linear(raw, STEER_CENTER, STEER_MAX, 0, 32767),
            0,
            32767,
        )

    return clamp(
        map_linear(raw, STEER_MIN, STEER_CENTER, -32768, 0),
        -32768,
        0,
    )


def pedal_fraction(raw: int, released: int, pressed: int) -> float:
    if released == pressed:
        raise ValueError("pedal calibration endpoints cannot match")

    raw = clamp(raw, min(released, pressed), max(released, pressed))
    fraction = (raw - released) / (pressed - released)

    return max(0.0, min(1.0, fraction))


def normalize_throttle(raw: int) -> int:
    fraction = pedal_fraction(
        raw,
        THROTTLE_RELEASED,
        THROTTLE_PRESSED,
    )

    if (
        fraction * abs(THROTTLE_PRESSED - THROTTLE_RELEASED)
        <= PEDAL_DEADBAND
    ):
        fraction = 0.0

    return clamp(round(127 + fraction * 128), 127, 255)


def normalize_brake(raw: int) -> int:
    fraction = pedal_fraction(
        raw,
        BRAKE_RELEASED,
        BRAKE_PRESSED,
    )

    if (
        fraction * abs(BRAKE_PRESSED - BRAKE_RELEASED)
        <= PEDAL_DEADBAND
    ):
        return 0

    return 1 if fraction >= 0.10 else 0


def rising_edge(
    current: WheelState,
    previous_buttons: tuple[int, ...] | None,
    button_index: int,
) -> bool:
    if not current.button_pressed(button_index):
        return False

    if previous_buttons is None:
        return True

    return previous_buttons[button_index] == 0


def update_from_packet(
    state: CockpitState,
    packet: bytes,
) -> tuple[Control, int, bool]:
    wheel = parse_packet(packet)

    state.throttle = normalize_throttle(wheel.throttle_raw)
    state.steering = normalize_steering(wheel.steering_raw)
    state.brake = normalize_brake(wheel.brake_raw)

    if rising_edge(
        wheel, state.previous_buttons, BUTTON_LEFT_BLINKER
    ):
        state.blinker = 0 if state.blinker == 1 else 1
        state.left_latched = state.blinker == 1
        state.right_latched = False
        state.left_crossed = False
        state.right_crossed = False

    if rising_edge(
        wheel, state.previous_buttons, BUTTON_RIGHT_BLINKER
    ):
        state.blinker = 0 if state.blinker == 2 else 2
        state.right_latched = state.blinker == 2
        state.left_latched = False
        state.left_crossed = False
        state.right_crossed = False

    if rising_edge(
        wheel, state.previous_buttons, BUTTON_HAZARD
    ):
        state.blinker = 0 if state.blinker == 3 else 3
        state.left_latched = False
        state.right_latched = False

    test_fault_requested = False

    if rising_edge(wheel, state.previous_buttons, BUTTON_TEST):
        now = time.monotonic()

        accepted = (
            state.last_test_edge_time is None
            or now - state.last_test_edge_time >= TEST_DEBOUNCE_S
        )

        if accepted:
            state.last_test_edge_time = now

            if not state.test_active:
                state.test_active = True
                state.test_pair_start_time = now
                test_fault_requested = True
            elif (
                state.test_pair_start_time is not None
                and now - state.test_pair_start_time
                <= TEST_DOUBLE_PRESS_S
            ):
                state.test_active = False
                state.test_pair_start_time = None
            else:
                state.test_pair_start_time = now

    if state.left_latched:
        if wheel.steering_raw <= -TURN_CANCEL_THRESHOLD:
            state.left_crossed = True
        elif (
            state.left_crossed
            and wheel.steering_raw >= -TURN_CANCEL_RETURN_THRESHOLD
        ):
            state.blinker = 0
            state.left_latched = False
            state.left_crossed = False

    if state.right_latched:
        if wheel.steering_raw >= TURN_CANCEL_THRESHOLD:
            state.right_crossed = True
        elif (
            state.right_crossed
            and wheel.steering_raw <= TURN_CANCEL_RETURN_THRESHOLD
        ):
            state.blinker = 0
            state.right_latched = False
            state.right_crossed = False

    state.previous_buttons = wheel.buttons

    buttons = 0

    if state.blinker == 1:
        buttons |= 0x01
    elif state.blinker == 2:
        buttons |= 0x02
    elif state.blinker == 3:
        buttons |= 0x04

    if state.test_active:
        buttons |= 0x08

    control = Control(
        steering=state.steering,
        throttle=state.throttle,
        brake=state.brake,
        buttons=buttons,
    )

    return control, state.blinker, test_fault_requested