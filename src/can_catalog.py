from __future__ import annotations

from dataclasses import dataclass


CAN_ID_CTRL = 0x010
CAN_ID_FAULT = 0x020
CAN_ID_HBN1 = 0x100
CAN_ID_HBN2 = 0x101
CAN_ID_HBN3 = 0x102
CAN_ID_BLINK = 0x200
CAN_ID_FORCE = 0x300

CAN_ZONE_N1 = 0x01
CAN_ZONE_N2 = 0x02
CAN_ZONE_N3 = 0x03

CAN_ZONE_STATE_OK = 0x00
CAN_ZONE_STATE_ERROR = 0x01

CAN_FAULT_NONE = 0x00
CAN_FAULT_TESTBTN = 0x01
CAN_FAULT_STALE_UDP = 0x02
CAN_FAULT_BAD_CTRL = 0x03

BLINK_OFF = 0x00
BLINK_LEFT = 0x01
BLINK_RIGHT = 0x02
BLINK_HAZARD = 0x03


@dataclass(frozen=True)
class Control:
    steering: int
    throttle: int
    brake: int
    buttons: int


def crc8(data: bytes) -> int:
    crc = 0x00

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


def byte0(flags: int, sequence: int) -> int:
    return ((flags & 0x0F) << 4) | (sequence & 0x0F)


def make_ctrl(sequence: int, control: Control, flags: int = 0) -> bytes:
    if not -32768 <= control.steering <= 32767:
        raise ValueError("steering must fit signed int16")

    if not 0 <= control.throttle <= 255:
        raise ValueError("throttle must fit uint8")

    if not 0 <= control.brake <= 255:
        raise ValueError("brake must fit uint8")

    if not 0 <= control.buttons <= 255:
        raise ValueError("buttons must fit uint8")

    steering_bytes = control.steering.to_bytes(
        2,
        byteorder="big",
        signed=True,
    )

    payload_without_crc = bytes(
        [
            byte0(flags, sequence),
            steering_bytes[0],
            steering_bytes[1],
            control.throttle,
            control.brake,
            control.buttons,
            0x00,
        ]
    )

    return payload_without_crc + bytes([crc8(payload_without_crc)])


def make_fault(
    sequence: int,
    zone_id: int,
    reason: int,
    flags: int = 0,
) -> bytes:
    return bytes(
        [
            byte0(flags, sequence),
            zone_id & 0xFF,
            reason & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )


def make_hbn1(
    sequence: int,
    zone_state: int,
    bus_a_healthy: bool,
    bus_b_healthy: bool,
    flags: int = 0,
) -> bytes:
    health_bits = 0

    if bus_a_healthy:
        health_bits |= 0x01

    if bus_b_healthy:
        health_bits |= 0x02

    return bytes(
        [
            byte0(flags, sequence),
            zone_state & 0xFF,
            health_bits,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )


def make_blink(
    sequence: int,
    blink_state: int,
    phase_bit: int,
    flags: int = 0,
) -> bytes:
    if blink_state not in {
        BLINK_OFF,
        BLINK_LEFT,
        BLINK_RIGHT,
        BLINK_HAZARD,
    }:
        raise ValueError("invalid blink state")

    return bytes(
        [
            byte0(flags, sequence),
            blink_state & 0xFF,
            phase_bit & 0x01,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]
    )


def can_sequence(payload: bytes) -> int:
    if not payload:
        raise ValueError("CAN payload is empty")

    return payload[0] & 0x0F