from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import time
from collections import defaultdict
from pathlib import Path

import can

from can_catalog import (
    BLINK_HAZARD,
    BLINK_OFF,
    CAN_FAULT_NONE,
    CAN_FAULT_STALE_UDP,
    CAN_FAULT_TESTBTN,
    CAN_ID_BLINK,
    CAN_ID_CTRL,
    CAN_ID_FAULT,
    CAN_ID_FORCE,
    CAN_ID_HBN1,
    CAN_ID_HBN2,
    CAN_ID_HBN3,
    CAN_ZONE_N1,
    CAN_ZONE_N2,
    CAN_ZONE_N3,
    CAN_ZONE_STATE_ERROR,
    CAN_ZONE_STATE_OK,
    Control,
    can_sequence,
    make_blink,
    make_ctrl,
    make_fault,
    make_hbn1,
)
from testpoints import TestPoints
from udp_protocol import CockpitState, update_from_packet


PEER_IDS = (CAN_ID_HBN2, CAN_ID_HBN3)
PEER_ZONES = (CAN_ZONE_N2, CAN_ZONE_N3)

MAX_UDP_BATCH = 16
MAX_CAN_BATCH = 64


def load_env(path: str) -> None:
    env_path = Path(path)

    if not env_path.exists():
        raise FileNotFoundError(
            f"configuration file not found: {env_path}"
        )

    for line in env_path.read_text().splitlines():
        line = line.strip()

        if not line or line.startswith("#") or "=" not in line:
            continue

        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip())


def env_int(name: str, default: int) -> int:
    return int(os.getenv(name, str(default)))


def env_float(name: str, default: float) -> float:
    return float(os.getenv(name, str(default)))


def next_sequence(counters: dict[int, int], can_id: int) -> int:
    sequence = counters[can_id] & 0x0F
    counters[can_id] = (sequence + 1) & 0x0F
    return sequence


def classify_pi_network(
    n2_a: bool,
    n2_b: bool,
    n3_a: bool,
    n3_b: bool,
) -> tuple[str, str, bool, bool]:
    isolated = not any((n2_a, n2_b, n3_a, n3_b))

    n2_lost = not (n2_a or n2_b)
    n3_lost = not (n3_a or n3_b)
    network_error = n2_lost or n3_lost

    if network_error:
        return "steady", "steady", True, isolated

    if not (n2_a or n3_a):
        mode_a = "steady"
    elif not (n2_a and n3_a):
        mode_a = "blink_1hz"
    else:
        mode_a = "off"

    if not (n2_b or n3_b):
        mode_b = "steady"
    elif not (n2_b and n3_b):
        mode_b = "blink_1hz"
    else:
        mode_b = "off"

    return mode_a, mode_b, False, False


def led_level(mode: str, now: float) -> bool:
    if mode == "steady":
        return True
    if mode == "blink_1hz":
        return int(now * 2.0) % 2 == 0
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Lab 3 Raspberry Pi cockpit CAN gateway"
    )
    parser.add_argument("--env", default="config/cockpit.env")
    parser.add_argument("--no-gpio", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        load_env(args.env)
    except FileNotFoundError as error:
        print(error, file=sys.stderr)
        return 1

    udp_bind = os.getenv("UDP_BIND", "0.0.0.0")
    udp_port = env_int("UDP_PORT", 8000)
    udp_packet_size = env_int("UDP_PACKET_SIZE", 276)
    udp_timeout_s = env_float("UDP_TIMEOUT_S", 0.100)

    can_a_name = os.getenv("CAN_A", "can0")
    can_b_name = os.getenv("CAN_B", "can1")

    heartbeat_period_s = env_float("HEARTBEAT_PERIOD_S", 0.020)
    heartbeat_timeout_s = env_float("HEARTBEAT_TIMEOUT_S", 0.060)
    blink_toggle_period_s = env_float(
        "BLINK_TOGGLE_PERIOD_S", 0.500
    )
    hazard_toggle_period_s = env_float(
        "HAZARD_TOGGLE_PERIOD_S", 0.250
    )
    health_poll_s = env_float("HEALTH_POLL_S", 0.005)

    periods = (
        udp_timeout_s,
        heartbeat_period_s,
        heartbeat_timeout_s,
        blink_toggle_period_s,
        hazard_toggle_period_s,
        health_poll_s,
    )

    if any(value <= 0.0 for value in periods):
        print("All configured periods must be positive", file=sys.stderr)
        return 1

    if can_a_name == can_b_name:
        print("CAN_A and CAN_B must be distinct", file=sys.stderr)
        return 1

    if udp_packet_size != 276:
        print(
            "udp_protocol.py expects UDP_PACKET_SIZE=276",
            file=sys.stderr,
        )
        return 1

    testpoints = None
    udp_socket = None
    bus_a = None
    bus_b = None

    try:
        testpoints = TestPoints(
            udprx_gpio=env_int("UDPRX_GPIO", 23),
            cmdtx_gpio=env_int("CMDTX_GPIO", 18),
            cantxa_gpio=env_int("CANTXA_GPIO", 22),
            cantxb_gpio=env_int("CANTXB_GPIO", 27),
            dega_gpio=env_int("DEGA_GPIO", 5),
            degb_gpio=env_int("DEGB_GPIO", 6),
            pulse_s=env_float("TESTPOINT_PULSE_S", 0.001),
            enabled=not args.no_gpio,
        )

        udp_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )
        udp_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )
        udp_socket.bind((udp_bind, udp_port))
        udp_socket.setblocking(False)

        bus_a = can.Bus(
            interface="socketcan", channel=can_a_name
        )
        bus_b = can.Bus(
            interface="socketcan", channel=can_b_name
        )

        buses = (
            (can_a_name, bus_a, testpoints.pulse_cantxa),
            (can_b_name, bus_b, testpoints.pulse_cantxb),
        )

        counters: dict[int, int] = defaultdict(int)
        last_log_time: dict[tuple, float] = {}
        last_tx_ok = {
            can_a_name: None,
            can_b_name: None,
        }

        cockpit_state = CockpitState()
        latest_control = Control(
            steering=0,
            throttle=127,
            brake=1,
            buttons=0,
        )
        requested_blink = BLINK_OFF

        last_udp_time: float | None = None

        last_peer_heartbeat = {
            name: {can_id: None for can_id in PEER_IDS}
            for name in (can_a_name, can_b_name)
        }

        peer_faults = {
            zone_id: CAN_FAULT_NONE
            for zone_id in PEER_ZONES
        }

        fault_last_sequence: dict[int, tuple[int, float]] = {}

        start_time = time.monotonic()
        next_heartbeat_time = start_time
        next_blink_toggle_time = float("inf")

        effective_blink: int | None = None
        blink_phase = 0
        previous_tx_enabled = False
        announced_local_reason = CAN_FAULT_NONE
        previous_status = None

        def limited_log(key: tuple, text: str) -> None:
            now = time.monotonic()

            if now - last_log_time.get(key, float("-inf")) >= 1.0:
                print(text, file=sys.stderr)
                last_log_time[key] = now

        def send_dual(
            can_id: int,
            payload: bytes,
            mark_command: bool = False,
        ) -> None:
            for index, (name, bus, pulse_tx) in enumerate(buses):
                message = can.Message(
                    arbitration_id=can_id,
                    data=payload,
                    is_extended_id=False,
                )

                try:
                    pulse_tx()
                    bus.send(message, timeout=0.005)
                except can.CanError as error:
                    last_tx_ok[name] = False
                    limited_log(
                        ("tx", name),
                        f"{name} TX failed for "
                        f"0x{can_id:03X}: {error}",
                    )
                else:
                    if mark_command and index == 0:
                        testpoints.pulse_cmdtx()

                    if last_tx_ok[name] is False and args.verbose:
                        print(f"{name} TX queuing recovered")

                    last_tx_ok[name] = True

        def send_fault(reason: int) -> None:
            send_dual(
                CAN_ID_FAULT,
                make_fault(
                    sequence=next_sequence(counters, CAN_ID_FAULT),
                    zone_id=CAN_ZONE_N1,
                    reason=reason,
                ),
            )

        def heard(name: str, can_id: int, now: float) -> bool:
            timestamp = last_peer_heartbeat[name][can_id]

            return (
                timestamp is not None
                and now - timestamp <= heartbeat_timeout_s
            )

        def network_snapshot(now: float):
            n2_a = heard(can_a_name, CAN_ID_HBN2, now)
            n2_b = heard(can_b_name, CAN_ID_HBN2, now)
            n3_a = heard(can_a_name, CAN_ID_HBN3, now)
            n3_b = heard(can_b_name, CAN_ID_HBN3, now)

            mode_a, mode_b, error, isolated = classify_pi_network(
                n2_a, n2_b, n3_a, n3_b
            )

            return (
                mode_a,
                mode_b,
                error,
                isolated,
                n2_a or n3_a,
                n2_b or n3_b,
            )

        def system_error_at(now: float) -> bool:
            udp_stale = (
                last_udp_time is None
                or now - last_udp_time > udp_timeout_s
            )
            network_error = network_snapshot(now)[2]

            return (
                udp_stale
                or cockpit_state.test_active
                or network_error
                or any(peer_faults.values())
            )

        def process_can_rx() -> None:
            for name, bus, _ in buses:
                for _ in range(MAX_CAN_BATCH):
                    try:
                        message = bus.recv(timeout=0.0)
                    except can.CanError as error:
                        limited_log(
                            ("rx", name),
                            f"{name} RX failed: {error}",
                        )
                        break

                    if message is None:
                        break

                    if (
                        message.is_error_frame
                        or message.is_remote_frame
                        or message.is_extended_id
                        or message.dlc != 8
                        or len(message.data) != 8
                    ):
                        continue

                    now = time.monotonic()
                    can_id = message.arbitration_id
                    payload = bytes(message.data)

                    if can_id in PEER_IDS:
                        last_peer_heartbeat[name][can_id] = now

                    elif can_id == CAN_ID_FAULT:
                        zone_id = payload[1]
                        reason = payload[2]

                        if zone_id in peer_faults:
                            sequence = can_sequence(payload)
                            previous = fault_last_sequence.get(zone_id)

                            duplicate = (
                                previous is not None
                                and previous[0] == sequence
                                and now - previous[1] <= 0.100
                            )

                            if not duplicate:
                                fault_last_sequence[zone_id] = (
                                    sequence, now
                                )
                                peer_faults[zone_id] = reason

                                if args.verbose:
                                    print(
                                        f"Peer FAULT zone={zone_id} "
                                        f"reason={reason}"
                                    )

                    elif can_id == CAN_ID_FORCE and args.verbose:
                        force = int.from_bytes(
                            payload[1:2], "big", signed=True
                        )
                        print(f"FORCE on {name}: {force}")

        print(f"Listening for wheel UDP on {udp_bind}:{udp_port}")
        print(
            f"Sending Lab 3 CAN traffic on "
            f"{can_a_name} and {can_b_name}"
        )

        while True:
            now = time.monotonic()
            deadlines = [
                next_heartbeat_time,
                now + health_poll_s,
            ]

            if next_blink_toggle_time != float("inf"):
                deadlines.append(next_blink_toggle_time)

            if last_udp_time is not None:
                stale_deadline = last_udp_time + udp_timeout_s
                if stale_deadline > now:
                    deadlines.append(stale_deadline)

            timeout = max(0.0, min(deadlines) - now)

            readable, _, _ = select.select(
                [udp_socket], [], [], timeout
            )

            process_can_rx()

            if readable:
                for _ in range(MAX_UDP_BATCH):
                    try:
                        packet, source = udp_socket.recvfrom(2048)
                    except BlockingIOError:
                        break

                    testpoints.pulse_udprx()

                    if len(packet) != udp_packet_size:
                        limited_log(
                            ("udp-size",),
                            f"Ignored {len(packet)}-byte UDP packet "
                            f"from {source}",
                        )
                        continue

                    try:
                        (
                            latest_control,
                            requested_blink,
                            test_fault_requested,
                        ) = update_from_packet(cockpit_state, packet)
                    except ValueError as error:
                        limited_log(
                            ("udp-invalid",),
                            f"UDP packet rejected: {error}",
                        )
                        continue

                    now = time.monotonic()
                    last_udp_time = now

                    if test_fault_requested:
                        if announced_local_reason != CAN_FAULT_TESTBTN:
                            send_fault(CAN_FAULT_TESTBTN)
                            announced_local_reason = CAN_FAULT_TESTBTN

                    if cockpit_state.test_active:
                        continue

                    if announced_local_reason != CAN_FAULT_NONE:
                        send_fault(CAN_FAULT_NONE)
                        announced_local_reason = CAN_FAULT_NONE

                    error_active = system_error_at(now)

                    control = Control(
                        steering=latest_control.steering,
                        throttle=(
                            127 if error_active
                            else latest_control.throttle
                        ),
                        brake=(
                            1 if error_active
                            else latest_control.brake
                        ),
                        buttons=latest_control.buttons,
                    )

                    send_dual(
                        CAN_ID_CTRL,
                        make_ctrl(
                            next_sequence(counters, CAN_ID_CTRL),
                            control,
                        ),
                        mark_command=True,
                    )

                    if args.verbose:
                        print(
                            f"UDP accepted from {source}: "
                            f"steering={control.steering} "
                            f"throttle={control.throttle} "
                            f"brake={control.brake} "
                            f"requested_blink={requested_blink}"
                        )

            now = time.monotonic()

            udp_stale = (
                last_udp_time is None
                or now - last_udp_time > udp_timeout_s
            )

            if cockpit_state.test_active:
                local_reason = CAN_FAULT_TESTBTN
            elif udp_stale:
                local_reason = CAN_FAULT_STALE_UDP
            else:
                local_reason = CAN_FAULT_NONE

            if local_reason != announced_local_reason:
                if local_reason != CAN_FAULT_NONE:
                    send_fault(local_reason)
                else:
                    send_fault(CAN_FAULT_NONE)

                announced_local_reason = local_reason

            (
                mode_a,
                mode_b,
                network_error,
                isolated,
                bus_a_healthy,
                bus_b_healthy,
            ) = network_snapshot(now)

            if now - start_time < heartbeat_timeout_s:
                mode_a = "off"
                mode_b = "off"

            testpoints.set_degraded(
                can_a_degraded=led_level(mode_a, now),
                can_b_degraded=led_level(mode_b, now),
            )

            peer_fault_active = any(peer_faults.values())
            local_failed = local_reason != CAN_FAULT_NONE

            system_error_active = (
                local_failed
                or network_error
                or peer_fault_active
            )

            tx_enabled = not local_failed
            desired_blink = (
                BLINK_HAZARD
                if system_error_active
                else requested_blink
            )

            if tx_enabled:
                if (
                    not previous_tx_enabled
                    or effective_blink != desired_blink
                ):
                    effective_blink = desired_blink
                    blink_phase = (
                        0 if effective_blink == BLINK_OFF else 1
                    )

                    send_dual(
                        CAN_ID_BLINK,
                        make_blink(
                            next_sequence(counters, CAN_ID_BLINK),
                            effective_blink,
                            blink_phase,
                        ),
                    )

                    if effective_blink == BLINK_OFF:
                        next_blink_toggle_time = float("inf")
                    else:
                        period = (
                            hazard_toggle_period_s
                            if effective_blink == BLINK_HAZARD
                            else blink_toggle_period_s
                        )
                        next_blink_toggle_time = (
                            time.monotonic() + period
                        )

                elif now >= next_blink_toggle_time:
                    blink_phase ^= 1

                    send_dual(
                        CAN_ID_BLINK,
                        make_blink(
                            next_sequence(counters, CAN_ID_BLINK),
                            effective_blink,
                            blink_phase,
                        ),
                    )

                    period = (
                        hazard_toggle_period_s
                        if effective_blink == BLINK_HAZARD
                        else blink_toggle_period_s
                    )
                    next_blink_toggle_time = (
                        time.monotonic() + period
                    )
            else:
                next_blink_toggle_time = float("inf")

            if now >= next_heartbeat_time:
                if tx_enabled:
                    send_dual(
                        CAN_ID_HBN1,
                        make_hbn1(
                            sequence=next_sequence(
                                counters, CAN_ID_HBN1
                            ),
                            zone_state=(
                                CAN_ZONE_STATE_ERROR
                                if system_error_active
                                else CAN_ZONE_STATE_OK
                            ),
                            bus_a_healthy=bus_a_healthy,
                            bus_b_healthy=bus_b_healthy,
                        ),
                    )

                finished = time.monotonic()
                skipped = int(
                    (finished - next_heartbeat_time)
                    / heartbeat_period_s
                ) + 1
                next_heartbeat_time += (
                    skipped * heartbeat_period_s
                )

            status = (
                mode_a,
                mode_b,
                network_error,
                isolated,
                local_reason,
                peer_fault_active,
            )

            if args.verbose and status != previous_status:
                print(
                    f"Health DEG_A={mode_a} DEG_B={mode_b} "
                    f"network_error={network_error} "
                    f"isolated={isolated} "
                    f"local_reason={local_reason} "
                    f"peer_fault={peer_fault_active} "
                    f"heartbeat_enabled={tx_enabled}"
                )
                previous_status = status

            previous_tx_enabled = tx_enabled

    except KeyboardInterrupt:
        print("Stopping cockpit gateway")
        return 0

    except (can.CanError, OSError, ValueError) as error:
        print(f"Cockpit error: {error}", file=sys.stderr)
        return 1

    finally:
        if udp_socket is not None:
            udp_socket.close()

        if bus_a is not None:
            bus_a.shutdown()

        if bus_b is not None:
            bus_b.shutdown()

        if testpoints is not None:
            testpoints.close()


if __name__ == "__main__":
    raise SystemExit(main())