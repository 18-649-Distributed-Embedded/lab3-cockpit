from __future__ import annotations

import argparse
import os
import select
import socket
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

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


def monotonic_ns() -> int:
    return time.monotonic_ns()


def load_env(path: str) -> None:
    env_path = Path(path)

    if not env_path.exists():
        raise FileNotFoundError(f"configuration file not found: {env_path}")

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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Lab 3 Raspberry Pi cockpit CAN gateway",
    )
    parser.add_argument(
        "--env",
        default="config/cockpit.env",
        help="path to cockpit environment file",
    )
    parser.add_argument(
        "--no-gpio",
        action="store_true",
        help="disable GPIO test-point output",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="print received UDP and CAN status messages",
    )
    args = parser.parse_args()

    try:
        load_env(args.env)
    except FileNotFoundError as error:
        print(error, file=sys.stderr)
        return 1

    udp_bind = os.getenv("UDP_BIND", "0.0.0.0")
    udp_port = env_int("UDP_PORT", 8000)
    udp_packet_size = env_int("UDP_PACKET_SIZE", 276)
    udp_timeout_s = env_float("UDP_TIMEOUT_S", 0.150)

    can_a_name = os.getenv("CAN_A", "can0")
    can_b_name = os.getenv("CAN_B", "can1")

    heartbeat_period_s = env_float("HEARTBEAT_PERIOD_S", 0.020)
    control_period_s = env_float("CONTROL_PERIOD_S", 0.020)
    blink_toggle_period_s = env_float("BLINK_TOGGLE_PERIOD_S", 0.500)
    heartbeat_timeout_s = env_float("HEARTBEAT_TIMEOUT_S", 0.060)
    testpoint_pulse_s = env_float("TESTPOINT_PULSE_S", 0.001)

    testpoints = TestPoints(
        udprx_gpio=env_int("UDPRX_GPIO", 23),
        cmdtx_gpio=env_int("CMDTX_GPIO", 18),
        cantxa_gpio=env_int("CANTXA_GPIO", 22),
        cantxb_gpio=env_int("CANTXB_GPIO", 27),
        dega_gpio=env_int("DEGA_GPIO", 5),
        degb_gpio=env_int("DEGB_GPIO", 6),
        pulse_s=testpoint_pulse_s,
    )

    if args.no_gpio:
        testpoints.close()
        testpoints.enabled = False

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((udp_bind, udp_port))
    udp_socket.setblocking(False)

    open_buses = []
    for ch_name, pin in ((can_a_name, testpoints.cantxa), (can_b_name, testpoints.cantxb)):
        try:
            b = can.Bus(
                interface="socketcan",
                channel=ch_name,
            )
            open_buses.append((ch_name, b, pin))
        except (can.CanError, OSError) as error:
            print(f"Warning: could not open SocketCAN {ch_name}: {error}", file=sys.stderr)

    if not open_buses:
        print("Error: No SocketCAN interfaces could be opened. Check setup_can.sh.", file=sys.stderr)
        udp_socket.close()
        testpoints.close()
        return 1

    buses = tuple(open_buses)

    counters: dict[int, int] = defaultdict(int)
    cockpit_state = CockpitState()

    latest_control = Control(
        steering=0,
        throttle=127,
        brake=1,
        buttons=0,
    )
    blink_state = BLINK_OFF
    blink_phase = 0

    last_udp_time: float | None = None
    stale_fault_active = False
    n1_fault_active = True

    last_peer_heartbeat = {
        can_a_name: {
            CAN_ID_HBN2: 0.0,
            CAN_ID_HBN3: 0.0,
        },
        can_b_name: {
            CAN_ID_HBN2: 0.0,
            CAN_ID_HBN3: 0.0,
        },
    }

    last_tx_error_time: dict[str, float] = defaultdict(float)

    next_heartbeat_time = time.monotonic()
    next_control_time = time.monotonic()
    next_blink_toggle_time = float("inf")

    def send_dual(can_id: int, payload: bytes) -> int:
        tx_time_ns = monotonic_ns()

        for interface_name, bus, pin in buses:
            message = can.Message(
                arbitration_id=can_id,
                data=payload,
                is_extended_id=False,
            )
            try:
                if pin is not None and testpoints.enabled:
                    pin.on()
                bus.send(message, timeout=0.0)
            except (can.CanError, OSError) as error:
                now_err = time.monotonic()
                if now_err - last_tx_error_time[interface_name] >= 1.0:
                    print(
                        f"{interface_name} TX failed for 0x{can_id:03X}: {error}",
                        file=sys.stderr,
                    )
                    last_tx_error_time[interface_name] = now_err
            finally:
                if pin is not None and testpoints.enabled:
                    pin.off()

        return tx_time_ns

    def send_blink(state: int, phase: int) -> None:
        sequence = next_sequence(counters, CAN_ID_BLINK)
        payload = make_blink(
            sequence=sequence,
            blink_state=state,
            phase_bit=phase,
        )
        send_dual(CAN_ID_BLINK, payload)

    def send_fault(reason: int) -> None:
        sequence = next_sequence(counters, CAN_ID_FAULT)
        payload = make_fault(
            sequence=sequence,
            zone_id=CAN_ZONE_N1,
            reason=reason,
        )
        send_dual(CAN_ID_FAULT, payload)

    def send_fault_clear() -> None:
        sequence = next_sequence(counters, CAN_ID_FAULT)
        payload = make_fault(
            sequence=sequence,
            zone_id=CAN_ZONE_N1,
            reason=CAN_FAULT_NONE,
        )
        send_dual(CAN_ID_FAULT, payload)

    def send_ctrl() -> None:
        testpoints.cmdtx_on()
        try:
            sequence = next_sequence(counters, CAN_ID_CTRL)
            payload = make_ctrl(sequence, latest_control)
            send_dual(CAN_ID_CTRL, payload)
        finally:
            testpoints.cmdtx_off()

    print(f"Listening for wheel UDP on {udp_bind}:{udp_port}")
    print(f"Sending Lab 3 CAN traffic on {can_a_name} and {can_b_name}")

    try:
        while True:
            now = time.monotonic()

            deadlines = [next_heartbeat_time, next_control_time]

            if blink_state != BLINK_OFF:
                deadlines.append(next_blink_toggle_time)

            timeout = max(0.0, min(deadlines) - now)

            read_fds = [udp_socket]
            for _, b, _ in buses:
                if hasattr(b, "fileno"):
                    try:
                        read_fds.append(b.fileno())
                    except Exception:
                        pass

            try:
                readable, _, _ = select.select(
                    read_fds,
                    [],
                    [],
                    timeout,
                )
            except (OSError, select.error):
                readable = []

            if udp_socket in readable:
                while True:
                    try:
                        packet, source = udp_socket.recvfrom(2048)
                    except (BlockingIOError, OSError):
                        break

                    if len(packet) != udp_packet_size:
                        print(
                            f"Ignored UDP packet of {len(packet)} bytes "
                            f"from {source}; expected {udp_packet_size}",
                            file=sys.stderr,
                        )
                        continue

                    testpoints.udprx_on()
                    previous_blink_state = blink_state

                    try:
                        (
                            latest_control,
                            blink_state,
                            test_fault_requested,
                        ) = update_from_packet(cockpit_state, packet)
                    except ValueError as error:
                        testpoints.udprx_off()
                        print(
                            f"UDP packet rejected: {error}",
                            file=sys.stderr,
                        )
                        continue

                    last_udp_time = time.monotonic()

                    if args.verbose:
                        print(
                            "UDP accepted "
                            f"from {source}: "
                            f"steering={latest_control.steering} "
                            f"throttle={latest_control.throttle} "
                            f"brake={latest_control.brake} "
                            f"blink={blink_state}",
                        )

                    if stale_fault_active:
                        stale_fault_active = False

                        if not cockpit_state.test_active:
                            n1_fault_active = False
                            send_fault_clear()

                    if test_fault_requested:
                        n1_fault_active = True
                        send_fault(CAN_FAULT_TESTBTN)

                    send_ctrl()
                    testpoints.udprx_off()
                    next_control_time = time.monotonic() + control_period_s

                    if blink_state != previous_blink_state:
                        blink_phase = 1 if blink_state != BLINK_OFF else 0
                        send_blink(blink_state, blink_phase)

                        if blink_state == BLINK_OFF:
                            next_blink_toggle_time = float("inf")
                        else:
                            next_blink_toggle_time = (
                                time.monotonic() + blink_toggle_period_s
                            )

            for interface_name, bus, _ in buses:
                while True:
                    try:
                        message = bus.recv(timeout=0.0)
                    except (can.CanError, OSError):
                        break

                    if message is None:
                        break

                    now = time.monotonic()

                    if message.arbitration_id in (
                        CAN_ID_HBN2,
                        CAN_ID_HBN3,
                    ):
                        last_peer_heartbeat[interface_name][
                            message.arbitration_id
                        ] = now

                    if message.arbitration_id == CAN_ID_FORCE:
                        payload = bytes(message.data)

                        if len(payload) >= 2:
                            print(
                                f"FORCE received on {interface_name}: "
                                f"{payload[1]}",
                            )

                    if args.verbose:
                        payload = bytes(message.data)
                        sequence = (
                            can_sequence(payload)
                            if payload
                            else None
                        )
                        print(
                            f"{interface_name} RX "
                            f"id=0x{message.arbitration_id:03X} "
                            f"seq={sequence} "
                            f"data={payload.hex()}",
                        )

            now = time.monotonic()

            udp_stale = (
                last_udp_time is None
                or now - last_udp_time > udp_timeout_s
            )

            if udp_stale:
                latest_control = Control(
                    steering=0,
                    throttle=127,
                    brake=1,
                    buttons=0,
                )
                blink_state = BLINK_HAZARD

                if not stale_fault_active:
                    stale_fault_active = True
                    n1_fault_active = True
                    send_fault(CAN_FAULT_STALE_UDP)

                    blink_phase = 1
                    send_blink(blink_state, blink_phase)
                    next_blink_toggle_time = (
                        time.monotonic() + blink_toggle_period_s
                    )

            # Periodic CTRL transmission
            if now >= next_control_time:
                send_ctrl()
                while next_control_time <= now:
                    next_control_time += control_period_s

            bus_a_healthy = any(
                now - last_peer_heartbeat[can_a_name][can_id]
                <= heartbeat_timeout_s
                for can_id in (CAN_ID_HBN2, CAN_ID_HBN3)
            )
            bus_b_healthy = any(
                now - last_peer_heartbeat[can_b_name][can_id]
                <= heartbeat_timeout_s
                for can_id in (CAN_ID_HBN2, CAN_ID_HBN3)
            )

            seen_any_peer_heartbeat = any(
                timestamp > 0.0
                for interface_heartbeats in last_peer_heartbeat.values()
                for timestamp in interface_heartbeats.values()
            )

            if seen_any_peer_heartbeat:
                testpoints.set_degraded(
                    can_a_degraded=not bus_a_healthy,
                    can_b_degraded=not bus_b_healthy,
                )
            else:
                testpoints.set_degraded(
                    can_a_degraded=False,
                    can_b_degraded=False,
                )

            if now >= next_heartbeat_time:
                zone_state = (
                    CAN_ZONE_STATE_ERROR
                    if n1_fault_active
                    else CAN_ZONE_STATE_OK
                )

                sequence = next_sequence(counters, CAN_ID_HBN1)
                payload = make_hbn1(
                    sequence=sequence,
                    zone_state=zone_state,
                    bus_a_healthy=bus_a_healthy,
                    bus_b_healthy=bus_b_healthy,
                )
                send_dual(CAN_ID_HBN1, payload)

                while next_heartbeat_time <= now:
                    next_heartbeat_time += heartbeat_period_s

            if (
                blink_state != BLINK_OFF
                and now >= next_blink_toggle_time
            ):
                blink_phase ^= 0x01
                send_blink(blink_state, blink_phase)

                while next_blink_toggle_time <= now:
                    next_blink_toggle_time += blink_toggle_period_s

    except KeyboardInterrupt:
        print("Stopping cockpit gateway")

    except can.CanError as error:
        print(f"CAN error: {error}", file=sys.stderr)
        return 1

    finally:
        udp_socket.close()

        for _, bus, _ in buses:
            try:
                bus.shutdown()
            except Exception:
                pass

        testpoints.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())