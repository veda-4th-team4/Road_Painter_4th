#!/usr/bin/env python3
"""실기 RPi <-> STM32 UART 무동작 통신 시험.

SET_SPEED(0, 0)을 100 ms마다 전송하고 STM32의 STATUS 프레임을 검증합니다.
모터를 회전시키거나 노즐을 켜는 명령은 전송하지 않습니다.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

STX = 0xAA
ETX = 0x55
MAX_PAYLOAD = 16

CMD_SET_SPEED = 0x01
CMD_CLEAR_ESTOP = 0x04
CMD_STATUS = 0x81

STATUS_NAMES = (
    (1 << 0, "MOVING"),
    (1 << 1, "ESTOP"),
    (1 << 2, "TIMEOUT"),
    (1 << 3, "NOZZLE"),
    (1 << 4, "RX_ERROR"),
    (1 << 5, "TX_OVERFLOW"),
)


def crc8(data: bytes) -> int:
    """CRC-8(poly=0x07, init=0x00, no reflection)를 계산합니다."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x07) & 0xFF
                if crc & 0x80
                else (crc << 1) & 0xFF
            )
    return crc


def encode(command: int, payload: bytes) -> bytes:
    """AA|LEN|CMD|PAYLOAD|CRC8|55 프레임을 생성합니다."""
    body = bytes((len(payload), command)) + payload
    return bytes((STX,)) + body + bytes((crc8(body), ETX))


def pop_frame(buffer: bytearray) -> bytes | None:
    """수신 버퍼에서 CRC까지 유효한 프레임 하나를 꺼냅니다."""
    while buffer:
        try:
            stx_index = buffer.index(STX)
        except ValueError:
            buffer.clear()
            return None

        if stx_index:
            del buffer[:stx_index]
        if len(buffer) < 2:
            return None

        payload_length = buffer[1]
        if payload_length > MAX_PAYLOAD:
            del buffer[0]
            continue

        frame_length = payload_length + 5
        if len(buffer) < frame_length:
            return None

        candidate = bytes(buffer[:frame_length])
        if candidate[-1] != ETX or crc8(candidate[1:-2]) != candidate[-2]:
            del buffer[0]
            continue

        del buffer[:frame_length]
        return candidate

    return None


def flag_text(flags: int) -> str:
    """STATUS flag를 사람이 읽을 수 있는 문자열로 변환합니다."""
    names = [name for mask, name in STATUS_NAMES if flags & mask]
    return "|".join(names) if names else "NONE"


def run(port: str, duration: float) -> int:
    """실제 serial port에서 무동작 UART 시험을 실행합니다."""
    try:
        import serial
    except ImportError:
        print(
            "pyserial이 없습니다. Raspberry Pi에서 "
            "'sudo apt install python3-serial'을 실행하세요.",
            file=sys.stderr,
        )
        return 2

    zero_speed = encode(CMD_SET_SPEED, struct.pack("<hh", 0, 0))
    clear_estop = encode(CMD_CLEAR_ESTOP, bytes.fromhex("5A A5"))
    rx_buffer = bytearray()
    valid_status_count = 0
    invalid_status_count = 0

    print(f"포트 열기: {port}, 115200-8-N-1")
    with serial.Serial(port, 115200, timeout=0.02) as uart:
        uart.reset_input_buffer()

        # 이전 시험 종료 후 timeout ESTOP 상태여도 재시험할 수 있게 먼저 정지
        # heartbeat를 보낸 뒤 안전키로 latch 해제를 요청합니다.
        uart.write(zero_speed)
        time.sleep(0.05)
        uart.write(clear_estop)

        start = time.monotonic()
        deadline = start + duration
        next_heartbeat = start

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                uart.write(zero_speed)
                next_heartbeat = now + 0.1

            received = uart.read(uart.in_waiting or 1)
            if received:
                rx_buffer.extend(received)

            while True:
                frame = pop_frame(rx_buffer)
                if frame is None:
                    break
                command = frame[2]
                payload = frame[3:-2]
                if command != CMD_STATUS or len(payload) != 9:
                    invalid_status_count += 1
                    continue

                left_steps, right_steps, flags = struct.unpack("<iiB", payload)
                valid_status_count += 1
                print(
                    f"STATUS #{valid_status_count:02d}: "
                    f"left={left_steps}, right={right_steps}, "
                    f"flags=0x{flags:02X}({flag_text(flags)})"
                )

    print(
        f"결과: valid STATUS={valid_status_count}, "
        f"unexpected frame={invalid_status_count}"
    )
    if valid_status_count == 0:
        print("FAIL: STM32 STATUS를 받지 못했습니다.", file=sys.stderr)
        return 1

    print("PASS: 양방향 UART와 STATUS CRC 검증 성공")
    print("종료 후 300 ms가 지나면 STM32 timeout ESTOP이 걸리는 것이 정상입니다.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RPi <-> STM32 UART 무동작 smoke test"
    )
    parser.add_argument(
        "--port",
        default="/dev/serial0",
        help="serial 장치(기본값: /dev/serial0, Windows 예: COM5)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="시험 시간 [s] (기본값: 3)",
    )
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration은 0보다 커야 합니다.")
    return run(args.port, args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
