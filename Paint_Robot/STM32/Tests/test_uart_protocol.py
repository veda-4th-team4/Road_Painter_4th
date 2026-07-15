"""RPi/STM32 UART wire contract golden tests."""

STX = 0xAA
ETX = 0x55


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(command: int, payload: bytes) -> bytes:
    body = bytes((len(payload), command)) + payload
    return bytes((STX,)) + body + bytes((crc8(body), ETX))


def decode(frame: bytes) -> tuple[int, bytes]:
    assert frame[0] == STX
    assert frame[-1] == ETX
    payload_length = frame[1]
    assert len(frame) == payload_length + 5
    assert crc8(frame[1:-2]) == frame[-2]
    return frame[2], frame[3:-2]


def test_golden_vectors() -> None:
    vectors = (
        (0x01, bytes.fromhex("F4 01 F4 01"), "AA 04 01 F4 01 F4 01 B1 55"),
        (0x01, bytes.fromhex("00 00 00 00"), "AA 04 01 00 00 00 00 C6 55"),
        (0x02, bytes.fromhex("01"), "AA 01 02 01 46 55"),
        (0x02, bytes.fromhex("00"), "AA 01 02 00 41 55"),
        (0x03, bytes.fromhex("01"), "AA 01 03 01 53 55"),
        (0x04, bytes.fromhex("5A A5"), "AA 02 04 5A A5 7B 55"),
        (
            0x81,
            bytes.fromhex("00 00 00 00 00 00 00 00 00"),
            "AA 09 81 00 00 00 00 00 00 00 00 00 03 55",
        ),
    )

    for command, payload, expected_hex in vectors:
        frame = encode(command, payload)
        assert frame == bytes.fromhex(expected_hex)
        assert decode(frame) == (command, payload)


def test_corrupted_crc_is_rejected() -> None:
    frame = bytearray(bytes.fromhex("AA 04 01 F4 01 F4 01 B1 55"))
    frame[-2] ^= 0x01
    try:
        decode(bytes(frame))
    except AssertionError:
        return
    raise AssertionError("corrupted CRC frame was accepted")


if __name__ == "__main__":
    test_golden_vectors()
    test_corrupted_crc_is_rejected()
    print("UART protocol golden vectors: PASS")
