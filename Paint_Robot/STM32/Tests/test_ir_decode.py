"""IR NEC key decode golden test (F103 예제와 동일 알고리즘)."""

IR_THRESHOLD = 1700


def decode_packet(raw_buf: list[int], length: int) -> int:
    if length < 32:
        return 0xFF
    key_byte = 0
    for i in range(8):
        if raw_buf[16 + i] > IR_THRESHOLD:
            key_byte |= 1 << (7 - i)
    return key_byte


def test_key_byte_msb_first() -> None:
    # bits 16..23: 0xC0 = 1100_0000 → UP
    raw = [1000] * 32
    raw[16] = 2000  # 1
    raw[17] = 2000  # 1
    raw[18] = 1000
    raw[19] = 1000
    raw[20] = 1000
    raw[21] = 1000
    raw[22] = 1000
    raw[23] = 1000
    assert decode_packet(raw, 32) == 0xC0


def test_incomplete_rejected() -> None:
    assert decode_packet([2000] * 16, 16) == 0xFF


def test_set_control_mode_frame() -> None:
    from test_uart_protocol import encode, decode

    frame = encode(0x05, bytes([1]))
    cmd, payload = decode(frame)
    assert cmd == 0x05
    assert payload == b"\x01"


if __name__ == "__main__":
    test_key_byte_msb_first()
    test_incomplete_rejected()
    test_set_control_mode_frame()
    print("IR decode + SET_CONTROL_MODE: PASS")
