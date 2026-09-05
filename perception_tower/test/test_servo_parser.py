from perception_tower.servo_client import ProtocolParser


def test_parse_position():
    p = ProtocolParser()
    assert p.feed(b"#000P5000!\r\n") == [("pos", 5000)]


def test_parse_ok():
    p = ProtocolParser()
    assert p.feed(b"#OK!\r\n") == [("ok",)]


def test_dirty_debug_strings_filtered():
    p = ProtocolParser()
    data = b"BOOT: ready\r\n#000P5000!\r\nMOV: 2000 -> 5000\r\n#OK!\r\n"
    assert p.feed(data) == [("pos", 5000), ("ok",)]


def test_partial_and_resumable():
    p = ProtocolParser()
    assert p.feed(b"#000P50") == []
    assert p.feed(b"00!\r\n#OK!") == [("pos", 5000), ("ok",)]
    assert p.feed(b"\r\n") == []


def test_custom_servo_id():
    p = ProtocolParser(servo_id=1)
    assert p.feed(b"#001P1234!") == [("pos", 1234)]
    assert p.feed(b"#000P1234!") == []
