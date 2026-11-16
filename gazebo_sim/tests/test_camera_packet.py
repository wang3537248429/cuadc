import pytest
from cuadc_sim.camera_packet import pack_frame,unpack_frame


def test_packet_keeps_source_sequence_time_and_exact_jpeg_bytes():
    jpeg=b'\xff\xd8example\xff\xd9'
    assert unpack_frame(pack_frame(58,12.345,jpeg))==(58,12.345,jpeg)


@pytest.mark.parametrize('data',[b'',b'bad data',b'X'*32])
def test_incomplete_or_wrong_packets_are_rejected(data):
    with pytest.raises(ValueError):unpack_frame(data)
