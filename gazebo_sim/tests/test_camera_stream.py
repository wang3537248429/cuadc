import threading

from cuadc_sim.camera_stream import LatestFrameMailbox


def test_slow_inference_gets_latest_frame_without_blocking_capture():
    mailbox = LatestFrameMailbox()
    mailbox.put(0)
    assert mailbox.take() == 0
    # Simulate capture continuing while the inference consumer is busy.
    for frame in range(1, 301):
        mailbox.put(frame)
    assert mailbox.take() == 300
    assert mailbox.skipped == 299
    assert mailbox.take(timeout=.001) is None


def test_close_wakes_a_waiting_consumer_and_rejects_new_frames():
    mailbox = LatestFrameMailbox()
    result = []
    reader = threading.Thread(target=lambda: result.append(mailbox.take(timeout=30)))
    reader.start()
    mailbox.close()
    reader.join(timeout=1)
    assert not reader.is_alive() and result == [None]
    mailbox.put(1)
    assert mailbox.latest is None
