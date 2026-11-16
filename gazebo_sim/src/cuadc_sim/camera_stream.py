"""Bounded handoff between camera capture and slower inference."""
from __future__ import annotations

import threading


class LatestFrameMailbox:
    def __init__(self):
        self.condition = threading.Condition()
        self.sequence = 0
        self.delivered = 0
        self.skipped = 0
        self.latest = None
        self.closed = False

    def put(self, value):
        with self.condition:
            if self.closed:
                return
            self.latest = value
            self.sequence += 1
            self.condition.notify()

    def take(self, timeout=.5):
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence > self.delivered or self.closed,
                timeout=timeout,
            )
            if self.closed or self.sequence == self.delivered:
                return None
            self.skipped += max(0, self.sequence - self.delivered - 1)
            self.delivered = self.sequence
            return self.latest

    def close(self):
        with self.condition:
            self.closed = True
            self.latest = None
            self.condition.notify_all()
