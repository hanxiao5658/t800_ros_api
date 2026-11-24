"""Thread-safe last-message cache (independent of ROS packages)."""

import copy
import threading
import time
from datetime import datetime, timezone


class LatestMessageCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._values = {}

    def update(self, key, topic, msg_type, data, parsed=None):
        timestamp = datetime.now(timezone.utc).isoformat(
            timespec='milliseconds'
        ).replace('+00:00', 'Z')
        payload = {
            'topic': topic,
            'msg_type': msg_type,
            'received_at': timestamp,
            'data': data,
        }
        if parsed is not None:
            payload['parsed'] = parsed
        with self._lock:
            self._values[key] = (time.monotonic(), copy.deepcopy(payload))

    def latest(self, key, stale_after_s):
        with self._lock:
            item = self._values.get(key)
            if item is None:
                return None
            received_monotonic, data = item
            snapshot = copy.deepcopy(data)
        age_s = max(0.0, time.monotonic() - received_monotonic)
        snapshot['age_ms'] = int(round(age_s * 1000))
        snapshot['stale'] = age_s > stale_after_s
        return snapshot

    def seen(self):
        with self._lock:
            return sorted(self._values)
