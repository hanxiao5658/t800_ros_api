"""Parse the diagnostic std_msgs/String from body_vel_twist_to_vendor.py.

The original data string is always kept verbatim in the REST response.
Parsing is best-effort and never drives robot safety decisions.
"""

import math
import re

_VECTOR = re.compile(
    r"\bvx=([^\s]+)\s+vy=([^\s]+)\s+wz=([^\s]+)"
)
_BOOLEAN_KEYS = {'motion_allowed', 'gamepad_connected', 'sticks_neutral'}


def _finite_float(raw):
    try:
        value = float(raw)
    except (ValueError, TypeError):
        return None
    return value if math.isfinite(value) else None


def _vector(raw):
    match = _VECTOR.search(raw)
    if match is None:
        return None
    values = [_finite_float(v) for v in match.groups()]
    if None in values:
        return None
    return dict(zip(('vx', 'vy', 'wz'), values))


def parse_controller_status(raw):
    """Decode known fields from the current pipe-separated T800 status format."""
    result = {}
    for part in raw.split('|'):
        part = part.strip()
        if not part:
            continue
        if part.startswith('RX Twist:'):
            result['rx_twist'] = _vector(part)  # None if no Twist has arrived
        elif part.startswith('TX '):
            prefix, sep, _ = part.partition(':')
            if sep:
                result['tx_topic'] = prefix.removeprefix('TX ').strip()
                result['tx_twist'] = _vector(part)
        elif '=' in part:
            key, value = part.split('=', 1)
            key, value = key.strip(), value.strip()
            if key in _BOOLEAN_KEYS and value in ('0', '1'):
                result[key] = value == '1'
            elif key == 'twist_age':
                result['twist_age_s'] = (
                    _finite_float(value[:-1]) if value.endswith('s') else None
                )
            else:
                result[key] = value
    return result
