"""NMEA helpers for OEM700 / UM982. No ROS imports."""

import math


def nmea_checksum(body):
    """XOR of the characters between '$' and '*' (body excludes both)."""
    value = 0
    for ch in body:
        value ^= ord(ch)
    return "%02X" % (value & 0xFF)


def format_sentence(body):
    return "$%s*%s" % (body, nmea_checksum(body))


def split_sentence(line):
    """Return (body_without_dollar, checksum_or_empty). Checksum is not verified."""
    text = (line or "").strip()
    if text.startswith("$"):
        text = text[1:]
    if "*" in text:
        body, cs = text.rsplit("*", 1)
        return body, cs.strip()[:2].upper()
    return text, ""


def checksum_ok(line):
    body, cs = split_sentence(line)
    if not cs:
        return True
    return nmea_checksum(body) == cs


def _dm_to_deg(dm_text, hemi):
    if dm_text is None or dm_text == "":
        return None
    dm = float(dm_text)
    sign = -1.0 if hemi in ("S", "W") else 1.0
    degrees = int(dm // 100)
    minutes = dm - degrees * 100.0
    return sign * (degrees + minutes / 60.0)


def parse_gga(line):
    """GGA → dict or None. quality is the raw fix digit (4 = RTK fixed)."""
    body, _cs = split_sentence(line)
    if "GGA" not in body[:6]:
        return None
    parts = body.split(",")
    if len(parts) < 8:
        return None
    try:
        lat = _dm_to_deg(parts[2], parts[3]) if len(parts) > 3 else None
        lon = _dm_to_deg(parts[4], parts[5]) if len(parts) > 5 else None
    except ValueError:
        return None
    try:
        quality = int(parts[6]) if parts[6] != "" else 0
    except ValueError:
        quality = 0
    sats = None
    if len(parts) > 7 and parts[7] != "":
        try:
            sats = int(float(parts[7]))
        except ValueError:
            sats = None
    hdop = None
    if len(parts) > 8 and parts[8] != "":
        try:
            hdop = float(parts[8])
        except ValueError:
            hdop = None
    alt = None
    if len(parts) > 9 and parts[9] != "":
        try:
            alt = float(parts[9])
        except ValueError:
            alt = None
    return {
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "quality": quality,
        "sats": sats,
        "hdop": hdop,
    }


def parse_rmc(line):
    """RMC mode character: N/A/D/F/R. None if the sentence is not RMC."""
    body, _cs = split_sentence(line)
    if "RMC" not in body[:6]:
        return None
    parts = body.split(",")
    if len(parts) < 3:
        return None
    status = parts[2]
    mode = ""
    # NMEA 2.3+: field 12 (0-based) is the mode indicator (R = RTK fixed).
    if len(parts) > 12 and parts[12] != "":
        mode = parts[12].strip()[:1].upper()
    course = None
    if len(parts) > 8 and parts[8] != "":
        try:
            course = float(parts[8])
        except ValueError:
            course = None
    return {"status": status, "mode": mode, "course_deg": course}


def _heading_deg_to_enu_yaw(heading_deg):
    """NMEA heading is clockwise from true north. ENU yaw is CCW from east."""
    return math.atan2(math.sin(math.radians(90.0 - heading_deg)),
                       math.cos(math.radians(90.0 - heading_deg)))


def parse_heading(line):
    """HPR or HDT → {yaw, valid, status} or None.

    GNHPR field 6 (1-based, sentence name is field 1) is the heading-solution
    status. 4 and 5 are usable; 0 is not. HDT has no status and is accepted
    only as a finite angle — the driver still requires an RTK fix before the
    arbiter uses yaw.
    """
    body, _cs = split_sentence(line)
    name = body.split(",", 1)[0]
    parts = body.split(",")
    if name.endswith("HPR"):
        if len(parts) < 3 or parts[2] == "":
            return None
        try:
            heading = float(parts[2])
        except ValueError:
            return None
        status = None
        if len(parts) > 5 and parts[5] != "":
            try:
                status = int(float(parts[5]))
            except ValueError:
                status = None
        valid = status in (4, 5)
        return {
            "yaw": _heading_deg_to_enu_yaw(heading),
            "valid": valid,
            "status": status,
            "heading_deg": heading,
        }
    if name.endswith("HDT"):
        if len(parts) < 2 or parts[1] == "":
            return None
        try:
            heading = float(parts[1])
        except ValueError:
            return None
        # HDT has no solution-quality field. This board emits a heading that
        # spins while the dog is standing still, so it must not steer the map.
        return {
            "yaw": _heading_deg_to_enu_yaw(heading),
            "valid": False,
            "status": None,
            "heading_deg": heading,
        }
    return None


def yaw_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
