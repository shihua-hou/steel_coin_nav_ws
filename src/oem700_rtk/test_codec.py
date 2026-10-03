#!/usr/bin/env python3
"""Offline checks for NMEA, datum math, and the fusion switch."""

import math
import os
import tempfile

from oem700_rtk.fusion_launch import datum_path, fusion_requested, write_nav2_params
from oem700_rtk.geodesy import base_from_antenna, enu_to_map, geodetic_to_enu
from oem700_rtk.nmea_codec import (
    format_sentence,
    nmea_checksum,
    parse_gga,
    parse_heading,
    parse_rmc,
)


def check(name, cond):
    if not cond:
        raise SystemExit("FAIL " + name)
    print("ok", name)


def main():
    check("gnp checksum", nmea_checksum("gnp,") == "55")
    check("gnp sentence", format_sentence("gnp,") == "$gnp,*55")
    check("sne checksum", nmea_checksum("sne,1,") == "49")
    check("sne sentence", format_sentence("sne,1,") == "$sne,1,*49")
    sample = "snp,vrs.sixents.com,8002,RTCM32_GNSS2,smrns453,pFGx5a6z,1"
    check("sample snp", nmea_checksum(sample) == "11")

    gga = parse_gga(
        "$GNGGA,022939.00,3158.62024845,N,11844.98328928,E,5,21,1.3,70.99,M,2.07,M,,*65")
    check("gga quality", gga["quality"] == 5 and gga["sats"] == 21)
    check("gga lat", abs(gga["lat"] - (31 + 58.62024845 / 60.0)) < 1e-9)
    check("gga lon", abs(gga["lon"] - (118 + 44.98328928 / 60.0)) < 1e-9)

    rmc = parse_rmc(
        "$GNRMC,022939.00,A,3158.62024845,N,11844.98328928,E,0.008,273.0,150926,6.1,W,F,C*55")
    check("rmc float", rmc["mode"] == "F" and rmc["status"] == "A")

    hpr = parse_heading("$GNHPR,022939.00,255.8813,53.4877,000.0000,5,15,0.00,0999*5C")
    check("hpr valid", hpr["valid"] and hpr["status"] == 5)
    # Heading 90° (east) → ENU yaw 0.
    east = parse_heading("$GNHDT,90.0,T*00")
    check("hdt east", abs(east["yaw"]) < 1e-9)

    e, n, u = geodetic_to_enu(31.0, 118.0, 10.0, 31.0, 118.0, 10.0)
    check("enu origin", abs(e) < 1e-6 and abs(n) < 1e-6 and abs(u) < 1e-6)
    mx, my = enu_to_map(3.0, 4.0, 0.0)
    check("map axes", abs(mx - 3.0) < 1e-9 and abs(my - 4.0) < 1e-9)
    bx, by = base_from_antenna(1.0, 0.0, 0.0, 0.25, 0.0)
    check("lever", abs(bx - 0.75) < 1e-9 and abs(by) < 1e-9)

    missing = "/tmp/does_not_exist_map.yaml"
    check("no datum", fusion_requested(missing, "auto") is False)
    check("forced off", fusion_requested(missing, "true") is False)
    with tempfile.TemporaryDirectory() as tmp:
        map_yaml = os.path.join(tmp, "yard.yaml")
        open(map_yaml, "w").write("image: yard.pgm\n")
        open(datum_path(map_yaml), "w").write("lat0: 1\n")
        check("auto on", fusion_requested(map_yaml, "auto") is True)
        check("explicit off", fusion_requested(map_yaml, "false") is False)
        src = os.path.join(tmp, "params.yaml")
        open(src, "w").write("amcl:\n  ros__parameters:\n    tf_broadcast: true\n")
        out = write_nav2_params(src, True, os.path.join(tmp, "out.yaml"))
        text = open(out).read()
        check("tf off", "tf_broadcast: false" in text and "tf_broadcast: true" not in text)
        same = write_nav2_params(src, False, os.path.join(tmp, "unused.yaml"))
        check("tf kept", same == src)

    print("all checks passed")


if __name__ == "__main__":
    main()
