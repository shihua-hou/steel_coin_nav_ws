"""Decide when RTK may own map→odom, and rewrite the Nav2 params copy.

Fusion stays off until maps/<stem>.datum.yaml exists. The stock params file
keeps tf_broadcast: true so an uncalibrated robot still navigates with AMCL.
"""

import os

WS_ROOT = os.environ.get("STEEL_COIN_WS", "/home/linaro/steel_coin_nav_ws")
DEFAULT_NAV2_PARAMS = os.path.join(
    WS_ROOT, "src", "isaac_go2_nav2", "config", "steel_coin_nav2_params.yaml")
RTK_PARAMS_OUT = "/tmp/steel_coin_nav2_params_rtk.yaml"


def datum_path(map_yaml):
    stem, _ext = os.path.splitext(os.path.abspath(map_yaml))
    return stem + ".datum.yaml"


def fusion_requested(map_yaml, mode="auto"):
    """mode: auto (datum file enables fusion), true, false."""
    text = (mode or "auto").strip().lower()
    if text in ("0", "false", "no", "off"):
        return False
    has_datum = bool(map_yaml) and os.path.isfile(datum_path(map_yaml))
    if text in ("1", "true", "yes", "on"):
        return has_datum
    return has_datum


def write_nav2_params(src_yaml, fuse, dest=RTK_PARAMS_OUT):
    """Return the params path Nav2 should load. AMCL TF is off only when fuse."""
    if not fuse:
        return src_yaml
    with open(src_yaml, "r", encoding="utf-8") as handle:
        text = handle.read()
    needle = "tf_broadcast: true"
    if needle not in text:
        raise RuntimeError("AMCL tf_broadcast: true not found in %s" % src_yaml)
    text = text.replace(needle, "tf_broadcast: false", 1)
    with open(dest, "w", encoding="utf-8") as handle:
        handle.write(text)
    return dest
