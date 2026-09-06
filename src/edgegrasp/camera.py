"""Two declared fixed simulated camera configurations; no object truth input."""

import math
import xml.etree.ElementTree as ET


def configure_sim_camera(urdf: str, *, rate_hz: float = 5., view: str = "upstream", resolution: str = "upstream") -> str:
    if not math.isfinite(rate_hz) or not 5 <= rate_hz <= 50:
        raise ValueError("camera update rate must be in [5, 50] Hz")
    if view not in ("upstream", "opposite_table_edge"):
        raise ValueError("unknown fixed camera view")
    if resolution not in ("upstream", "320x180"):
        raise ValueError("unknown camera resolution")
    if rate_hz == 5 and view == "upstream" and resolution == "upstream":
        return urdf
    root = ET.fromstring(urdf)
    rates = root.findall(".//sensor[@name='camera_head'][@type='rgbd_camera']/update_rate")
    if len(rates) != 1:
        raise ValueError("expected exactly one pinned RGB-D sensor")
    rates[0].text = format(rate_hz, ".17g")
    if resolution == "320x180":
        for tag, value in (("width", "320"), ("height", "180")):
            elements = root.findall(f".//sensor[@name='camera_head']/camera/image/{tag}")
            if len(elements) != 1:
                raise ValueError("expected pinned camera image size")
            elements[0].text = value
    if view == "opposite_table_edge":
        # Camera center (0.5, 0.5, 0.5); pole and base stay beyond table Y=0.4.
        # Move the existing fixed assembly, preserving its collision shapes.
        for name, xyz, rpy in [
            ("base_link_to_torso_link", "0.5 0.49 0", "0 0 0"),
            ("camera_head_joint", "0 0.01 0.50", "0 0.5759586531581288 -2.181661564992912"),
        ]:
            joints = root.findall(f"./joint[@name='{name}'][@type='fixed']/origin")
            if len(joints) != 1:
                raise ValueError(f"missing fixed camera joint: {name}")
            joints[0].set("xyz", xyz)
            joints[0].set("rpy", rpy)
    return ET.tostring(root, encoding="unicode")
