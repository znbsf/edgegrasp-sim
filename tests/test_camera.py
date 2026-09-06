import xml.etree.ElementTree as ET

import pytest

from edgegrasp.camera import configure_sim_camera


URDF = """<robot name="test">
<link name="camera"><collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
<joint name="base_link_to_torso_link" type="fixed"><origin xyz="0 0 0"/></joint>
<joint name="camera_head_joint" type="fixed"><origin xyz="0 0 1"/></joint>
<joint name="arm" type="revolute"><origin xyz="1 2 3"/></joint>
<gazebo><sensor name="camera_head" type="rgbd_camera"><camera><image><width>424</width><height>240</height></image></camera><update_rate>5</update_rate></sensor></gazebo>
</robot>"""


def test_camera_overlay_preserves_arm_and_collision_shapes():
    assert configure_sim_camera(URDF) == URDF
    before = ET.fromstring(URDF)
    after = ET.fromstring(configure_sim_camera(URDF, rate_hz=50, view="opposite_table_edge", resolution="320x180"))
    for path in ("./link", "./joint[@name='arm']"):
        assert ET.tostring(after.find(path)) == ET.tostring(before.find(path))
    assert after.find(".//update_rate").text == "50"
    assert after.find(".//image/width").text == "320"
    assert after.find(".//image/height").text == "180"
    assert after.find("./joint[@name='base_link_to_torso_link']/origin").get("xyz") == "0.5 0.49 0"
    assert after.find("./joint[@name='camera_head_joint']/origin").get("xyz") == "0 0.01 0.50"


def test_camera_overlay_rejects_unknown_or_incomplete_configuration():
    with pytest.raises(ValueError):
        configure_sim_camera(URDF, view="arbitrary")
    with pytest.raises(ValueError):
        configure_sim_camera(URDF, rate_hz=51)
    with pytest.raises(ValueError):
        configure_sim_camera(URDF.replace('name="camera_head_joint"', 'name="missing"'), view="opposite_table_edge")
