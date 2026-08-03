from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from prepare_unitree_scene import TRACK_BLUE_RGBA, prepare_scene


class PrepareUnitreeSceneTest(unittest.TestCase):
    def test_standalone_scene_uses_the_same_track_geometry(self):
        root = ET.parse(ROOT / "assets" / "standalone_lane_scene.xml").getroot()
        self.assertEqual(
            root.find(".//material[@name='track']").get("rgba"),
            TRACK_BLUE_RGBA,
        )
        left = root.find(".//geom[@name='left_lane_line']")
        right = root.find(".//geom[@name='right_lane_line']")
        self.assertEqual(left.get("pos"), "50 1.0 0.002")
        self.assertEqual(right.get("pos"), "50 -1.0 0.002")
        self.assertEqual(left.get("size"), "52.5 0.05 0.002")
        self.assertEqual(right.get("size"), "52.5 0.05 0.002")

    def test_official_four_lane_track_with_locked_second_lane(self):
        source_xml = """\
<mujoco model="test">
  <include file="g1_29dof.xml"/>
  <asset>
    <material name="original_floor" rgba="0.2 0.2 0.2 1"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" material="original_floor"/>
  </worldbody>
</mujoco>
"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "scene.xml"
            output = Path(directory) / "scene_race.xml"
            source.write_text(source_xml, encoding="utf-8")

            prepare_scene(
                source,
                output,
                "g1_29dof_race.xml",
                lane_width=2.1,
                line_width=0.10,
                track_length=100.0,
                lane_count=4,
                target_lane_index=1,
            )

            root = ET.parse(output).getroot()
            self.assertEqual(
                root.find(".//material[@name='race_track']").get("rgba"),
                TRACK_BLUE_RGBA,
            )
            self.assertEqual(
                root.find(".//geom[@name='floor']").get("material"),
                "race_track",
            )

            boundaries = [
                root.find(f".//geom[@name='race_lane_boundary_{index}']")
                for index in range(5)
            ]
            finish = root.find(".//geom[@name='race_finish_line']")
            self.assertTrue(all(boundary is not None for boundary in boundaries))
            self.assertEqual(
                [boundary.get("pos") for boundary in boundaries],
                [
                    "50 -3.15 0.002",
                    "50 -1.05 0.002",
                    "50 1.05 0.002",
                    "50 3.15 0.002",
                    "50 5.25 0.002",
                ],
            )
            self.assertTrue(
                all(
                    boundary.get("size") == "52.5 0.05 0.002"
                    for boundary in boundaries
                )
            )
            self.assertEqual(finish.get("pos"), "100 1.05 0.002")
            self.assertEqual(finish.get("size"), "0.05 4.2 0.002")

    def test_rejects_non_positive_dimensions(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "scene.xml"
            output = Path(directory) / "out.xml"
            source.write_text(
                "<mujoco><include/><worldbody><geom name='floor'/></worldbody></mujoco>",
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                prepare_scene(source, output, "g1.xml", 0.0, 0.10, 100.0)

    def test_100m_finish_can_have_post_finish_runoff(self):
        source_xml = """\
<mujoco><include file="g1.xml"/><worldbody><geom name="floor"/></worldbody></mujoco>
"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "scene.xml"
            output = Path(directory) / "scene_race.xml"
            source.write_text(source_xml, encoding="utf-8")

            prepare_scene(
                source,
                output,
                "g1_race.xml",
                lane_width=2.0,
                line_width=0.10,
                track_length=115.0,
                finish_distance=100.0,
            )

            root = ET.parse(output).getroot()
            boundary = root.find(".//geom[@name='race_lane_boundary_0']")
            finish = root.find(".//geom[@name='race_finish_line']")
            self.assertEqual(boundary.get("pos"), "57.5 -3 0.002")
            self.assertEqual(boundary.get("size"), "60 0.05 0.002")
            self.assertEqual(finish.get("pos"), "100 1 0.002")


if __name__ == "__main__":
    unittest.main()
