"""LiDARのPointCloud2受信・用途別filter・Mock 3D fixtureの回帰試験。"""
import struct
import time
import unittest
from types import SimpleNamespace

import numpy as np

from cockpit.server import BIN_LIDAR, RobotBridge
from m2_navila.elevation_node import parse_pointcloud2


def pointcloud2(points, *, endian="<", datatype=7, row_padding=0):
    """organized 2-row PointCloud2風fixture。datatype 7=f32 / 8=f64。"""
    points = np.asarray(points)
    if points.shape[0] % 2:
        raise ValueError("fixtureの点数は偶数にしてください")
    width, height = points.shape[0] // 2, 2
    scalar = "f" if datatype == 7 else "d"
    size = 4 if datatype == 7 else 8
    point_step = size * 3
    row_step = width * point_step + row_padding
    data = bytearray(row_step * height)
    for i, xyz in enumerate(points):
        row, col = divmod(i, width)
        struct.pack_into(endian + scalar * 3, data, row * row_step + col * point_step,
                         *[float(v) for v in xyz])
    fields = [SimpleNamespace(name=name, offset=i * size, datatype=datatype, count=1)
              for i, name in enumerate(("x", "y", "z"))]
    return SimpleNamespace(
        fields=fields, point_step=point_step, row_step=row_step,
        width=width, height=height, is_bigendian=endian == ">", data=data)


class PointCloudParserTest(unittest.TestCase):
    def test_organized_rows_with_padding(self):
        expected = np.array([[1, 2, 3], [4, 5, 6], [-1, -2, -3], [7, 8, 9]],
                            dtype=np.float32)
        actual = parse_pointcloud2(pointcloud2(expected, row_padding=8))
        np.testing.assert_allclose(actual, expected)

    def test_big_endian_float32(self):
        expected = np.array([[1.25, 2.5, 3.75], [4, 5, 6], [7, 8, 9], [10, 11, 12]],
                            dtype=np.float32)
        actual = parse_pointcloud2(pointcloud2(expected, endian=">"))
        np.testing.assert_allclose(actual, expected)

    def test_float64_and_nonfinite_filter(self):
        points = np.array([[1, 2, 3], [4, 5, 6], [7, np.nan, 9], [10, 11, 12]],
                          dtype=np.float64)
        actual = parse_pointcloud2(pointcloud2(points, datatype=8))
        np.testing.assert_allclose(actual, points[[0, 1, 3]].astype(np.float32))

    def test_missing_xyz_is_rejected(self):
        msg = pointcloud2(np.zeros((4, 3), np.float32))
        msg.fields = msg.fields[:2]
        with self.assertRaisesRegex(ValueError, "field"):
            parse_pointcloud2(msg)


class CloudFilterTest(unittest.TestCase):
    def setUp(self):
        self.bridge = RobotBridge.__new__(RobotBridge)
        self.bridge.pose = (0.0, 0.0, 0.31, 0.0)

    def test_ui_keeps_walls_while_elevation_keeps_ground_band(self):
        floor = [1.0, 0.0, 0.0]
        high_wall = [1.0, 0.0, 2.5]
        bad_depth = [1.0, 0.0, -14.0]
        too_far = [11.0, 0.0, 0.0]
        pts = np.asarray([floor, high_wall, bad_depth, too_far], dtype=np.float32)

        ui = self.bridge._filter_cloud_ui(pts)
        elev = self.bridge._filter_cloud_elevation(pts)

        np.testing.assert_allclose(ui, np.asarray([floor, high_wall], np.float32))
        np.testing.assert_allclose(elev, np.asarray([floor], np.float32))


class CloudFrameGateTest(unittest.TestCase):
    def make_bridge(self):
        bridge = RobotBridge.__new__(RobotBridge)
        bridge.pose = (0.0, 0.0, 0.31, 0.0)
        bridge.pose_src = "lidar_odom"
        bridge.cloud_pts = np.ones((1, 3), np.float32)  # 直前のvalid scan
        bridge.cloud_ts = time.monotonic()
        bridge.cloud_rx_ts = bridge.cloud_ts
        bridge.cloud_scan_valid = True
        bridge.cloud_hz = 0.0
        bridge.cloud_frame = "odom"
        bridge.cloud_raw_n = bridge.cloud_ui_n = bridge.cloud_elev_n = 1
        bridge.cloud_parse_errors = 0
        bridge.cloud_error = None
        bridge.cloud_bounds = None
        bridge._cloud_warn_ts = time.monotonic()
        bridge._cloud_n = 0
        bridge._cloud_t0 = time.monotonic()
        bridge.elev_inserted = []
        bridge.elev = SimpleNamespace(
            recenter=lambda *_: None,
            insert=lambda pts: bridge.elev_inserted.append(pts.copy()))
        return bridge

    def test_sensor_frame_is_not_accumulated_or_inserted(self):
        for frame_id in ("utlidar_lidar", "map", "not_odom", ""):
            with self.subTest(frame_id=frame_id or "missing"):
                bridge = self.make_bridge()
                msg = pointcloud2(np.asarray(
                    [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], np.float32))
                msg.header = SimpleNamespace(frame_id=frame_id)

                bridge._on_cloud(msg)

                self.assertFalse(bridge.cloud_scan_valid)
                self.assertEqual(bridge.cloud_ui_n, 0)
                self.assertEqual(bridge.cloud_elev_n, 0)
                self.assertEqual(bridge.elev_inserted, [])
                self.assertIsNone(bridge.lidar_frame())
                self.assertIn("破棄", bridge.cloud_error)

    def test_empty_scan_stops_retransmitting_previous_hits(self):
        bridge = self.make_bridge()
        msg = pointcloud2(np.empty((0, 3), np.float32))
        msg.header = SimpleNamespace(frame_id="odom")

        bridge._on_cloud(msg)

        self.assertFalse(bridge.cloud_scan_valid)
        self.assertIsNone(bridge.lidar_frame())
        self.assertEqual(bridge.elev_inserted, [])
        self.assertIn("0件", bridge.cloud_error)

    def test_malformed_scan_does_not_escape_callback(self):
        bridge = self.make_bridge()
        msg = pointcloud2(np.ones((4, 3), np.float32))
        msg.header = SimpleNamespace(frame_id="odom")
        msg.data = msg.data[:3]

        bridge._on_cloud(msg)  # 例外をDDS readerへ伝播させない

        self.assertFalse(bridge.cloud_scan_valid)
        self.assertEqual(bridge.cloud_parse_errors, 1)
        self.assertIsNone(bridge.lidar_frame())
        self.assertIn("処理失敗", bridge.cloud_error)

    def test_scan_waits_until_pose_is_available(self):
        bridge = self.make_bridge()
        bridge.pose = None
        bridge.pose_src = "none"
        bridge.bot = SimpleNamespace(state=lambda: {})
        msg = pointcloud2(np.ones((4, 3), np.float32))
        msg.header = SimpleNamespace(frame_id="odom")

        bridge._on_cloud(msg)

        self.assertFalse(bridge.cloud_scan_valid)
        self.assertEqual(bridge.cloud_ui_n, 0)
        self.assertEqual(bridge.elev_inserted, [])
        self.assertIsNone(bridge.lidar_frame())
        self.assertIn("pose未受信", bridge.cloud_error)


class MockSpatialSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene, cls.ground = RobotBridge._make_mock_scene()

    def test_fixture_is_finite_float32_and_volumetric(self):
        self.assertEqual(self.scene.dtype, np.float32)
        self.assertTrue(np.isfinite(self.scene).all())
        self.assertGreater(len(self.scene), 20000)
        self.assertGreater(float(np.ptp(self.scene[:, 2])), 2.5)
        self.assertGreater(float(np.ptp(self.scene[:, 1])), 4.0)
        self.assertLess(float(self.ground[:, 2].max()), 0.7)

    def test_each_stair_has_a_vertical_riser(self):
        for i in range(RobotBridge.MOCK_N_STEP):
            x = RobotBridge.MOCK_STEP_X0 + i * RobotBridge.MOCK_TREAD
            mask = (np.abs(self.scene[:, 0] - x) < 1e-5) & (np.abs(self.scene[:, 1]) < 1.0)
            z = self.scene[mask, 2]
            lo, hi = i * RobotBridge.MOCK_STEP_H, (i + 1) * RobotBridge.MOCK_STEP_H
            interior = z[(z > lo + 0.02) & (z < hi - 0.02)]
            self.assertGreater(len(interior), 50, "step %d に垂直面がない" % (i + 1))

    def test_wire_frame_preserves_3d_points(self):
        bridge = RobotBridge.__new__(RobotBridge)
        bridge.cloud_pts = self.scene
        bridge.cloud_ts = time.monotonic()
        bridge.cloud_scan_valid = True
        frame = bridge.lidar_frame(max_pts=8000)
        kind, count = struct.unpack_from("<BI", frame)
        xyz = np.frombuffer(frame, dtype="<f4", offset=5).reshape(-1, 3)

        self.assertEqual(kind, BIN_LIDAR)
        self.assertEqual(count, 8000)
        self.assertEqual(xyz.shape, (8000, 3))
        self.assertTrue(np.isfinite(xyz).all())
        self.assertGreater(float(np.ptp(xyz[:, 2])), 2.5)


if __name__ == "__main__":
    unittest.main()
