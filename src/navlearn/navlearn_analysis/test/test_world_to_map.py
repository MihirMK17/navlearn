#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the world-geometry map generator.

Why these are strict
    A map generator fails quietly. A wrong slice height, a dropped unit conversion or a
    transposed axis all produce a plausible-looking map-shaped image, and the first
    symptom is a localization result that is wrong for reasons nobody suspects. Every
    test here therefore checks a value that is known by construction rather than
    checking that the code merely ran.

    The unit conversion has its own test against a real asset because it is the failure
    with the largest blast radius: the AWS meshes are authored in centimetres, and
    skipping `unitmeter` yields a world 100x too large that still rasterizes happily.
"""

import math
import pathlib

import numpy as np
import pytest

# world_to_map imports defusedxml at module scope, so without it this file fails at
# COLLECTION -- an error, not a skip -- and takes the whole pytest session's exit code
# with it. Both it and pycollada are declared in package.xml (python3-defusedxml,
# python3-collada), so a rosdep-provisioned environment never skips; this guard is for
# a bare one.
pytest.importorskip("defusedxml")

from navlearn_analysis.world_to_map import (  # noqa: E402
    box_triangles,
    collect_world_triangles,
    cylinder_triangles,
    flood_free,
    load_mesh_triangles,
    parse_pose,
    pose_matrix,
    rasterize,
    read_pgm,
    slice_triangles,
    sphere_triangles,
    to_pgm_array,
    write_pgm,
)


def _aws_mesh():
    """The vendored warehouse bucket mesh, resolved through the ament index."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            pathlib.Path(get_package_share_directory("bumperbot_description"))
            / "models"
            / "aws_robomaker_warehouse_Bucket_01"
            / "meshes"
            / "aws_robomaker_warehouse_Bucket_01_collision.DAE"
        )
    except Exception:  # noqa: BLE001 - bare environment without the workspace sourced
        return pathlib.Path("/nonexistent")


AWS_MESH = _aws_mesh()


class TestPose:
    def test_missing_components_are_zero(self):
        assert parse_pose("1 2 3") == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)

    def test_none_is_identity(self):
        assert parse_pose(None) == (0.0,) * 6

    def test_identity_matrix(self):
        assert np.allclose(pose_matrix((0, 0, 0, 0, 0, 0)), np.eye(4))

    def test_translation(self):
        m = pose_matrix((1.0, 2.0, 3.0, 0, 0, 0))
        assert np.allclose(m[:3, 3], [1, 2, 3])

    def test_yaw_90_maps_x_to_y(self):
        m = pose_matrix((0, 0, 0, 0, 0, math.pi / 2))
        got = m @ np.array([1.0, 0.0, 0.0, 1.0])
        assert np.allclose(got[:3], [0.0, 1.0, 0.0], atol=1e-12)

    def test_roll_90_maps_y_to_z(self):
        m = pose_matrix((0, 0, 0, math.pi / 2, 0, 0))
        got = m @ np.array([0.0, 1.0, 0.0, 1.0])
        assert np.allclose(got[:3], [0.0, 0.0, 1.0], atol=1e-12)


class TestPrimitives:
    def test_box_is_twelve_triangles_of_the_right_extent(self):
        tris = box_triangles(2.0, 4.0, 6.0)
        assert tris.shape == (12, 3, 3)
        pts = tris.reshape(-1, 3)
        assert np.allclose(pts.max(axis=0), [1.0, 2.0, 3.0])
        assert np.allclose(pts.min(axis=0), [-1.0, -2.0, -3.0])

    def test_cylinder_radius_and_length(self):
        tris = cylinder_triangles(1.5, 4.0, segments=64)
        pts = tris.reshape(-1, 3)
        assert pytest.approx(pts[:, 2].max(), abs=1e-9) == 2.0
        assert pytest.approx(pts[:, 2].min(), abs=1e-9) == -2.0
        radii = np.hypot(pts[:, 0], pts[:, 1])
        assert radii.max() <= 1.5 + 1e-9

    def test_sphere_stays_inside_its_radius(self):
        pts = sphere_triangles(2.0, segments=32).reshape(-1, 3)
        assert np.linalg.norm(pts, axis=1).max() <= 2.0 + 1e-9


class TestSlice:
    def test_box_cross_section_is_its_footprint(self):
        # A 2x4 box spanning z in [-3, 3], cut at z=0: the cross-section must be the
        # 2x4 rectangle, so the extreme points sit on its corners.
        segs = slice_triangles(box_triangles(2.0, 4.0, 6.0), 0.0)
        assert len(segs) > 0
        pts = segs.reshape(-1, 2)
        assert np.allclose(pts[:, 0].max(), 1.0, atol=1e-9)
        assert np.allclose(pts[:, 0].min(), -1.0, atol=1e-9)
        assert np.allclose(pts[:, 1].max(), 2.0, atol=1e-9)
        assert np.allclose(pts[:, 1].min(), -2.0, atol=1e-9)

    def test_height_above_the_geometry_yields_nothing(self):
        assert len(slice_triangles(box_triangles(1.0, 1.0, 1.0), 5.0)) == 0

    def test_height_below_the_geometry_yields_nothing(self):
        assert len(slice_triangles(box_triangles(1.0, 1.0, 1.0), -5.0)) == 0

    def test_the_slice_height_actually_selects(self):
        # THE property the whole tool exists for. A box occupying z in [0.1, 0.2] is
        # present at 0.15 and absent at 0.25. A tool that ignored the height argument
        # would pass every other test in this file.
        tris = box_triangles(1.0, 1.0, 0.1)
        shifted = tris + np.array([0.0, 0.0, 0.15])
        assert len(slice_triangles(shifted, 0.15)) > 0
        assert len(slice_triangles(shifted, 0.25)) == 0

    def test_sphere_cross_section_radius_matches_the_analytic_value(self):
        # A sphere of radius 1 cut at z = 0.6 has cross-section radius 0.8.
        segs = slice_triangles(sphere_triangles(1.0, segments=64), 0.6)
        r = np.hypot(*segs.reshape(-1, 2).T)
        assert pytest.approx(r.max(), rel=0.02) == 0.8

    def test_a_translated_box_slices_where_it_was_moved_to(self):
        tris = box_triangles(1.0, 1.0, 1.0) + np.array([10.0, -5.0, 0.0])
        pts = slice_triangles(tris, 0.0).reshape(-1, 2)
        assert pytest.approx(pts[:, 0].mean(), abs=1e-9) == 10.0
        assert pytest.approx(pts[:, 1].mean(), abs=1e-9) == -5.0


class TestRasterize:
    def test_a_horizontal_segment_marks_a_contiguous_row(self):
        segs = np.array([[[0.0, 0.0], [1.0, 0.0]]])
        grid = rasterize(segs, 0.1, (0.0, 0.0), (5, 15))
        assert grid[0, :10].all()
        assert not grid[1:, :].any()

    def test_a_diagonal_segment_has_no_gaps(self):
        # Sub-cell stepping exists so a diagonal cannot skip cells; a gap in a wall
        # lets the flood fill leak and turns unknown space into free space.
        segs = np.array([[[0.0, 0.0], [1.0, 1.0]]])
        grid = rasterize(segs, 0.05, (0.0, 0.0), (25, 25))
        rows = np.nonzero(grid.any(axis=1))[0]
        assert set(range(rows.min(), rows.max() + 1)) <= set(rows.tolist())

    def test_points_outside_the_grid_are_dropped_not_wrapped(self):
        segs = np.array([[[-10.0, -10.0], [-9.0, -9.0]]])
        grid = rasterize(segs, 0.1, (0.0, 0.0), (10, 10))
        assert not grid.any()


class TestFloodFree:
    def _closed_box(self):
        occ = np.zeros((20, 20), dtype=bool)
        occ[5, 5:15] = True
        occ[14, 5:15] = True
        occ[5:15, 5] = True
        occ[5:15, 14] = True
        return occ

    def test_interior_seed_fills_only_the_interior(self):
        occ = self._closed_box()
        free = flood_free(occ, (10, 10))
        assert free[10, 10] and free[6, 6]
        assert not free[0, 0], "flood escaped a closed room"
        assert not free[5, 5]

    def test_exterior_seed_does_not_enter_a_closed_room(self):
        occ = self._closed_box()
        free = flood_free(occ, (0, 0))
        assert free[0, 0]
        assert not free[10, 10]

    def test_seed_on_an_obstacle_is_refused(self):
        occ = np.zeros((5, 5), dtype=bool)
        occ[2, 2] = True
        with pytest.raises(RuntimeError, match="occupied"):
            flood_free(occ, (2, 2))

    def test_seed_outside_the_grid_is_refused(self):
        with pytest.raises(RuntimeError, match="outside"):
            flood_free(np.zeros((5, 5), dtype=bool), (99, 99))


class TestPgm:
    def test_pixel_values_follow_map_server_conventions(self):
        occ = np.zeros((3, 3), dtype=bool)
        free = np.zeros((3, 3), dtype=bool)
        occ[0, 0] = True
        free[2, 2] = True
        img = to_pgm_array(occ, free)
        assert img[2, 0] == 0  # occupied, and row 0 of the array is the image bottom
        assert img[0, 2] == 254  # free
        assert img[1, 1] == 205  # unknown

    def test_write_then_read_roundtrips(self, tmp_path):
        img = np.array([[0, 128], [205, 254]], dtype=np.uint8)
        path = tmp_path / "m.pgm"
        write_pgm(path, img)
        assert np.array_equal(read_pgm(path), img)

    def test_occupied_wins_over_free(self):
        occ = np.ones((2, 2), dtype=bool)
        free = np.ones((2, 2), dtype=bool)
        assert (to_pgm_array(occ, free) == 0).all()


@pytest.mark.skipif(not AWS_MESH.exists(), reason="AWS collision mesh not vendored")
class TestRealMeshUnits:
    @pytest.fixture(autouse=True)
    def _needs_pycollada(self):
        # load_mesh_triangles imports collada lazily; skip rather than error where the
        # rosdep dependency (python3-collada) was never installed.
        pytest.importorskip("collada")

    def test_centimetre_authored_mesh_comes_back_in_metres(self):
        # The bucket is a metre-scale object. Without unitmeter it would measure ~100 m
        # and the resulting map would be a hundred times too large -- still a valid
        # image, still wrong.
        tris = load_mesh_triangles(AWS_MESH)
        extent = tris.reshape(-1, 3).max(axis=0) - tris.reshape(-1, 3).min(axis=0)
        assert extent.max() < 3.0, f"mesh is {extent} m; unitmeter was not applied"
        assert extent.max() > 0.3, f"mesh is {extent} m; suspiciously small"

    def test_it_slices_at_the_robot_lidar_height(self):
        tris = load_mesh_triangles(AWS_MESH)
        assert len(slice_triangles(tris, 0.1538)) > 0


class TestWorldWalk:
    def test_model_and_link_poses_compose(self):
        sdf = """<sdf version='1.7'><world name='w'>
          <model name='m'><pose>10 0 0 0 0 0</pose>
            <link name='l'><pose>0 5 0 0 0 0</pose>
              <collision name='c'><geometry><box><size>1 1 1</size></box></geometry>
              </collision>
            </link>
          </model>
        </world></sdf>"""
        tris, counts, warnings = collect_world_triangles(sdf, "/nonexistent")
        pts = tris.reshape(-1, 3)
        assert pytest.approx(pts[:, 0].mean(), abs=1e-9) == 10.0
        assert pytest.approx(pts[:, 1].mean(), abs=1e-9) == 5.0
        assert counts["collisions"] == 1
        assert warnings == []

    def test_nested_models_are_followed(self):
        sdf = """<sdf version='1.7'><world name='w'>
          <model name='outer'><pose>1 0 0 0 0 0</pose>
            <model name='inner'><pose>2 0 0 0 0 0</pose>
              <link name='l'>
                <collision name='c'><geometry><box><size>1 1 1</size></box></geometry>
                </collision>
              </link>
            </model>
          </model>
        </world></sdf>"""
        tris, _, _ = collect_world_triangles(sdf, "/nonexistent")
        assert pytest.approx(tris.reshape(-1, 3)[:, 0].mean(), abs=1e-9) == 3.0

    def test_a_ground_plane_is_not_an_obstacle(self):
        sdf = """<sdf version='1.7'><world name='w'>
          <model name='ground'><link name='l'>
            <collision name='c'><geometry><plane><normal>0 0 1</normal></plane>
            </geometry></collision></link></model>
          <model name='box'><link name='l'>
            <collision name='c'><geometry><box><size>1 1 1</size></box></geometry>
            </collision></link></model>
        </world></sdf>"""
        tris, counts, _ = collect_world_triangles(sdf, "/nonexistent")
        assert counts["collisions"] == 1, "the plane should contribute no geometry"
        assert len(tris) == 12

    def test_a_world_with_no_collisions_is_refused(self):
        sdf = "<sdf version='1.7'><world name='w'></world></sdf>"
        with pytest.raises(RuntimeError, match="no collision geometry"):
            collect_world_triangles(sdf, "/nonexistent")

    def test_an_unresolvable_mesh_warns_rather_than_vanishing(self):
        sdf = """<sdf version='1.7'><world name='w'>
          <model name='m'><link name='l'>
            <collision name='c'><geometry><mesh><uri>model://nope/x.dae</uri></mesh>
            </geometry></collision></link></model>
          <model name='b'><link name='l'>
            <collision name='c'><geometry><box><size>1 1 1</size></box></geometry>
            </collision></link></model>
        </world></sdf>"""
        _, _, warnings = collect_world_triangles(sdf, "/nonexistent")
        assert any("unresolved mesh" in w for w in warnings)
