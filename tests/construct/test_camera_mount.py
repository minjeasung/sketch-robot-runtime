"""Camera mount geometry, independent of camera transport implementation."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from construct_scene_objects import ZED_MOUNT_ROTATION


def test_camera_mount_is_a_proper_right_handed_rotation():
    rotation = np.asarray(ZED_MOUNT_ROTATION)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3))
    assert np.linalg.det(rotation) == pytest.approx(1.)
    np.testing.assert_allclose(np.cross(rotation[:, 0], rotation[:, 1]), rotation[:, 2])
