"""Spray illustrations share one render with RGB/depth and the viewport."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import rb20_painting_sensors as sensors


class Message:
    def __init__(self, **kwargs):
        self.header = SimpleNamespace()
        self.__dict__.update(kwargs)


@pytest.fixture
def scene(monkeypatch):
    """Only the Isaac renderer and ROS transport are substituted."""
    overlay = SimpleNamespace(hidden=False)
    overlay.set_sensor_capture = lambda hidden: setattr(overlay, 'hidden', hidden)
    created, published = [], {}
    viewport = SimpleNamespace(updates_enabled=True)
    monkeypatch.setitem(sys.modules, 'omni.kit.viewport.utility',
                        SimpleNamespace(get_active_viewport=lambda: viewport))

    class Camera:
        def __init__(self, *args, **kwargs):
            self.enabled = True
            self.pending = deque()
            self.index = 0
            self.data = dict(rendering_frame=0, rendering_time=0., rgb=None,
                             distance_to_image_plane=None)
            texture = SimpleNamespace(set_updates_enabled=lambda value: setattr(self, 'enabled', value))
            self._render_product = SimpleNamespace(hydra_texture=texture)
            created.append(self)
        def set_focal_length(self, *args): pass
        def set_horizontal_aperture(self, *args): pass
        def set_clipping_range(self, *args): pass
        def initialize(self): pass
        def add_distance_to_image_plane_to_frame(self): pass
        def get_current_frame(self): return self.data
        def get_intrinsics_matrix(self): return np.eye(3)
        def get_rgba(self):
            # The live annotator can advance independently of the frame callback.
            return np.full((2, 3, 4), 201, dtype=np.uint8)

    monkeypatch.setitem(sys.modules, 'isaacsim.sensors.camera', SimpleNamespace(Camera=Camera))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', SimpleNamespace(Image=Message, CameraInfo=Message))
    monkeypatch.setitem(sys.modules, 'builtin_interfaces.msg', SimpleNamespace(Time=Message))
    def publisher(kind, topic, qos):
        published[topic] = []
        return SimpleNamespace(publish=published[topic].append)
    cameras = sensors.RenderedCameras(SimpleNamespace(create_publisher=publisher), {},
        dict(camera_names=['zed', 'd405'], camera_resolution=[3, 2], zed_world_optical=np.eye(4)))
    world = SimpleNamespace(viewport_overlay=None, fail=False, renders=0)
    def render():
        world.renders += 1
        if world.fail:
            raise RuntimeError('render failure')
        if viewport.updates_enabled:
            world.viewport_overlay = not overlay.hidden
        for camera in created:
            # Readback lags submission by a render; disabling products must also
            # keep the viewport pass out of this queue, not merely pause callbacks.
            if camera.pending:
                camera.data = camera.pending.popleft()
            if camera.enabled:
                camera.index += 1
                camera.pending.append(dict(rendering_frame=camera.index,
                    rendering_time=camera.index / 15.,
                    rgb=np.full((2, 3, 4), 19 if overlay.hidden else 201, dtype=np.uint8),
                    distance_to_image_plane=np.full((2, 3), 2. if overlay.hidden else .2)))
    world.render = render
    return cameras, world, overlay, created, published


def test_camera_and_viewport_share_visible_spray_without_extra_renders(scene):
    cameras, world, overlay, created, published = scene
    for _ in range(4):
        sensors.render_sensor_and_viewport(world, cameras, overlay, playing=True)
    assert world.renders == 4
    for name in ('zed', 'd405'):
        rgb = published[sensors.CAMERA_TOPICS[name][0]]
        depth = published[sensors.CAMERA_TOPICS[name][1]]
        assert len(rgb) == len(depth) == 3
        assert all(np.all(np.frombuffer(msg.data, np.uint8) == 201) for msg in rgb)
        assert all(np.all(np.frombuffer(msg.data, np.float32) == .2) for msg in depth)
        assert rgb[0].header.stamp.nanosec == 66666666
    assert world.viewport_overlay is True
    assert all(camera.enabled for camera in created)


@pytest.mark.parametrize('covered', [False, True])
def test_inactive_spray_keeps_one_render_with_or_without_coverage(scene, covered):
    cameras, world, overlay, created, _ = scene
    overlay._enabled = False
    overlay.covered = np.array([covered])
    for _ in range(3):
        sensors.render_sensor_and_viewport(world, cameras, overlay, playing=True)
    assert world.renders == 3
    assert all(camera.enabled for camera in created)


def test_paused_render_does_not_publish_sensor_frames(scene):
    cameras, world, overlay, _, published = scene
    for _ in range(3):
        sensors.render_sensor_and_viewport(world, cameras, overlay, playing=False)
    assert world.renders == 3
    assert not published[sensors.CAMERA_TOPICS['zed'][0]]


def test_failed_render_does_not_publish_or_change_viewport_state(scene):
    cameras, world, overlay, _, published = scene
    cameras.viewport.updates_enabled = False
    world.fail = True
    with pytest.raises(RuntimeError, match='render failure'):
        sensors.render_sensor_and_viewport(world, cameras, overlay, playing=True)
    assert not published[sensors.CAMERA_TOPICS['zed'][0]]
    assert cameras.viewport.updates_enabled is False
    assert overlay.hidden is False


def test_frame_rgb_is_paired_with_its_depth_instead_of_live_annotator(scene):
    cameras, _, _, created, published = scene
    for camera in created:
        camera.data = dict(rendering_frame=7, rendering_time=2.25,
            rgb=np.full((2, 3, 4), 19, dtype=np.uint8),
            distance_to_image_plane=np.full((2, 3), 2.))
    cameras.publish(10.)
    rgb = published[sensors.CAMERA_TOPICS['zed'][0]][0]
    assert np.all(np.frombuffer(rgb.data, np.uint8) == 19)
    assert rgb.header.stamp.sec == 2 and rgb.header.stamp.nanosec == 250000000


def test_missing_frame_rgb_does_not_fall_back_to_unpaired_annotator(scene):
    cameras, _, _, created, published = scene
    for camera in created:
        camera.data = dict(rendering_frame=7, rendering_time=2.25, rgb=None,
                           distance_to_image_plane=np.full((2, 3), 2.))
    cameras.publish(10.)
    assert not published[sensors.CAMERA_TOPICS['zed'][0]]


