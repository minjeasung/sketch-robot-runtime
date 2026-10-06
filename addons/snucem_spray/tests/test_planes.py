import copy
import numpy as np
import pytest
from test_contracts import module


def entry(offset=0):
    # Two independently measured rectangles with empty space between them.
    return dict(plane_id='zed:1', center=[0, 0, 1], normal=[0, 0, -1],
                inlier_count=120, rms_m=.002, cells=[
                    [[offset-.3, -.2, 1], [offset-.1, -.2, 1], [offset-.1, .2, 1], [offset-.3, .2, 1]],
                    [[offset+.1, -.2, 1], [offset+.3, -.2, 1], [offset+.3, .2, 1], [offset+.1, .2, 1]]])


def test_timestamp_only_keeps_identity_geometry_change_revokes():
    m = module('planes')
    a = m.snapshot([entry()], 'session', 1, 'cal')
    b = m.snapshot([entry()], 'session', 2, 'cal')
    assert a['revision'] == b['revision']
    assert m.snapshot([entry(.01)], 'session', 2, 'cal')['revision'] != a['revision']
    assert m.snapshot([entry()], 'other', 2, 'cal')['revision'] != a['revision']
    assert m.snapshot([entry()], 'session', 2, 'newcal')['revision'] != a['revision']


def test_verified_hold_preserves_frozen_measured_support_but_loss_revokes():
    m = module('planes')
    tracker = m.StableSupport()
    first = tracker.update([entry()], 'workcell:1')
    a = m.snapshot(first, 'session', 1, 'cal')
    changed = entry()
    changed['inlier_count'], changed['rms_m'] = 140, .003
    # Same locked plane, independently remeshed measured cells and extra area.
    cell = changed['cells'].pop(0)
    changed['cells'].extend([[cell[0], cell[1], cell[2]], [cell[0], cell[2], cell[3]]])
    changed['cells'].append([[-.4, -.2, 1], [-.3, -.2, 1], [-.3, .2, 1], [-.4, .2, 1]])
    b = m.snapshot(tracker.update([changed], 'workcell:1'), 'session', 2, 'cal')
    assert b['revision'] == a['revision']
    assert b['planes'][0]['inlier_count'] == 140
    changed['cells'] = changed['cells'][:1]
    c = m.snapshot(tracker.update([changed], 'workcell:1'), 'session', 3, 'cal')
    assert c['revision'] != a['revision']
    assert tracker.update([], 'workcell:1') == []


def test_cached_snapshot_still_validates_quality_and_calibration():
    m = module('planes')
    builder = m.SnapshotBuilder()
    first = builder.build([entry()], 'session', 1, 'cal')
    e = entry()
    e['rms_m'], e['inlier_count'] = .005, 200
    second = builder.build([e], 'session', 2, 'cal')
    assert second['revision'] == first['revision']
    assert second['stamp_ns'] == 2 and second['planes'][0]['inlier_count'] == 200
    assert first['planes'][0]['inlier_count'] == 120
    assert builder.build([e], 'session', 3, 'new')['revision'] != first['revision']
    e['rms_m'] = .1
    with pytest.raises(ValueError):
        builder.build([e], 'session', 4, 'new')


def test_support_rejects_gap_and_crossing_even_when_endpoints_are_inside():
    m = module('planes')
    cells = entry()['cells']
    m.require_supported_polygon([[-.29, -.1, 1], [-.11, -.1, 1], [-.11, .1, 1], [-.29, .1, 1]], cells, [0, 0, -1])
    with pytest.raises(ValueError, match='support'):
        m.require_supported_polygon([[-.2, -.1, 1], [.2, -.1, 1], [.2, .1, 1], [-.2, .1, 1]], cells, [0, 0, -1])


@pytest.mark.parametrize('field,value', [('normal', [0, 0, 0]), ('center', [0, 0, 1000]), ('cells', []), ('rms_m', float('nan'))])
def test_invalid_geometry_rejected(field, value):
    m = module('planes')
    e = entry()
    e[field] = value
    with pytest.raises(ValueError):
        m.snapshot([e], 'session', 1, 'cal')


def test_projection_uses_request_identity_and_rejects_behind_camera():
    m = module('planes')
    snap = m.snapshot([entry()], 'session', 1, 'cal')
    K = [[300, 0, 320], [0, 300, 240], [0, 0, 1]]
    result = m.project_catalogue(snap, K, (640, 480), np.eye(4), '123', [[0, 0], [639, 479]])
    assert result['generation'] == '123'
    assert len(result['planes']) == 1
    assert len(result['planes'][0]['support_cells']) == 2
    transform = np.eye(4)
    transform[2, 3] = -2
    assert not m.project_catalogue(snap, K, (640, 480), transform, '124', [[0, 0], [639, 479]])['planes']


def test_measured_support_does_not_include_inferred_points():
    m = module('planes')
    xy = np.array([[x, y, 1] for x in np.linspace(0, .1, 11) for y in np.linspace(0, .1, 11)])
    cells = m.measured_cells(xy, [0, 0, -1], max_edge=.025)
    assert cells
    assert np.max(np.array(cells)[:, :, 0]) <= .1
    with pytest.raises(ValueError):
        m.require_supported_polygon([[0, 0, 1], [.5, 0, 1], [.5, .1, 1], [0, .1, 1]], cells, [0, 0, -1])
