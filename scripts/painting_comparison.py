"""Pure geometry and measured analysis for the three-stroke experiment."""
import numpy as np


def strokes(direction, roller_length, overlap):
    if direction not in ('horizontal', 'vertical'):
        raise ValueError('unknown direction')
    if not np.isfinite([roller_length, overlap]).all() or roller_length <= 0 or not 0 <= overlap < 1:
        raise ValueError('invalid roller geometry')
    spacing = roller_length * (1 - overlap)
    side = roller_length + 2 * spacing
    result = []
    for i, offset in enumerate((-spacing, 0, spacing)):
        ends = [-side / 2, side / 2] if i % 2 == 0 else [side / 2, -side / 2]
        result.append([[end, offset] for end in ends])
    result = np.array(result)
    return result if direction == 'horizontal' else result[:, :, ::-1]


def contact_class(phase, force, valid):
    if not valid or force is None or not np.isfinite(force):
        return 'unknown'
    if phase == 'PAINT' and 18 <= force <= 32:
        return 'contact'
    if phase != 'PAINT' and abs(force) <= 2:
        return 'noncontact'
    return 'transition'


def motion_metrics(times, positions, phases):
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if positions.ndim != 2 or len(times) != len(positions) or len(phases) != len(times):
        raise ValueError('sample dimensions differ')
    if not np.isfinite(times).all() or not np.isfinite(positions).all() or np.any(np.diff(times) <= 0):
        raise ValueError('invalid/nonmonotonic measured samples')
    result = {}
    for phase in sorted(set(phases)):
        pairs = np.array([a == b == phase for a, b in zip(phases, phases[1:])])
        dt = np.diff(times)
        # Missing samples must not create inferred long-distance motion.
        pairs &= dt <= .20
        delta = np.diff(positions, axis=0)[pairs]
        result[phase] = dict(rotation_rad=np.abs(delta).sum(axis=0).tolist(),
                             duration_s=float(dt[pairs].sum()))
    return result


def stroke_displacement(points, direction, requested_length):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<2 or not np.isfinite(points).all():
        return dict(passed=False,reason='missing measured TCP points')
    axis=0 if direction=='horizontal' else 2
    other=2 if axis==0 else 0
    distance=float(abs(points[-1,axis]-points[0,axis]))
    cross=float(np.ptp(points[:,other]))
    return dict(passed=bool(.85*requested_length<=distance<=1.15*requested_length and cross<=.03),
                net_direction_distance_m=distance,cross_direction_span_m=cross)
