"""Immutable source-CAD spray frustum, extrapolated along nozzle TCP +Z."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re


@dataclass(frozen=True)
class SprayEnvelope:
    axial_start_m: float
    start_radius_m: float
    length_m: float
    end_radius_m: float
    source_sha256: str
    profile_id: str
    source_file: str
    basis: str
    sha256: str

    def diameter_at(self, distance):
        """Return diameter at nozzle standoff, retaining the original cone slope."""
        if type(distance) not in (int, float) or not math.isfinite(distance) or distance < 0:
            raise ValueError('spray envelope distance must be finite and nonnegative')
        radius = self.start_radius_m + (distance - self.axial_start_m) * (
            self.end_radius_m - self.start_radius_m) / self.length_m
        if radius <= 0 or not math.isfinite(radius):
            raise ValueError('spray envelope radius must be finite and positive')
        return 2 * radius

    def metadata(self):
        return {
            'spray_envelope_profile_sha256': self.sha256,
            'spray_envelope_source_sha256': self.source_sha256,
            'spray_envelope_profile_id': self.profile_id,
        }


def load_spray_envelope(path):
    """Load and validate local profile geometry without loading its source CAD."""
    try:
        data = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
    except (OSError, TypeError, UnicodeError) as exc:
        raise ValueError(f'spray envelope unavailable: {exc}') from exc
    fields = {'axial_start_m', 'start_radius_m', 'length_m', 'end_radius_m',
              'source_sha256', 'profile_id', 'source_file', 'basis'}
    if not isinstance(data, dict) or set(data) != fields:
        raise ValueError('spray envelope requires its geometry and source provenance fields')
    for key in ('axial_start_m', 'start_radius_m', 'length_m', 'end_radius_m'):
        if type(data[key]) not in (int, float) or not math.isfinite(data[key]):
            raise ValueError(f'{key} must be a finite number')
    if (data['length_m'] <= 0 or data['start_radius_m'] < 0
            or data['end_radius_m'] <= data['start_radius_m']):
        raise ValueError('spray envelope must have positive length and expanding radii')
    if data['basis'] != 'nozzle_tcp_+z':
        raise ValueError('spray envelope basis must be nozzle_tcp_+z')
    for key in ('profile_id', 'source_file'):
        if not isinstance(data[key], str) or not data[key].strip():
            raise ValueError(f'{key} must be nonempty text')
    if not isinstance(data['source_sha256'], str) or not re.fullmatch('[0-9a-f]{64}', data['source_sha256']):
        raise ValueError('spray envelope source_sha256 must be a lowercase SHA-256')
    digest = hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'),
                                      allow_nan=False).encode()).hexdigest()
    envelope = SprayEnvelope(**data, sha256=digest)
    envelope.diameter_at(0.)
    return envelope
