#!/usr/bin/env python3
"""Split the mechanically rescaled roller facets from the EOAT collision STL.

The ``*_roller52.stl`` asset was produced from the original collision mesh by
changing only the roller cylinder.  Both binary STL files preserve facet order,
so facets whose vertices changed are the roller and unchanged facets form the
support/bracket collision mesh.  The generated support mesh lets MoveIt grant
contact only to a dedicated roller link instead of the complete EOAT assembly.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct


FACET_SIZE = 50
HEADER_SIZE = 84


def _read_binary_stl(path: Path) -> tuple[bytes, list[bytes]]:
    payload = path.read_bytes()
    if len(payload) < HEADER_SIZE:
        raise ValueError(f"{path}: too short for binary STL")
    count = struct.unpack_from("<I", payload, 80)[0]
    expected = HEADER_SIZE + count * FACET_SIZE
    if len(payload) != expected:
        raise ValueError(
            f"{path}: expected {expected} bytes for {count} facets, "
            f"found {len(payload)}"
        )
    return payload[:80], [
        payload[HEADER_SIZE + i * FACET_SIZE:HEADER_SIZE + (i + 1) * FACET_SIZE]
        for i in range(count)
    ]


def _vertices(facet: bytes) -> tuple[float, ...]:
    # 12 floats are normal xyz followed by three xyz vertices.
    return struct.unpack_from("<12f", facet, 0)[3:]


def split_support_facets(
    original_path: Path, roller52_path: Path, tolerance_m: float = 1e-7
) -> tuple[list[bytes], int]:
    _header_a, original = _read_binary_stl(original_path)
    _header_b, roller52 = _read_binary_stl(roller52_path)
    if len(original) != len(roller52):
        raise ValueError("source meshes do not preserve facet count/order")

    support: list[bytes] = []
    roller_count = 0
    for before, after in zip(original, roller52):
        changed = any(
            abs(a - b) > tolerance_m
            for a, b in zip(_vertices(before), _vertices(after))
        )
        if changed:
            roller_count += 1
        else:
            support.append(after)
    if not support or not roller_count:
        raise ValueError("could not identify both support and roller facets")
    return support, roller_count


def write_binary_stl(path: Path, facets: list[bytes]) -> None:
    header = b"RB10 EOAT support-only collision; roller is separate link"
    header = header[:80].ljust(80, b"\0")
    path.write_bytes(header + struct.pack("<I", len(facets)) + b"".join(facets))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("original", type=Path)
    parser.add_argument("roller52", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    support, roller_count = split_support_facets(args.original, args.roller52)
    write_binary_stl(args.output, support)
    print(
        f"wrote {args.output}: support_facets={len(support)}, "
        f"separated_roller_facets={roller_count}"
    )


if __name__ == "__main__":
    main()
