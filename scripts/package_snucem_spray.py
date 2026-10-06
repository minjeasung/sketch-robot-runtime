#!/usr/bin/env python3
"""Build the standalone Humble add-on, excluding upstream and model assets."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'addons/snucem_spray'))
from snucem_spray_addon.packaging import build_bundle

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    path, manifest = build_bundle(ROOT, args.output)
    print(f'{path}: compressed={path.stat().st_size}, owned files={manifest["unpacked_bytes"]} bytes')
