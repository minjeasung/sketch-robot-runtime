#!/usr/bin/env python3
"""Seed an isolated official Michelo profile without example physical equipment."""
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
source = ROOT.parent / 'official-michelo-builder/backend/db_profile/db_init'
target = ROOT / '.runtime/construct/michelo-profiles/db_init'
if not target.exists():
    shutil.copytree(source, target)
    # The upstream starter profile contains a real-network PLC example.
    # No hardware identity is invented for the unsupported Isaac input.
    (target / 'hw_interface/hw_interfaces.json').write_text(
        json.dumps({'hw_interfaces': []}, indent=2) + '\n'
    )
print(f'Official backend profile seed: {target}')
