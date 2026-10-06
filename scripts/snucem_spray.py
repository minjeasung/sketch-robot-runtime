#!/usr/bin/env python3
"""Run from a source archive without cloning or installing the full repository."""
from pathlib import Path
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'addons/snucem_spray'))
from snucem_spray_addon.cli import main
if __name__ == '__main__':
    main()
