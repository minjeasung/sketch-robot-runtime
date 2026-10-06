#!/usr/bin/env python3
"""Compare unchanged legacy contact tests against the pinned parent on Humble.

Existing failures stay visible. A new failure, skip or missing test fails CI;
this is not a claim that the legacy contact suite passes.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

TEST = 'src/sketch_control/test/test_moveit_executor_fail_closed.py'


def outcomes(root, other, report):
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    inherited = [p for p in env.get('PYTHONPATH', '').split(os.pathsep) if p
                 and not Path(p).resolve().is_relative_to(root)
                 and not Path(p).resolve().is_relative_to(other)]
    env['PYTHONPATH'] = os.pathsep.join([str(root/'src/sketch_control'),
        str(root/'src/rbpodo_painting_control'), *inherited])
    result = subprocess.run([sys.executable, '-m', 'pytest', '-q', TEST,
                             '--tb=short', '--junitxml', str(report)], cwd=root, env=env)
    if result.returncode not in (0, 1) or not report.is_file():
        raise RuntimeError('baseline comparison could not collect/run '+str(root))
    cases = {}
    for case in ET.parse(report).iter('testcase'):
        key = case.get('classname')+'::'+case.get('name')
        cases[key] = next((kind for kind in ('error', 'failure', 'skipped') if case.find(kind) is not None), 'passed')
    if not cases:
        raise RuntimeError('empty regression comparison')
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--current', required=True, type=Path)
    parser.add_argument('--reports', required=True, type=Path)
    args = parser.parse_args()
    base, current, reports = (p.resolve() for p in (args.baseline, args.current, args.reports))
    reports.mkdir(parents=True, exist_ok=True)
    before = outcomes(base, current, reports/'baseline.xml')
    after = outcomes(current, base, reports/'current.xml')
    regressions = [name for name, state in after.items() if state != 'passed' and state != before.get(name)]
    missing = sorted(set(before)-set(after))
    known = [name for name, state in after.items() if state != 'passed' and state == before.get(name)]
    report = dict(tests=len(after), unchanged_legacy_failures=known, new_regressions=regressions, missing=missing)
    (reports/'comparison.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 1 if regressions or missing else 0


if __name__ == '__main__':
    raise SystemExit(main())
