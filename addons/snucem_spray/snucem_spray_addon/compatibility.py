"""Read-only verification of the exact upstream interfaces inspected for Humble."""
import hashlib
from pathlib import Path

UPSTREAM_REPOSITORY = 'JongHyunSeo11/SNUCEM_Robot_22.04'
UPSTREAM_REVISION = '7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c'
INTERFACES = {
    'linux/control/rb20_spray/node.py': '298a611e4e0d8c3b2a35f2d05c1fdfc279255037',
    'linux/control/rb20_spray/node_base.py': 'ff2fe37168e5644f4e01b02000b4805972a109a0',
    'linux/control/rb20_spray/plane_support.py': 'aca9bffc738f6fa54cacf19ca5b571c7b707a55c',
    'linux/control/rb20_spray/hbeam_memory.py': 'c9502fb85c0ec37266e8f79c6b9dbb759d3eddc7',
    'linux/control/rb20_spray/stack_model.py': '6726a04634e5adda8420657933e71ce8e6901782',
}


def check_upstream(root):
    root = Path(root).resolve()
    missing, changed = [], []
    for name, expected in INTERFACES.items():
        path = root / name
        if not path.is_file() or not path.resolve().is_relative_to(root):
            missing.append(name)
            continue
        data = path.read_bytes().replace(b'\r\n', b'\n')
        actual = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        if actual != expected:
            changed.append(name)
    return dict(repository=UPSTREAM_REPOSITORY, supported_revision=UPSTREAM_REVISION,
                compatible=not (missing or changed), missing=missing, changed=changed)


def require_upstream(root):
    report = check_upstream(root)
    if not report['compatible']:
        raise ValueError('unsupported upstream interfaces: ' + ', '.join(report['missing'] + report['changed']))
    return report
