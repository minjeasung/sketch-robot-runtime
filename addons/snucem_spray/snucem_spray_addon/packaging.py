"""Deterministic source-only distribution with an explicit dependency closure."""
import ast
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile
from . import __version__
from .config import TARGET_PLATFORM

ENTRY_MODULES = ('sketch_control.moveit_executor', 'sketch_control.wall_projector_node',
                 'sketch_control.sketch_to_waypoints_node', 'sketch_control.zed_preview_node')


def source_closure(root):
    pending, seen = list(ENTRY_MODULES), set()
    while pending:
        name = pending.pop()
        if name in seen:
            continue
        package = name.split('.')[0]
        if package not in ('sketch_control', 'rbpodo_painting_control'):
            continue
        path = root/'src'/package/Path(*name.split('.')).with_suffix('.py')
        if not path.is_file():
            raise ValueError('unresolved packaged module: '+name)
        seen.add(name)
        yield path
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            if isinstance(node, ast.ImportFrom):
                dependency = node.module or ''
                if node.level:
                    dependency = '.'.join(name.split('.')[:-node.level]+([dependency] if dependency else []))
                if dependency.startswith(('sketch_control.', 'rbpodo_painting_control.')):
                    pending.append(dependency)
            elif isinstance(node, ast.Import):
                pending.extend(alias.name for alias in node.names if alias.name.startswith(('sketch_control.', 'rbpodo_painting_control.')))


def build_bundle(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    paths = list(source_closure(root))
    paths += list((root/'addons/snucem_spray/snucem_spray_addon').glob('*.py'))
    paths += [root/'addons/snucem_spray/requirements-humble.txt']
    paths += [root/'scripts/snucem_spray.py']
    paths += [root/'src'/p/p/'__init__.py' for p in ('sketch_control', 'rbpodo_painting_control')]
    # Only assets used by the Sketch page and the add-on manager are shipped.
    paths += [root/'web'/p for p in ('index.html', 'addon.html', 'style.css', 'sketch.css')]
    paths += [root/'web/js'/p for p in ('app.js', 'roslib.min.js', 'target_refine_gate.js',
              'zed_surface_gate.js', 'image_topic.js', 'plane_workflow.js')]
    for doc in ('docs/SNUCEM_SPRAY_ADDON.md', 'docs/SNUCEM_SPRAY_VALIDATION.md'):
        if (root/doc).is_file():
            paths.append(root/doc)
    contents = {}
    for path in sorted(set(paths)):
        if path.is_symlink() or not path.resolve().is_relative_to(root):
            raise ValueError('source symlink outside bundle boundary')
        contents[path.relative_to(root).as_posix()] = path.read_bytes().replace(b'\r\n', b'\n')
    for package in ('sketch_control', 'rbpodo_painting_control'):
        contents['share/ament_index/resource_index/packages/'+package] = b''
    # A placeholder target only; the external executor refuses to publish a
    # scene until a measured dynamic surface exists, and renames this owned ID.
    contents['share/sketch_control/config/objects.yaml'] = (
        'active_target: wall\nobjects:\n  - name: wall\n    shape: box\n    position: [0, 0, 0]\n'
        '    size: [1, 1, 0.02]\n    sketch_face: "+z"\n    enabled: true\n').encode()
    unpacked = sum(map(len, contents.values()))
    if unpacked > 10_000_000:
        raise ValueError('unpacked add-on exceeds 10 MB')
    manifest = dict(schema_version=2, name='snucem-spray-humble-arm64-cuda12.6', version=__version__,
                    upstream_repository='JongHyunSeo11/SNUCEM_Robot_22.04',
                    upstream_revision='7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c',
                    **TARGET_PLATFORM, unpacked_bytes=unpacked,
                    cuda_dependency='external_camera_only',
                    dependencies_included=False,
                    files={name:hashlib.sha256(data).hexdigest() for name, data in sorted(contents.items())})
    contents['manifest.json'] = json.dumps(manifest, sort_keys=True, indent=2).encode()
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w', format=tarfile.USTAR_FORMAT) as archive:
        for name, data in sorted(contents.items()):
            member = tarfile.TarInfo(name)
            member.size, member.mode, member.mtime = len(data), 0o644, 0
            archive.addfile(member, io.BytesIO(data))
    compressed = gzip.compress(buffer.getvalue(), compresslevel=9, mtime=0)
    if len(compressed) > 5_000_000:
        raise ValueError('compressed add-on exceeds 5 MB')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(compressed)
    output.with_suffix(output.suffix+'.sha256').write_text(hashlib.sha256(compressed).hexdigest()+'  '+output.name+'\n')
    output.with_suffix(output.suffix+'.manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return output, manifest
