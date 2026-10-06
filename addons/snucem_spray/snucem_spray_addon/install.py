"""Verify every archive member before writing only under the owned install root."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile
from .config import TARGET_PLATFORM, validate_paths, owned_path


def verified_members(bundle):
    contents = {}
    total = 0
    with tarfile.open(bundle, 'r:gz') as archive:
        for member in archive:
            path = PurePosixPath(member.name)
            if (not member.isfile() or path.is_absolute() or '..' in path.parts
                    or '\\' in member.name or ':' in member.name or member.name in contents
                    or member.size < 0 or member.size > 10_000_000):
                raise ValueError('unsafe archive member')
            total += member.size
            if total > 10_100_000 or len(contents) > 500:
                raise ValueError('archive exceeds bounded size')
            contents[member.name] = archive.extractfile(member).read()
    try:
        manifest_bytes = contents.pop('manifest.json')
        manifest = json.loads(manifest_bytes)
        if (manifest['schema_version'] != 2 or any(manifest.get(key) != TARGET_PLATFORM[key]
                for key in ('ubuntu', 'ros_distro', 'python', 'architecture', 'cuda', 'zed_sdk'))
                or set(manifest['files']) != set(contents)):
            raise ValueError('manifest does not match archive')
        for name, data in contents.items():
            if hashlib.sha256(data).hexdigest() != manifest['files'][name]:
                raise ValueError('checksum mismatch: '+name)
        if sum(map(len, contents.values())) != manifest['unpacked_bytes']:
            raise ValueError('manifest byte count mismatch')
    except (KeyError, TypeError) as exc:
        raise ValueError('invalid bundle manifest') from exc
    contents['manifest.json'] = manifest_bytes
    return contents


def install_bundle(bundle, config, *, activate=True, service_active=False):
    upstream, install, state = validate_paths(config.upstream_root, config.install_root, config.state_root)
    if service_active or (state/'service-active.json').exists():
        raise ValueError('refusing upgrade while add-on service is active')
    bundle = Path(bundle)
    if bundle.stat().st_size > 5_000_000:
        raise ValueError('bundle exceeds 5 MB')
    contents = verified_members(bundle)
    version = hashlib.sha256(bundle.read_bytes()).hexdigest()[:16]
    destination = owned_path(install, 'versions/'+version)
    if destination.is_symlink() or (install/'versions').is_symlink():
        raise ValueError('version destination must not be a symlink')
    validate_paths(upstream, destination, state)
    install.mkdir(parents=True, exist_ok=True)
    if not destination.exists():
        temporary = Path(tempfile.mkdtemp(prefix='.stage-', dir=install))
        try:
            for name, data in contents.items():
                path = temporary/name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary.rename(destination)
        except Exception:
            # This path was created by this function, resolved under owned install.
            if temporary.resolve().is_relative_to(install) and temporary.exists():
                shutil.rmtree(temporary)
            raise
    else:
        for name, data in contents.items():
            path = destination/name
            if path.is_symlink() or not path.resolve().is_relative_to(destination.resolve()) or path.read_bytes() != data:
                raise ValueError('existing version has been modified')
    if activate:
        activate_version(config, version, destination)
    return dict(version=version, version_root=str(destination), activated=activate)


def activate_version(config, version, destination):
    _, _, state = validate_paths(config.upstream_root, config.install_root, config.state_root)
    if (state/'service-active.json').exists():
        raise ValueError('refusing activation while add-on service is active')
    state.mkdir(parents=True, exist_ok=True)
    temporary = owned_path(state, 'active-version.tmp')
    temporary.write_text(json.dumps(dict(version=version, root=str(destination))), encoding='utf-8')
    temporary.replace(owned_path(state, 'active-version.json'))
