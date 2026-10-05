"""Version changes must round-trip without touching unrelated work or staging."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[1]/'plane_versions.py'


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args])


@pytest.fixture
def repo(tmp_path):
    git(tmp_path, 'init', '-q')
    git(tmp_path, 'config', 'user.name', 'Test')
    git(tmp_path, 'config', 'user.email', 'test@example.invalid')
    (tmp_path/'config').mkdir()
    config = {'paths': ['algorithm.py', 'extra.py'], 'versions': {
        key: {'ref': 'refs/tags/plane-extraction/'+key, 'label': key}
        for key in ('v0', 'v1', 'v2')}}
    (tmp_path/'config/plane_extraction_versions.json').write_text(json.dumps(config))
    (tmp_path/'unrelated.txt').write_text('original')
    for version in ('v0', 'v1', 'v2'):
        (tmp_path/'algorithm.py').write_text(version)
        if version != 'v0':
            (tmp_path/'extra.py').write_text(version)
        git(tmp_path, 'add', '.')
        git(tmp_path, 'commit', '-qm', version)
        git(tmp_path, 'tag', 'plane-extraction/'+version)
    (tmp_path/'unrelated.txt').write_text('user staged changes')
    git(tmp_path, 'add', 'unrelated.txt')
    (tmp_path/'unrelated.txt').write_text('user unstaged changes')
    return tmp_path


def run(repo, *args):
    return subprocess.run([sys.executable, str(SCRIPT), '--root', str(repo), *args],
                          capture_output=True, text=True)


def test_switch_roundtrips_only_versioned_files_preserving_the_index(repo):
    index = (repo/'.git/index').read_bytes()
    for version in ('v0', 'v1', 'v2'):
        result = run(repo, 'switch', version)
        assert result.returncode == 0, result.stderr
        assert (repo/'algorithm.py').read_text() == version
        assert (repo/'extra.py').exists() == (version != 'v0')
        assert (repo/'unrelated.txt').read_text() == 'user unstaged changes'
        assert (repo/'.git/index').read_bytes() == index
        assert json.loads(run(repo, 'status').stdout)['version'] == version


def test_modified_algorithm_blocks_all_writes(repo):
    (repo/'extra.py').write_text('new unrecorded experiment')
    result = run(repo, 'switch', 'v0')
    assert result.returncode != 0
    assert 'extra.py' in result.stderr
    assert (repo/'algorithm.py').read_text() == 'v2'
    assert (repo/'extra.py').read_text() == 'new unrecorded experiment'


def test_missing_snapshot_blocks_all_writes(repo):
    git(repo, 'tag', '-d', 'plane-extraction/v0')
    assert run(repo, 'switch', 'v0').returncode != 0
    assert (repo/'algorithm.py').read_text() == 'v2'
    assert (repo/'extra.py').read_text() == 'v2'


def test_fetched_version_branch_works_without_a_local_tag(repo):
    sha = git(repo, 'rev-parse', 'refs/tags/plane-extraction/v0').decode().strip()
    git(repo, 'update-ref', 'refs/remotes/origin/plane-extraction/v0', sha)
    git(repo, 'tag', '-d', 'plane-extraction/v0')
    result = run(repo, 'switch', 'v0')
    assert result.returncode == 0, result.stderr
    assert (repo/'algorithm.py').read_text() == 'v0'
    assert not (repo/'extra.py').exists()


def test_dry_run_reports_changes_without_applying(repo):
    result = run(repo, 'switch', 'v0', '--dry-run')
    assert result.returncode == 0, result.stderr
    assert set(json.loads(result.stdout)['changed']) == {'algorithm.py', 'extra.py'}
    assert (repo/'algorithm.py').read_text() == 'v2'
    assert (repo/'extra.py').read_text() == 'v2'


def test_symlink_is_not_followed_or_overwritten(repo, tmp_path):
    external = repo.parent/(repo.name+'-external')
    external.write_text('v2')
    (repo/'algorithm.py').unlink()
    (repo/'algorithm.py').symlink_to(external)
    assert run(repo, 'switch', 'v0').returncode != 0
    assert external.read_text() == 'v2'
    assert (repo/'algorithm.py').is_symlink()
