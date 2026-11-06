import importlib.util
import json
from pathlib import Path
import pytest


def test_export_rejects_cross_session_data_before_video_creation(tmp_path,monkeypatch):
    scripts=Path(__file__).parents[2]/'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec=importlib.util.spec_from_file_location('painting_export',scripts/'render_painting_comparison.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    (tmp_path/'camera.json').write_text(json.dumps(dict(session='new')))
    (tmp_path/'report.json').write_text(json.dumps(dict(session_id='old',passed=True)))
    (tmp_path/'trace.json').write_text(json.dumps(dict(samples=[dict(session='old')])))
    (tmp_path/'measured.jsonl').write_text('{}\n')
    (tmp_path/'frames.jsonl').write_text('{}\n')
    (tmp_path/'events.jsonl').write_text('')
    with pytest.raises(ValueError,match='session'):
        module.render(tmp_path)
    assert not (tmp_path/'annotated.mp4').exists()
