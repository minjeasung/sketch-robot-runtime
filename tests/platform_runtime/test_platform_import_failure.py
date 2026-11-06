"""SimulationApp.close must not convert an importer exception into success."""
from pathlib import Path
from types import SimpleNamespace
import runpy
import sys
import pytest
ROOT=Path(__file__).resolve().parents[2]


def test_import_error_remains_nonzero_when_simulator_close_exits(monkeypatch,tmp_path,capsys):
    class App:
        def __init__(self,*args,**kw):pass
        def close(self,**kwargs):raise SystemExit(kwargs.get('exit_code',0))
    monkeypatch.setitem(sys.modules,'isaacsim',SimpleNamespace(SimulationApp=App))
    monkeypatch.setitem(sys.modules,'charset_normalizer',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'omni',None)  # deterministic import failure; no GPU initialization
    monkeypatch.setattr(sys,'argv',['import_platform_usd.py','--urdf',str(tmp_path/'missing.urdf'),'--output',str(tmp_path/'robot.usd')])
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(ROOT/'scripts/import_platform_usd.py'),run_name='__main__')
    assert exc.value.code==1
    assert 'Traceback' in capsys.readouterr().err
