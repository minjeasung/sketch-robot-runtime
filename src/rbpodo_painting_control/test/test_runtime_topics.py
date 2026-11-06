from pathlib import Path
import importlib.util
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def test_arm_remaps_force_commands_feedback_and_tare_but_keeps_global_abort():
    import rbpodo_painting_control.runtime_topics as runtime
    remaps=dict(runtime.painting_control_remappings('/platform/right/'))
    for topic in ('/painting_admittance/mode','/painting_admittance/wrench_guard_status',
                  '/painting_admittance/reset_safety','/admittance_controller/wrench_reference',
                  '/force_torque_sensor_broadcaster/wrench','/painting_system/hardware_motion_inhibited',
                  '/rbpodo_ft_tare/runtime_free_space_tare'):
        assert remaps[topic]=='/platform/right'+topic
    assert '/motion_abort' not in remaps
    assert '/painting_system/process_mode' not in remaps
    assert runtime.painting_control_remappings('')==[]


def test_namespaced_launch_keeps_tuning_yaml_but_overrides_selected_frames():
    import pytest
    pytest.importorskip('launch_ros')
    path=Path(__file__).resolve().parents[1]/'launch/painting_admittance_control.launch.py'
    spec=importlib.util.spec_from_file_location('painting_launch',path)
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    layers=launch._parameter_layers('config.yaml',{'dry_run':False,'force_filter_tau_s':99.,'ft_frame':'legacy'},
                                    wiring_overrides={'ft_frame':'left_ft','tcp_frame':'left_tcp'})
    assert layers==['config.yaml',{'dry_run':False},{'ft_frame':'left_ft','tcp_frame':'left_tcp'}]


@pytest.mark.parametrize('restart_only', [False, True])
def test_shared_launch_forwards_public_stop_policy_to_both_force_nodes_with_yaml(monkeypatch,restart_only):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    path=Path(__file__).resolve().parents[1]/'launch/painting_admittance_control.launch.py'
    spec=importlib.util.spec_from_file_location('painting_stop_policy_launch',path)
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    captured=[]
    monkeypatch.setattr(launch,'Node',lambda **kwargs: captured.append(kwargs) or object())
    description=launch.generate_launch_description()
    context=LaunchContext()
    context.launch_configurations.update(config_file='/tmp/paint-profile.yaml',namespace='/platform/left')
    if restart_only:
        context.launch_configurations['public_motion_abort_requires_restart']='true'
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    for action in description.entities:
        if isinstance(action,OpaqueFunction):action.execute(context)
    targets=[item for item in captured if item.get('name') in ('painting_wrench_reference','painting_wrench_guard')]
    assert len(targets)==2
    for target in targets:
        layers=[layer for layer in target['parameters'] if isinstance(layer,dict)]
        values={key:value for layer in evaluate_parameters(context,normalize_parameters(layers)) for key,value in layer.items()}
        assert values['public_motion_abort_requires_restart'] is restart_only
