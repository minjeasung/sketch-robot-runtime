"""Allow-listed simulation launch contract for every selected arm/process."""
from .robot_models import model_execution_contract


def launch_contract(selection):
    if selection.execution_mode not in ('simulation','mixed'):raise ValueError('Simulator cannot start physical mode')
    model='rb10_1300e_u' if selection.platform=='rb10' else 'construct_'+selection.active_arm
    arm=model_execution_contract(model)
    return dict(model_id=model,joints=list(arm['arm_joint_names']),tcp=arm['ee_link'],base_frame=arm['base_frame'],
                controller=arm['trajectory_controller'],group=arm['planning_group'],
                hardware_plugin='isaac_painting_hardware/IsaacPaintingSystem' if selection.process=='paint' else 'topic_based_ros2_control/TopicBasedSystem',
                virtual_cameras=selection.execution_mode=='simulation',physical_cameras=selection.execution_mode=='mixed',
                weld_provider='external' if selection.execution_mode=='mixed' else 'fixture',native_weld_tool=True,
                force_pipeline=selection.process=='paint',require_task_scene_ack=selection.execution_mode=='mixed')


def build_platform_specs(root,bundle):
    from .process_supervisor import ProcessSpec
    return [ProcessSpec('simulation','상용로봇/RB10 · Isaac · 선택 공구·카메라',(),
                         ('bash','scripts/run_platform_stack.sh','--assets',bundle['directory']),
                         ('platform_simulation','controller_manager','move_group','moveit_executor','painting_rosbridge_websocket'))]
