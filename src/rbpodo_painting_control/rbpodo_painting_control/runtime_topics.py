"""Shared active-arm force wiring for the controller, monitor and executor."""
import re


PAINTING_CONTROL_TOPICS = (
    '/admittance_controller/compliance_active',
    '/admittance_controller/compliance_enable',
    '/admittance_controller/normal_limit_reached',
    '/admittance_controller/roller_balance/limit_reached',
    '/admittance_controller/wrench_reference',
    '/force_torque_sensor_broadcaster/wrench',
    '/force_torque_sensor_broadcaster_raw/wrench',
    '/painting_admittance/abort',
    '/painting_admittance/abort_reason',
    '/painting_admittance/command_force_tcp_y_n',
    '/painting_admittance/contact_confirmed',
    '/painting_admittance/contact_state',
    '/painting_admittance/controller_fault',
    '/painting_admittance/current_mode',
    '/painting_admittance/desired_force_n',
    '/painting_admittance/enable_force',
    '/painting_admittance/executor_heartbeat',
    '/painting_admittance/force_tcp_filtered',
    '/painting_admittance/force_tcp_raw',
    '/painting_admittance/free_space_confirmed',
    '/painting_admittance/mode',
    '/painting_admittance/normal_force_tcp_y',
    '/painting_admittance/overforce_state',
    '/painting_admittance/ramp_complete',
    '/painting_admittance/ramp_status',
    '/painting_admittance/requested_wrench_reference',
    '/painting_admittance/reset_safety',
    '/painting_admittance/reset_wrench_guard',
    '/painting_admittance/robot_stationary',
    '/painting_admittance/safety_status',
    '/painting_admittance/target_wrench_ft',
    '/painting_admittance/target_wrench_tcp',
    '/painting_admittance/trajectory_active',
    '/painting_admittance/wrench_guard_status',
    '/painting_system/hardware_motion_inhibited',
    '/rbpodo_ft_tare/runtime_free_space_tare',
)


def painting_control_remappings(namespace):
    namespace=str(namespace).rstrip('/')
    if not namespace:
        return []
    if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*',namespace):
        raise ValueError('Force-control namespace must be an absolute ROS namespace')
    return [(topic,namespace+topic) for topic in PAINTING_CONTROL_TOPICS]
