# Construct Isaac Sim setup

User request: download current main and configure Isaac Sim using the two Construct ZIPs in Downloads.

Use a fresh clone of main at ebc36dd676602ad913a187b32762b3aedd15b88f. Keep existing checkout untouched. Import original geometry, fourteen joints, joint limits and initial positions. Preserve MoveIt SRDF groups and collision exclusions. Use installed Isaac Sim 6.1, ROS 2 Jazzy, topic_based_ros2_control, separate ROS domain 105 with localhost discovery. Supply independent Isaac and MoveIt launchers, a saved USD scene, and repeatable tests of simulation state and trajectory execution. Real hardware plugins are never loaded by this launch path.

Scope: Construct robot setup and ROS/MoveIt simulation. Existing sketch painting UI has RB10/RB20-specific assumptions and is not converted to a dual-arm application. Camera meshes are imported; image/depth emulation is outside this setup.

Implementation findings: The source right D435 has 236.6848 kg and an assembly-scale COM. Its 1,252-triangle mesh matches the left D435 to 1e-12 m. Generated simulation URDFs copy left local inertia (0.0817263 kg), while original source stays byte-identical. Head source disables acceleration limits, which Jazzy TOTG rejects; generated planning config supplies simulation-only 0.5 rad/s² and 2 rad/s³ jerk. PhysX receives all original joint velocity/effort limits. Jazzy response adapters retain Ruckig smoothing.
