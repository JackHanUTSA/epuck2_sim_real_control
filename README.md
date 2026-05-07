# epuck2_sim_real_control

New project scaffold for e-puck2 sim-to-real control in ROS 2.

Goal:
- use Webots as the simulation environment for e-puck2 controller development
- design one controller in simulation and run the same controller on the real e-puck2
- keep simulation and real-robot interfaces aligned so datasets are comparable
- capture motion-status datasets in both sim and real runs
- use an overhead camera to recover real-world trajectories/dynamics and improve real-state estimation
- make it easy to add safety checks before real deployment

Current scaffold includes:
- a shared motion/status contract for sim and real adapters
- Webots sim and real-robot adapter classes that normalize motion status into one schema
- a camera-fusion tracker for overhead-camera-based real-state estimation
- a dataset writer for matched sim/real motion-status collection
- a YAML session manifest for sim/real configuration
- a `project_info` node/executable that publishes the active manifest plus the Phase 1 architecture summary
- a launch file for quick bring-up sanity checks

Main files:
- `epuck2_sim_real_control/session_manifest.py`
- `epuck2_sim_real_control/contracts.py`
- `epuck2_sim_real_control/adapters.py`
- `epuck2_sim_real_control/camera_tracking.py`
- `epuck2_sim_real_control/dataset_logging.py`
- `epuck2_sim_real_control/phase1_architecture.py`
- `epuck2_sim_real_control/project_info.py`
- `config/default_session.yaml`
- `launch/epuck2_sim_real_control.launch.py`

Quick start:

```zsh
cd ~/ws_xarm
source /opt/ros/humble/setup.zsh
colcon build --packages-select epuck2_sim_real_control
source ~/ws_xarm/install/setup.zsh
ros2 run epuck2_sim_real_control project_info
```

Launch with explicit mode:

```zsh
ros2 launch epuck2_sim_real_control epuck2_sim_real_control.launch.py mode:=sim
```

Next good steps:
1. connect `WebotsSimAdapter` to real Webots supervisor/controller topics
2. connect `RealRobotAdapter` to the physical e-puck2 driver interface
3. add the shared controller module that issues the same `VelocityCommand` in sim and real
4. log matched sim and real `DatasetSample` streams for the same controller behaviors
5. connect the overhead camera pipeline to `CameraTracker` for state-estimation fusion
6. add sim-vs-real comparison reports and adaptation/system-identification steps
