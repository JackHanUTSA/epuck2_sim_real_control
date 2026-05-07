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
- Webots sim and real-robot adapter classes that normalize motion status into one schema and translate commands into backend-specific actuation
- a Webots controller bootstrap and status reader that can import the real Webots Python controller API and pull pose/twist/wheel data from a robot instance
- a shared pose controller that can run unchanged in sim and real
- an episode runner that logs matched sim/real dataset samples with the same command contract
- runtime source iterators for status streams and camera streams
- a live runtime builder that assembles sim or real episode execution from those sources
- a camera-observation recorder for overhead-camera trajectories collected during real runs
- a camera-ingestion and camera-fusion tracker pipeline for overhead-camera-based real-state estimation
- a dataset writer for matched sim/real motion-status collection
- a YAML session manifest for sim/real configuration
- a `project_info` node/executable that publishes the active manifest plus the Phase 1/Phase 2 architecture summary
- a launch file for quick bring-up sanity checks

Main files:
- `epuck2_sim_real_control/session_manifest.py`
- `epuck2_sim_real_control/contracts.py`
- `epuck2_sim_real_control/adapters.py`
- `epuck2_sim_real_control/webots_connector.py`
- `epuck2_sim_real_control/shared_controller.py`
- `epuck2_sim_real_control/episode_runner.py`
- `epuck2_sim_real_control/runtime_sources.py`
- `epuck2_sim_real_control/live_runtime.py`
- `epuck2_sim_real_control/camera_ingest.py`
- `epuck2_sim_real_control/camera_runtime.py`
- `epuck2_sim_real_control/camera_tracking.py`
- `epuck2_sim_real_control/dataset_logging.py`
- `epuck2_sim_real_control/phase1_architecture.py`
- `epuck2_sim_real_control/phase2_architecture.py`
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
1. instantiate `WebotsStatusReader` inside a real Webots controller process and stream its raw status into `live_runtime.py`
2. connect `RealRobotAdapter` to the physical e-puck2 ROS/driver interface
3. feed real overhead-camera JSONL or live detections into `camera_ingest.py`
4. run `live_runtime.py` from actual sim and real status streams instead of synthetic iterables
5. add sim-vs-real comparison reports and adaptation/system-identification steps
6. extend to multi-robot orchestration after the single-robot pipeline is verified
