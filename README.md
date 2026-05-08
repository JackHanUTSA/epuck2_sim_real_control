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
- a Webots live controller entrypoint that runs a Robot loop, applies wheel commands, and writes dataset samples while the robot moves
- a shared pose controller that can run unchanged in sim and real
- an episode runner that logs matched sim/real dataset samples with the same command contract
- runtime source iterators for status streams and camera streams
- a live runtime builder that assembles sim or real episode execution from those sources
- a camera-observation recorder for overhead-camera trajectories collected during real runs
- a camera-ingestion and camera-fusion tracker pipeline for overhead-camera-based real-state estimation
- a FreeMoCap session loader, markerless e-puck2 detector scaffold, and offline JSONL exporter for turning recorded overhead footage into runtime-ingestible camera observations
- a dataset writer for matched sim/real motion-status collection
- a YAML session manifest for sim/real configuration
- a `project_info` node/executable that publishes the active manifest plus the Phase 1/Phase 2 architecture summary
- a launch file for quick bring-up sanity checks

Main files:
- `epuck2_sim_real_control/session_manifest.py`
- `epuck2_sim_real_control/contracts.py`
- `epuck2_sim_real_control/adapters.py`
- `epuck2_sim_real_control/webots_connector.py`
- `epuck2_sim_real_control/webots_live_controller.py`
- `epuck2_sim_real_control/shared_controller.py`
- `epuck2_sim_real_control/episode_runner.py`
- `epuck2_sim_real_control/runtime_sources.py`
- `epuck2_sim_real_control/live_runtime.py`
- `epuck2_sim_real_control/camera_ingest.py`
- `epuck2_sim_real_control/freemocap_detection_cli.py`
- `epuck2_sim_real_control/freemocap_session_loader.py`
- `epuck2_sim_real_control/freemocap_epuck2_detector.py`
- `epuck2_sim_real_control/camera_runtime.py`
- `epuck2_sim_real_control/camera_tracking.py`
- `epuck2_sim_real_control/dataset_logging.py`
- `epuck2_sim_real_control/phase1_architecture.py`
- `epuck2_sim_real_control/phase2_architecture.py`
- `epuck2_sim_real_control/project_info.py`
- `config/default_session.yaml`
- `launch/epuck2_sim_real_control.launch.py`
- `scripts/webots_epuck2_controller.py`

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

FreeMoCap offline detection workflow:

The current camera path is offline-first: record a FreeMoCap session, export e-puck2 detections into JSONL, then feed that JSONL into the runtime/fusion stack.

Detection assumptions for the e-puck2:
- use a mostly top-down overhead view
- keep the robot large enough in frame for the body contour to be visible
- maintain stable lighting and a static background when possible
- leave a visible natural heading cue on the robot body itself, because a perfectly symmetric top view makes `theta` ambiguous

Example export command:

```zsh
cd ~/ws_xarm
source /opt/ros/humble/setup.zsh
colcon build --packages-select epuck2_sim_real_control
source ~/ws_xarm/install/setup.zsh
ros2 run epuck2_sim_real_control run_freemocap_epuck2_detection \
  --session-root ~/freemocap_data/my_epuck2_session \
  --output-path ~/freemocap_data/my_epuck2_session/camera_observations.jsonl \
  --pixels-to-world 0.0025 \
  --world-origin-px 640 360 \
  --min-confidence 0.6
```

Equivalent direct script entrypoint:

```zsh
python3 scripts/run_freemocap_epuck2_detection.py \
  --session-root ~/freemocap_data/my_epuck2_session \
  --pixels-to-world 0.0025
```

After export, the JSONL file can be supplied to the existing runtime builder as the `camera_source` input so the e-puck2 detections are fused into the real-state estimate.

Next good steps:
1. validate the detector against a real FreeMoCap overhead recording where the e-puck2 is clearly visible
2. connect `RealRobotAdapter` to the physical e-puck2 ROS/driver interface
3. feed real overhead-camera JSONL or live detections into `camera_ingest.py`
4. run `live_runtime.py` from actual sim and real status streams instead of synthetic iterables
5. add sim-vs-real comparison reports and adaptation/system-identification steps
6. extend to multi-robot orchestration after the single-robot pipeline is verified
