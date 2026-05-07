# FreeMoCap Markerless E-puck2 Camera Pipeline Implementation Plan

> For Hermes: Use subagent-driven-development skill to implement this plan task-by-task.

Goal: Add a markerless overhead-camera pipeline that uses FreeMoCap for calibrated capture and produces CameraMeasurement records for the e-puck2 sim-to-real stack.

Architecture: Treat FreeMoCap as the acquisition and calibration layer, not the robot detector. Add a dedicated top-view detector that estimates the e-puck2 body pose `(x, y, theta)` from overhead frames using contour/appearance cues, then export those observations into the existing `CameraMeasurement` and `camera_tracking.py` path.

Tech Stack: Python, OpenCV, NumPy, FreeMoCap recordings/calibration outputs, existing `epuck2_sim_real_control` camera contracts.

---

## Scope and assumptions
- No fiducial tags or markers on the robot.
- The first version targets one overhead camera path, with FreeMoCap still allowed to calibrate/manage multi-camera sessions.
- The detector should run offline on recorded frames first, then be adapted to live frames.
- Output format must remain compatible with `camera_ingest.py`, `camera_runtime.py`, and `camera_tracking.py`.

## Existing code to integrate with
- `epuck2_sim_real_control/contracts.py`
- `epuck2_sim_real_control/camera_ingest.py`
- `epuck2_sim_real_control/camera_runtime.py`
- `epuck2_sim_real_control/camera_tracking.py`
- `epuck2_sim_real_control/live_runtime.py`
- `epuck2_sim_real_control/episode_runner.py`

## Proposed new files
- `epuck2_sim_real_control/freemocap_epuck2_detector.py`
- `epuck2_sim_real_control/freemocap_session_loader.py`
- `test/test_freemocap_epuck2_detector.py`
- `test/test_freemocap_session_loader.py`
- optional later: `scripts/run_freemocap_epuck2_detection.py`

---

### Task 1: Define the FreeMoCap-to-CameraMeasurement interface

Objective: Lock down the exact data contract between FreeMoCap-derived detection output and the existing camera pipeline.

Files:
- Modify: `README.md`
- Create: `docs/freemocap-markerless-design.md`

Steps:
1. Write a short design note documenting:
   - expected input frame source
   - world-coordinate assumptions
   - required output fields: `timestamp`, `pose.x`, `pose.y`, `pose.theta`, `tracking_confidence`
   - confidence semantics
2. Add one JSONL example line showing the final detector output.
3. Update README to mention the FreeMoCap markerless direction.
4. Verify by reading the docs and checking that the output format matches `camera_measurement_from_dict(...)`.

### Task 2: Add a session loader for FreeMoCap recordings

Objective: Read FreeMoCap session folders without coupling detector logic to the GUI/runtime layout.

Files:
- Create: `epuck2_sim_real_control/freemocap_session_loader.py`
- Test: `test/test_freemocap_session_loader.py`

Steps:
1. Write a failing test for locating a session directory and enumerating candidate video files.
2. Implement a minimal loader that:
   - accepts a session root path
   - discovers likely synchronized video files
   - exposes frame rate and a deterministic frame timestamp helper
3. Run the targeted test.
4. Commit.

Verification command:
- `pytest -q test/test_freemocap_session_loader.py`

### Task 3: Implement a first markerless e-puck2 detector on synthetic top-view frames

Objective: Build the first detector around body-shape/top-view cues before touching real FreeMoCap footage.

Files:
- Create: `epuck2_sim_real_control/freemocap_epuck2_detector.py`
- Test: `test/test_freemocap_epuck2_detector.py`

Steps:
1. Write failing tests for:
   - detecting the robot center from a synthetic circular/elliptical body mask
   - estimating heading from an asymmetric front cue
   - rejecting frames with low confidence
2. Implement minimal detector pieces:
   - grayscale or color preprocessing
   - contour extraction
   - candidate scoring by size/circularity
   - heading estimation from front-half asymmetry or brightest directional cue
   - `DetectionResult -> CameraMeasurement`
3. Run targeted tests until green.
4. Commit.

Verification command:
- `pytest -q test/test_freemocap_epuck2_detector.py`

### Task 4: Add an offline recording-to-JSONL exporter

Objective: Convert a recorded FreeMoCap session into camera observations that the existing runtime can ingest.

Files:
- Modify: `epuck2_sim_real_control/freemocap_epuck2_detector.py`
- Modify: `epuck2_sim_real_control/camera_runtime.py` or create helper module
- Test: `test/test_camera_runtime.py` or a new dedicated export test

Steps:
1. Write a failing test that feeds synthetic frame timestamps and detection results into an output JSONL writer.
2. Implement an exporter that writes one `CameraMeasurement` per accepted frame.
3. Reuse `CameraObservationRecorder` where possible instead of duplicating JSONL logic.
4. Verify the output loads through `load_camera_measurements_jsonl(...)`.
5. Commit.

Verification commands:
- `pytest -q test/test_camera_runtime.py test/test_camera_ingest.py`

### Task 5: Integrate offline camera detections with the existing runtime

Objective: Prove the FreeMoCap-derived camera observations can flow into the current sim/real fusion path.

Files:
- Modify: `epuck2_sim_real_control/live_runtime.py`
- Modify: `README.md`
- Test: `test/test_live_runtime.py`

Steps:
1. Add a small integration path that accepts the exported JSONL as camera input.
2. Write or extend a test showing that offline camera detections are fused by `CameraTracker` in real mode.
3. Update README with the expected workflow:
   - record FreeMoCap session
   - run detector/exporter
   - feed JSONL into runtime
4. Run tests.
5. Commit.

Verification command:
- `pytest -q test/test_live_runtime.py test/test_camera_tracking.py`

### Task 6: Add real-footage calibration notes and failure modes

Objective: Make the first real-world capture reproducible.

Files:
- Modify: `docs/freemocap-markerless-design.md`
- Modify: `README.md`

Steps:
1. Document capture constraints:
   - overhead placement
   - static background
   - stable lighting
   - visible heading cue on the robot body itself
   - pixel footprint large enough for contour stability
2. Document common failure modes:
   - shadows merging with the body
   - heading ambiguity from a symmetric top view
   - occlusion by cables/hands
   - mismatch between camera/world axes and runtime axes
3. Add a first-session checklist.
4. Commit.

---

## Detector design notes
- First-pass detection should be classical CV, not a learned model.
- Start with one-camera offline processing before live mode.
- Heading estimation likely needs a natural asymmetry on the robot body: front sensor geometry, LED pattern, or battery/casing texture visible from above.
- If heading is unreliable from a single raw silhouette, keep position confidence separate from heading confidence and degrade `tracking_confidence` accordingly.

## JSONL output example
```json
{"timestamp": 12.450, "pose": {"x": 0.312, "y": -0.184, "theta": 1.047}, "tracking_confidence": 0.91}
```

## Test strategy
- Unit tests on synthetic masks and frame metadata first.
- Fixture-based tests on a tiny recorded frame set next.
- Runtime integration tests last.

## Exit criteria
- A FreeMoCap recording can be processed into camera-observation JSONL.
- JSONL loads through `load_camera_measurements_jsonl(...)`.
- `CameraTracker` can fuse those measurements into the existing runtime path.
- README and design docs are sufficient for repeating the workflow.
