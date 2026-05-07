# FreeMoCap Markerless E-puck2 Camera Pipeline

## Goal
Use FreeMoCap as the calibrated capture layer for real-world e-puck2 observation, then run a custom markerless top-view detector that produces `CameraMeasurement` JSONL compatible with the existing repo.

## Why this exists
FreeMoCap is good for synchronized capture and calibration, but it is not already an e-puck2 detector. The missing piece is a robot-body pose estimator that converts frames into `(x, y, theta, confidence)` observations.

## Data contract
The detector output must stay compatible with:
- `epuck2_sim_real_control/camera_ingest.py`
- `epuck2_sim_real_control/camera_runtime.py`
- `epuck2_sim_real_control/camera_tracking.py`

Required JSONL schema:
```json
{"timestamp": 12.450, "pose": {"x": 0.312, "y": -0.184, "theta": 1.047}, "tracking_confidence": 0.91}
```

## Proposed pipeline
1. Capture a session with FreeMoCap.
2. Load the recording and camera metadata from the session folder.
3. Select the overhead-view stream for the first detector pass.
4. Detect the e-puck2 body from each frame using classical CV.
5. Estimate center position and heading from robot-body cues.
6. Convert accepted detections into `CameraMeasurement` records.
7. Write JSONL with `CameraObservationRecorder`.
8. Feed JSONL into the existing runtime and fusion path.

## First detector design
Use classical CV first:
- background-friendly preprocessing
- thresholding / edge extraction
- contour filtering by size and circularity
- candidate scoring
- heading estimation from a natural asymmetry on the robot body

Possible heading cues without markers:
- front sensor geometry visible from above
- asymmetric shell texture
- LED pattern if stable and intentionally controlled
- wheel cutout / front opening shape if resolution is sufficient

## Coordinate expectations
- `x` and `y` should be in a stable world frame, not image pixels, before export if calibration is available
- `theta` should be aligned with the same convention expected by `Pose2D`
- timestamps should be frame-derived and monotonic

## Confidence guidance
`tracking_confidence` should be reduced when:
- multiple body candidates score similarly
- contour shape is badly occluded
- heading cue is weak or ambiguous
- motion blur is high
- segmentation depends heavily on unstable lighting

## Real-world capture constraints
- overhead camera should be as close to top-down as possible
- static background preferred
- avoid glossy reflections on the floor
- keep the robot large enough in frame for contour and heading extraction
- maintain stable lighting
- avoid people/hands entering the field during test runs

## Failure modes to watch
- circular body produces heading ambiguity if no strong natural asymmetry is visible
- shadows merge with the robot and distort contour shape
- low-resolution recording makes theta unreliable
- coordinate-frame mismatch between camera world frame and runtime world frame
- FreeMoCap session layout or filenames may vary across versions

## Recommended implementation order
1. synthetic-frame detector tests
2. FreeMoCap session loader
3. offline exporter to JSONL
4. runtime integration
5. first real recording validation

## Related files
- `docs/plans/2026-05-07-freemocap-markerless-epuck2-pipeline.md`
- `epuck2_sim_real_control/camera_ingest.py`
- `epuck2_sim_real_control/camera_runtime.py`
- `epuck2_sim_real_control/camera_tracking.py`
- `epuck2_sim_real_control/live_runtime.py`
