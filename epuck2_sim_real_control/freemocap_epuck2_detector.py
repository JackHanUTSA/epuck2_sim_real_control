from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from epuck2_sim_real_control.contracts import CameraMeasurement, Pose2D


@dataclass(frozen=True)
class DetectionResult:
    center_x_px: float
    center_y_px: float
    heading_rad: float
    confidence: float


@dataclass(frozen=True)
class _Candidate:
    pixels: np.ndarray
    score: float


def detect_epuck2_measurement(
    frame: np.ndarray,
    *,
    timestamp: float,
    pixels_to_world: float,
    min_confidence: float = 0.5,
    world_origin_px: tuple[float, float] | None = None,
    invert_image_y: bool = True,
) -> CameraMeasurement | None:
    _validate_measurement_arguments(
        timestamp=timestamp,
        pixels_to_world=pixels_to_world,
        min_confidence=min_confidence,
        world_origin_px=world_origin_px,
    )

    detection = detect_epuck2_body(frame)
    if detection is None:
        return None
    if detection.confidence < float(min_confidence):
        return None

    origin_x, origin_y = world_origin_px if world_origin_px is not None else ((frame.shape[1] - 1) / 2.0, (frame.shape[0] - 1) / 2.0)
    world_x = (float(detection.center_x_px) - float(origin_x)) * float(pixels_to_world)
    if invert_image_y:
        world_y = (float(origin_y) - float(detection.center_y_px)) * float(pixels_to_world)
        heading = -float(detection.heading_rad)
    else:
        world_y = (float(detection.center_y_px) - float(origin_y)) * float(pixels_to_world)
        heading = float(detection.heading_rad)

    return CameraMeasurement(
        timestamp=float(timestamp),
        pose=Pose2D(
            x=world_x,
            y=world_y,
            theta=heading,
        ),
        tracking_confidence=float(detection.confidence),
    )


def detect_epuck2_body(frame: np.ndarray) -> DetectionResult | None:
    normalized = _normalize_frame(frame)
    body_mask = normalized >= 0.5
    candidates = _extract_candidates(body_mask)
    if not candidates:
        return None

    candidate = max(candidates, key=lambda item: item.score)
    body_pixels = candidate.pixels
    if _covers_entire_frame(body_pixels, normalized.shape):
        return None
    if _is_border_hugging_elongated_component(body_pixels, normalized.shape):
        return None
    center_y = float(body_pixels[:, 0].mean())
    center_x = float(body_pixels[:, 1].mean())

    cue_pixels = _extract_cue_pixels(normalized, body_pixels)
    heading_strength = 0.0
    heading = 0.0
    if cue_pixels.shape[0] > 0:
        cue_weights = normalized[cue_pixels[:, 0], cue_pixels[:, 1]]
        cue_y = float(np.average(cue_pixels[:, 0], weights=cue_weights))
        cue_x = float(np.average(cue_pixels[:, 1], weights=cue_weights))
        delta_x = cue_x - center_x
        delta_y = cue_y - center_y
        heading_strength = min(1.0, math.hypot(delta_x, delta_y) / 8.0)
        if heading_strength > 0.0:
            heading = math.atan2(delta_y, delta_x)

    body_fraction = min(1.0, float(body_pixels.shape[0]) / 300.0)
    cue_fraction = min(1.0, float(cue_pixels.shape[0]) / max(1.0, float(body_pixels.shape[0]) * 0.08)) if cue_pixels.shape[0] else 0.0
    confidence = max(0.0, min(1.0, 0.45 * body_fraction + 0.35 * candidate.score + 0.20 * min(heading_strength, cue_fraction)))
    if heading_strength <= 0.0:
        confidence *= 0.45

    return DetectionResult(
        center_x_px=center_x,
        center_y_px=center_y,
        heading_rad=heading,
        confidence=confidence,
    )


def _validate_measurement_arguments(
    *,
    timestamp: float,
    pixels_to_world: float,
    min_confidence: float,
    world_origin_px: tuple[float, float] | None,
) -> None:
    if not math.isfinite(float(timestamp)):
        raise ValueError('timestamp must be finite')
    if not math.isfinite(float(pixels_to_world)) or float(pixels_to_world) <= 0.0:
        raise ValueError('pixels_to_world must be a finite positive scale')
    if not math.isfinite(float(min_confidence)) or not 0.0 <= float(min_confidence) <= 1.0:
        raise ValueError('min_confidence must be between 0.0 and 1.0')
    if world_origin_px is not None:
        if len(world_origin_px) != 2:
            raise ValueError('world_origin_px must contain exactly two coordinates')
        if not all(math.isfinite(float(value)) for value in world_origin_px):
            raise ValueError('world_origin_px must contain finite coordinates')


def _extract_candidates(body_mask: np.ndarray) -> list[_Candidate]:
    visited = np.zeros_like(body_mask, dtype=bool)
    height, width = body_mask.shape
    candidates: list[_Candidate] = []

    for row in range(height):
        for col in range(width):
            if visited[row, col] or not body_mask[row, col]:
                continue
            pixels = _connected_component(body_mask, visited, row, col)
            if pixels.shape[0] < 25:
                continue
            score = _candidate_score(pixels)
            candidates.append(_Candidate(pixels=pixels, score=score))
    return candidates


def _connected_component(mask: np.ndarray, visited: np.ndarray, start_row: int, start_col: int) -> np.ndarray:
    queue: deque[tuple[int, int]] = deque([(start_row, start_col)])
    visited[start_row, start_col] = True
    pixels: list[tuple[int, int]] = []
    height, width = mask.shape

    while queue:
        row, col = queue.popleft()
        pixels.append((row, col))
        for d_row in (-1, 0, 1):
            for d_col in (-1, 0, 1):
                if d_row == 0 and d_col == 0:
                    continue
                next_row = row + d_row
                next_col = col + d_col
                if next_row < 0 or next_col < 0 or next_row >= height or next_col >= width:
                    continue
                if visited[next_row, next_col] or not mask[next_row, next_col]:
                    continue
                visited[next_row, next_col] = True
                queue.append((next_row, next_col))

    return np.asarray(pixels, dtype=int)


def _candidate_score(pixels: np.ndarray) -> float:
    min_row = int(pixels[:, 0].min())
    max_row = int(pixels[:, 0].max())
    min_col = int(pixels[:, 1].min())
    max_col = int(pixels[:, 1].max())
    height = max_row - min_row + 1
    width = max_col - min_col + 1
    bbox_area = float(height * width)
    fill_ratio = float(pixels.shape[0]) / bbox_area
    aspect_ratio = min(width, height) / max(width, height)
    area_score = min(1.0, float(pixels.shape[0]) / 250.0)
    fill_score = max(0.0, min(1.0, fill_ratio / 0.8))
    aspect_score = max(0.0, min(1.0, aspect_ratio / 0.85))
    return max(0.0, min(1.0, 0.35 * area_score + 0.35 * fill_score + 0.30 * aspect_score))


def _extract_cue_pixels(normalized: np.ndarray, body_pixels: np.ndarray) -> np.ndarray:
    cue_rows = body_pixels[:, 0]
    cue_cols = body_pixels[:, 1]
    intensities = normalized[cue_rows, cue_cols]
    if intensities.size == 0:
        return np.empty((0, 2), dtype=int)
    cue_threshold = max(0.85, float(intensities.mean()) + 0.15)
    cue_mask = intensities >= cue_threshold
    cue_pixels = body_pixels[cue_mask]
    if cue_pixels.shape[0] < 1:
        return np.empty((0, 2), dtype=int)
    return cue_pixels


def _covers_entire_frame(body_pixels: np.ndarray, image_shape: tuple[int, int]) -> bool:
    image_height, image_width = image_shape
    min_row = int(body_pixels[:, 0].min())
    max_row = int(body_pixels[:, 0].max())
    min_col = int(body_pixels[:, 1].min())
    max_col = int(body_pixels[:, 1].max())
    return min_row == 0 and min_col == 0 and max_row == image_height - 1 and max_col == image_width - 1


def _is_border_hugging_elongated_component(body_pixels: np.ndarray, image_shape: tuple[int, int]) -> bool:
    image_height, image_width = image_shape
    min_row = int(body_pixels[:, 0].min())
    max_row = int(body_pixels[:, 0].max())
    min_col = int(body_pixels[:, 1].min())
    max_col = int(body_pixels[:, 1].max())
    height = max_row - min_row + 1
    width = max_col - min_col + 1
    aspect_ratio = min(width, height) / max(width, height)
    touches_border = min_row == 0 or min_col == 0 or max_row == image_height - 1 or max_col == image_width - 1
    return touches_border and aspect_ratio < 0.2


def _normalize_frame(frame: np.ndarray) -> np.ndarray:
    array = np.asarray(frame, dtype=float)
    if array.ndim == 3:
        array = array.mean(axis=2)
    if array.ndim != 2:
        raise ValueError('frame must be a 2D grayscale image or a 3D color image')

    max_value = float(array.max()) if array.size else 0.0
    min_value = float(array.min()) if array.size else 0.0
    if max_value <= min_value:
        return np.zeros_like(array, dtype=float)
    return (array - min_value) / (max_value - min_value)
