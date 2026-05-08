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
    focus_roi_px: tuple[float, float, float, float] | None = None,
) -> CameraMeasurement | None:
    _validate_measurement_arguments(
        timestamp=timestamp,
        pixels_to_world=pixels_to_world,
        min_confidence=min_confidence,
        world_origin_px=world_origin_px,
        focus_roi_px=focus_roi_px,
    )

    detection = None
    if focus_roi_px is not None:
        detection = _detect_epuck2_body_in_focus_roi(np.asarray(frame), focus_roi_px, min_confidence=float(min_confidence))
        if detection is not None and detection.confidence < float(min_confidence):
            detection = None
    if detection is None:
        detection = _detect_epuck2_body_with_arena_focus(frame, min_confidence=float(min_confidence))
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
    return _detect_epuck2_body_from_normalized(normalized)


def _detect_epuck2_body_with_arena_focus(frame: np.ndarray, *, min_confidence: float) -> DetectionResult | None:
    normalized = _normalize_frame(frame)
    direct_detection = _detect_epuck2_body_from_normalized(normalized)
    focused_detection = _detect_epuck2_body_in_dark_arena(normalized)
    if focused_detection is None:
        return direct_detection
    direct_confidence = direct_detection.confidence if direct_detection is not None else 0.0
    if focused_detection.confidence >= max(float(min_confidence), direct_confidence + 0.05):
        return focused_detection
    if direct_detection is None:
        return focused_detection
    return direct_detection


def _detect_epuck2_body_from_normalized(normalized: np.ndarray) -> DetectionResult | None:
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


def _detect_epuck2_body_in_dark_arena(normalized: np.ndarray) -> DetectionResult | None:
    arena_roi = _find_dark_arena_roi(normalized)
    if arena_roi is None:
        return None
    left, top, width, height = arena_roi
    focused_detection = _detect_epuck2_body_from_normalized(normalized[top:top + height, left:left + width])
    if focused_detection is None:
        return None
    return DetectionResult(
        center_x_px=focused_detection.center_x_px + float(left),
        center_y_px=focused_detection.center_y_px + float(top),
        heading_rad=focused_detection.heading_rad,
        confidence=focused_detection.confidence,
    )


def _detect_epuck2_body_in_focus_roi(
    frame: np.ndarray,
    focus_roi_px: tuple[float, float, float, float],
    min_confidence: float,
) -> DetectionResult | None:
    image_height, image_width = frame.shape[:2]
    left = max(0, int(round(float(focus_roi_px[0]))))
    top = max(0, int(round(float(focus_roi_px[1]))))
    width = max(1, int(round(float(focus_roi_px[2]))))
    height = max(1, int(round(float(focus_roi_px[3]))))
    right = min(image_width, left + width)
    bottom = min(image_height, top + height)
    if right <= left or bottom <= top:
        return None
    crop = np.asarray(frame)[top:bottom, left:right]
    whole_crop_detection = detect_epuck2_body(crop)
    if whole_crop_detection is not None and whole_crop_detection.confidence >= float(min_confidence):
        return DetectionResult(
            center_x_px=whole_crop_detection.center_x_px + float(left),
            center_y_px=whole_crop_detection.center_y_px + float(top),
            heading_rad=whole_crop_detection.heading_rad,
            confidence=whole_crop_detection.confidence,
        )
    best_detection: DetectionResult | None = whole_crop_detection
    best_offset = (left, top)
    crop_height, crop_width = crop.shape[:2]

    search_windows = [(0, 0, crop_width, crop_height)]
    for frac in (0.85, 0.7, 0.55):
        window_width = max(64, int(round(crop_width * frac)))
        window_height = max(64, int(round(crop_height * frac)))
        if window_width > crop_width or window_height > crop_height:
            continue
        x_positions = sorted({0, max(0, (crop_width - window_width) // 2), max(0, crop_width - window_width)})
        y_positions = sorted({0, max(0, (crop_height - window_height) // 2), max(0, crop_height - window_height)})
        for crop_left in x_positions:
            for crop_top in y_positions:
                search_windows.append((crop_left, crop_top, window_width, window_height))

    for crop_left, crop_top, window_width, window_height in search_windows:
        search_crop = crop[crop_top:crop_top + window_height, crop_left:crop_left + window_width]
        resized_crop = _resize_linear(search_crop, image_height, image_width)
        focused_detection = detect_epuck2_body(resized_crop)
        if focused_detection is None:
            continue
        focused_detection = DetectionResult(
            center_x_px=focused_detection.center_x_px * (float(window_width) / float(image_width)),
            center_y_px=focused_detection.center_y_px * (float(window_height) / float(image_height)),
            heading_rad=focused_detection.heading_rad,
            confidence=focused_detection.confidence,
        )
        if best_detection is None or focused_detection.confidence > best_detection.confidence:
            best_detection = focused_detection
            best_offset = (left + crop_left, top + crop_top)

    if best_detection is None:
        return None
    offset_left, offset_top = best_offset
    return DetectionResult(
        center_x_px=best_detection.center_x_px + float(offset_left),
        center_y_px=best_detection.center_y_px + float(offset_top),
        heading_rad=best_detection.heading_rad,
        confidence=best_detection.confidence,
    )


def _validate_measurement_arguments(
    *,
    timestamp: float,
    pixels_to_world: float,
    min_confidence: float,
    world_origin_px: tuple[float, float] | None,
    focus_roi_px: tuple[float, float, float, float] | None,
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
    if focus_roi_px is not None:
        if len(focus_roi_px) != 4:
            raise ValueError('focus_roi_px must contain exactly four values: left, top, width, height')
        if not all(math.isfinite(float(value)) for value in focus_roi_px):
            raise ValueError('focus_roi_px must contain finite coordinates')
        if float(focus_roi_px[2]) <= 0.0 or float(focus_roi_px[3]) <= 0.0:
            raise ValueError('focus_roi_px width and height must be positive')


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


def _find_dark_arena_roi(normalized: np.ndarray) -> tuple[int, int, int, int] | None:
    dark_mask = normalized <= 0.2
    visited = np.zeros_like(dark_mask, dtype=bool)
    image_height, image_width = dark_mask.shape
    best_roi: tuple[int, int, int, int] | None = None
    best_score = -1.0

    for row in range(image_height):
        for col in range(image_width):
            if visited[row, col] or not dark_mask[row, col]:
                continue
            pixels = _connected_component(dark_mask, visited, row, col)
            if pixels.shape[0] < max(100, (image_height * image_width) // 500):
                continue
            min_row = int(pixels[:, 0].min())
            max_row = int(pixels[:, 0].max())
            min_col = int(pixels[:, 1].min())
            max_col = int(pixels[:, 1].max())
            height = max_row - min_row + 1
            width = max_col - min_col + 1
            if width < image_width * 0.2 or height < image_height * 0.2:
                continue
            if min_row == 0 or min_col == 0 or max_row == image_height - 1 or max_col == image_width - 1:
                continue
            bbox_area = float(width * height)
            fill_ratio = float(pixels.shape[0]) / bbox_area
            if not 0.01 <= fill_ratio <= 0.2:
                continue
            aspect_ratio = min(width, height) / max(width, height)
            if aspect_ratio < 0.45:
                continue
            area_score = bbox_area / float(image_width * image_height)
            frame_like_score = 1.0 - min(1.0, abs(fill_ratio - 0.08) / 0.08)
            score = 0.7 * area_score + 0.3 * frame_like_score
            if score <= best_score:
                continue
            inset = max(6, int(round(min(width, height) * 0.04)))
            inner_left = min_col + inset
            inner_top = min_row + inset
            inner_right = max_col - inset
            inner_bottom = max_row - inset
            if inner_right <= inner_left or inner_bottom <= inner_top:
                continue
            best_roi = (inner_left, inner_top, inner_right - inner_left + 1, inner_bottom - inner_top + 1)
            best_score = score
    return best_roi


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


def _resize_linear(frame: np.ndarray, output_height: int, output_width: int) -> np.ndarray:
    if output_height <= 0 or output_width <= 0:
        raise ValueError('output dimensions must be positive')
    array = np.asarray(frame)
    input_height, input_width = array.shape[:2]
    if input_height == output_height and input_width == output_width:
        return array

    x_old = np.arange(input_width, dtype=float)
    x_new = np.linspace(0.0, float(input_width - 1), output_width)
    y_old = np.arange(input_height, dtype=float)
    y_new = np.linspace(0.0, float(input_height - 1), output_height)

    if array.ndim == 2:
        horizontal = np.vstack([np.interp(x_new, x_old, row.astype(float)) for row in array])
        vertical = np.vstack([np.interp(y_new, y_old, horizontal[:, col]) for col in range(output_width)]).T
        return vertical.astype(array.dtype)

    channels = []
    for channel_index in range(array.shape[2]):
        channel = array[:, :, channel_index].astype(float)
        horizontal = np.vstack([np.interp(x_new, x_old, row) for row in channel])
        vertical = np.vstack([np.interp(y_new, y_old, horizontal[:, col]) for col in range(output_width)]).T
        channels.append(vertical)
    return np.stack(channels, axis=2).astype(array.dtype)
