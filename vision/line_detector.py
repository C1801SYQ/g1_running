from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class LaneDetectorConfig:
    """Parameters for detecting two white lane boundaries in an RGB image."""

    roi_top_ratio: float = 0.38
    lookahead_ratio: float = 0.22
    white_value_min: int = 175
    white_saturation_max: int = 100
    adaptive_value_floor: int = 55
    adaptive_value_margin: int = 18
    adaptive_value_smoothing: float = 0.35
    max_neutral_chroma: int = 72
    local_contrast_min: int = 14
    local_contrast_kernel: int = 41
    morphology_kernel: int = 3
    vertical_close_kernel: int = 9
    min_component_area: float = 90.0
    min_vertical_span_ratio: float = 0.22
    min_lane_width_ratio: float = 0.20
    max_lane_width_ratio: float = 0.95
    max_top_to_bottom_width_ratio: float = 1.25
    min_depth_m: float = 0.35
    max_depth_m: float = 12.0
    keep_invalid_depth: bool = True
    expected_lane_width_ratio: float = 0.52
    vertical_fov_degrees: float = 72.0
    max_abs_heading_error_rad: float = 0.75
    initial_center_weight: float = 0.80
    temporal_pair_weight: float = 5.00
    max_boundary_step_ratio: float = 0.18
    max_common_shift_ratio: float = 0.20
    max_lane_width_change_ratio: float = 0.25
    max_pair_center_offset_ratio: float = 0.48
    lock_update_alpha: float = 0.12
    guided_search_margin_ratio: float = 0.075
    guided_min_pixels: int = 70
    guided_min_vertical_span_ratio: float = 0.18


@dataclass(frozen=True)
class LineModel:
    """A lane line represented as x = slope * y + intercept."""

    slope: float
    intercept: float
    score: float
    y_min: float
    y_max: float

    def x_at(self, y: float) -> float:
        return self.slope * y + self.intercept


@dataclass
class DetectionResult:
    valid: bool
    lateral_error: float = 0.0
    heading_error_rad: float = 0.0
    confidence: float = 0.0
    source: str = "none"
    left_line: Optional[LineModel] = None
    right_line: Optional[LineModel] = None
    roi_y: int = 0
    lookahead_y: int = 0
    image_width: int = 0
    image_height: int = 0
    value_threshold: float = 0.0
    mask: Optional[np.ndarray] = None


class WhiteLaneDetector:
    """Detect both white lane boundaries using RGB and optional depth."""

    def __init__(self, config: LaneDetectorConfig | None = None):
        self.config = config or LaneDetectorConfig()
        self._locked_pair_geometry: Optional[np.ndarray] = None
        self._adaptive_value_threshold: Optional[float] = None

    def reset(self) -> None:
        """Forget the selected lane so the next valid pair becomes the target."""

        self._locked_pair_geometry = None

    def detect(
        self, rgb: np.ndarray, depth_m: Optional[np.ndarray] = None
    ) -> DetectionResult:
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must have shape (height, width, 3)")

        height, width = rgb.shape[:2]
        if depth_m is not None and depth_m.shape != (height, width):
            raise ValueError("depth_m must match the RGB height and width")

        roi_y = int(round(height * self.config.roi_top_ratio))
        roi_y = int(np.clip(roi_y, 0, height - 2))
        roi_height = height - roi_y
        lookahead_y = roi_y + int(round(roi_height * self.config.lookahead_ratio))
        # Evaluate lane width and lateral offset on a near-field row where a
        # D435i-class field of view still contains both 2.1 m boundaries. At
        # the literal bottom row one boundary is often outside the image; line
        # extrapolation there would falsely reject a clearly observed pair.
        bottom_y = roi_y + int(round(roi_height * 0.62))
        bottom_y = int(np.clip(bottom_y, lookahead_y + 1, height - 1))

        mask, value_threshold = self._build_white_mask(rgb, roi_y)

        if depth_m is not None:
            finite = np.isfinite(depth_m)
            measured = finite & (depth_m > 0.0)
            outside = measured & (
                (depth_m < self.config.min_depth_m)
                | (depth_m > self.config.max_depth_m)
            )
            mask[outside] = 0
            if not self.config.keep_invalid_depth:
                mask[~measured] = 0

        kernel_size = max(3, int(self.config.morphology_kernel) | 1)
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        close_height = max(kernel_size, int(self.config.vertical_close_kernel) | 1)
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, close_height)
        )
        # Closing first reconnects white-line fragments produced by motion blur
        # and exposure flicker.  The smaller opening then removes isolated
        # highlights without erasing a distant, narrow boundary.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)

        candidates = self._extract_candidates(mask, roi_y, roi_height)
        pair = self._choose_pair(candidates, width, lookahead_y, bottom_y)
        pair_source = "two-lines"
        if pair is None:
            # A fast gait can split each physical boundary into several short
            # contours.  Search around the already locked geometry and refit
            # both boundaries from actual mask pixels.  This is still strict
            # two-line observation; no missing boundary is synthesized.
            pair = self._guided_pair_from_mask(
                mask,
                width,
                roi_y,
                roi_height,
                lookahead_y,
                bottom_y,
            )
            if pair is not None:
                pair_source = "two-lines-guided"

        if pair is not None:
            left, right = pair
            result = self._result_from_pair(
                left,
                right,
                width,
                height,
                roi_y,
                lookahead_y,
                bottom_y,
                mask,
                value_threshold,
                pair_source,
            )
            if (
                abs(result.heading_error_rad)
                > self.config.max_abs_heading_error_rad
            ):
                return DetectionResult(
                    valid=False,
                    source="none",
                    roi_y=roi_y,
                    lookahead_y=lookahead_y,
                    image_width=width,
                    image_height=height,
                    value_threshold=value_threshold,
                    mask=mask,
                )
            self._remember_pair(left, right, width, lookahead_y, bottom_y)
            return result

        return DetectionResult(
            valid=False,
            source="none",
            roi_y=roi_y,
            lookahead_y=lookahead_y,
            image_width=width,
            image_height=height,
            value_threshold=value_threshold,
            mask=mask,
        )

    def _build_white_mask(
        self, rgb: np.ndarray, roi_y: int
    ) -> tuple[np.ndarray, float]:
        """Segment neutral bright paint under changing exposure and shadows."""

        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        roi_value = value[roi_y:, :]

        if roi_value.size:
            otsu_threshold, _ = cv2.threshold(
                roi_value,
                0,
                255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU,
            )
            raw_threshold = float(
                np.clip(
                    otsu_threshold + self.config.adaptive_value_margin,
                    self.config.adaptive_value_floor,
                    self.config.white_value_min,
                )
            )
        else:
            raw_threshold = float(self.config.white_value_min)

        if self._adaptive_value_threshold is None:
            self._adaptive_value_threshold = raw_threshold
        else:
            configured_alpha = float(
                np.clip(self.config.adaptive_value_smoothing, 0.0, 1.0)
            )
            # Adapt quickly when entering a shadow, but raise the threshold
            # more slowly so one bright frame cannot make the next frame fail.
            alpha = (
                max(configured_alpha, 0.70)
                if raw_threshold < self._adaptive_value_threshold
                else configured_alpha
            )
            self._adaptive_value_threshold += alpha * (
                raw_threshold - self._adaptive_value_threshold
            )
        value_threshold = float(self._adaptive_value_threshold)

        rgb_i16 = rgb.astype(np.int16)
        chroma = rgb_i16.max(axis=2) - rgb_i16.min(axis=2)
        neutral = (
            (saturation <= self.config.white_saturation_max)
            & (chroma <= self.config.max_neutral_chroma)
        )

        kernel_size = max(9, int(self.config.local_contrast_kernel) | 1)
        contrast_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)
        )
        local_contrast = cv2.morphologyEx(
            value, cv2.MORPH_TOPHAT, contrast_kernel
        )
        absolute_white = value >= value_threshold
        locally_bright = (
            (value >= self.config.adaptive_value_floor)
            & (local_contrast >= self.config.local_contrast_min)
        )
        white = neutral & (absolute_white | locally_bright)
        white[:roi_y, :] = False

        mask = np.zeros(value.shape, dtype=np.uint8)
        mask[white] = 255
        return mask, value_threshold

    def _extract_candidates(
        self, mask: np.ndarray, roi_y: int, roi_height: int
    ) -> list[LineModel]:
        contours, _ = cv2.findContours(
            mask[roi_y:, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        candidates: list[LineModel] = []
        min_span = roi_height * self.config.min_vertical_span_ratio

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.min_component_area:
                continue
            points = contour.reshape(-1, 2).astype(np.float64)
            points[:, 1] += roi_y
            y_min = float(points[:, 1].min())
            y_max = float(points[:, 1].max())
            span = y_max - y_min
            if span < min_span:
                continue

            fitted = self._fit_line(points, roi_height, y_min, y_max)
            if fitted is None:
                continue
            area_score = float(np.clip(area / (roi_height * 18.0), 0.0, 1.0))
            score = 0.75 * fitted.score + 0.25 * area_score
            candidates.append(
                LineModel(
                    slope=fitted.slope,
                    intercept=fitted.intercept,
                    score=score,
                    y_min=y_min,
                    y_max=y_max,
                )
            )

        return candidates

    def _choose_pair(
        self,
        candidates: list[LineModel],
        width: int,
        lookahead_y: int,
        bottom_y: int,
    ) -> Optional[tuple[LineModel, LineModel]]:
        best: Optional[tuple[LineModel, LineModel]] = None
        best_score = -np.inf
        min_width = width * self.config.min_lane_width_ratio
        max_width = width * self.config.max_lane_width_ratio
        expected = width * self.config.expected_lane_width_ratio
        image_center = 0.5 * width

        for index, first in enumerate(candidates):
            for second in candidates[index + 1 :]:
                left, right = sorted(
                    (first, second), key=lambda line: line.x_at(bottom_y)
                )
                bottom_width = right.x_at(bottom_y) - left.x_at(bottom_y)
                top_width = right.x_at(lookahead_y) - left.x_at(lookahead_y)
                if not (min_width <= bottom_width <= max_width):
                    continue
                if top_width <= 0.04 * width:
                    continue
                if top_width > bottom_width * self.config.max_top_to_bottom_width_ratio:
                    continue

                width_match = np.exp(-abs(bottom_width - expected) / max(expected, 1.0))
                pair_center = 0.5 * (
                    left.x_at(bottom_y) + right.x_at(bottom_y)
                )
                if (
                    abs(pair_center - image_center)
                    > self.config.max_pair_center_offset_ratio * width
                ):
                    # A neighbouring lane can be fully visible, but it does
                    # not contain the camera's near-field ground ray. Never
                    # use such a pair for steering.
                    continue
                center_match = np.exp(
                    -abs(pair_center - image_center) / max(0.35 * width, 1.0)
                )
                temporal_match = 0.0
                if self._locked_pair_geometry is not None:
                    geometry = self._pair_geometry(
                        left, right, width, lookahead_y, bottom_y
                    )
                    previous = self._locked_pair_geometry
                    # Near-field boundary positions remain stable under the
                    # large torso pitch oscillation of the running gait. Far
                    # positions do not, so they must not be a hard identity
                    # gate. Switching to an adjacent lane still replaces one
                    # near boundary and produces roughly a full-lane jump.
                    near_delta = geometry[:2] - previous[:2]
                    common_shift = float(abs(np.mean(near_delta)))
                    deformation = float(
                        np.max(np.abs(near_delta - np.mean(near_delta)))
                    )
                    previous_width = previous[1] - previous[0]
                    width_change = abs(
                        (geometry[1] - geometry[0]) - previous_width
                    )
                    # Roll/yaw shake moves both boundaries together.  Treat
                    # that common image translation separately from a shape
                    # change, while retaining a tighter absolute limit that
                    # rejects a complete jump into the neighbouring lane.
                    if deformation > self.config.max_boundary_step_ratio:
                        continue
                    if common_shift > self.config.max_common_shift_ratio:
                        continue
                    if width_change > (
                        self.config.max_lane_width_change_ratio
                        * max(previous_width, 1e-6)
                    ):
                        continue
                    temporal_distance = max(deformation, 0.65 * common_shift)
                    temporal_match = float(np.exp(-temporal_distance / 0.10))

                score = (
                    left.score
                    + right.score
                    + 0.35 * float(width_match)
                    + self.config.initial_center_weight * float(center_match)
                    + self.config.temporal_pair_weight * temporal_match
                )
                if score > best_score:
                    best_score = score
                    best = (left, right)
        return best

    def _guided_pair_from_mask(
        self,
        mask: np.ndarray,
        width: int,
        roi_y: int,
        roi_height: int,
        lookahead_y: int,
        bottom_y: int,
    ) -> Optional[tuple[LineModel, LineModel]]:
        """Refit two observed fragmented boundaries near the locked lane."""

        if self._locked_pair_geometry is None:
            return None

        y_pixels, x_pixels = np.nonzero(mask[roi_y:, :])
        if y_pixels.size == 0:
            return None
        y_pixels = y_pixels.astype(np.float64) + roi_y
        x_pixels = x_pixels.astype(np.float64)
        geometry = self._locked_pair_geometry
        margin = max(4.0, width * self.config.guided_search_margin_ratio)
        min_span = roi_height * self.config.guided_min_vertical_span_ratio

        fitted: list[LineModel] = []
        for top_x_ratio, bottom_x_ratio in (
            (geometry[2], geometry[0]),
            (geometry[3], geometry[1]),
        ):
            slope = (
                width * (bottom_x_ratio - top_x_ratio)
                / max(float(bottom_y - lookahead_y), 1.0)
            )
            predicted_x = width * top_x_ratio + slope * (
                y_pixels - lookahead_y
            )
            selected = np.abs(x_pixels - predicted_x) <= margin
            if int(np.count_nonzero(selected)) < self.config.guided_min_pixels:
                return None

            points = np.column_stack((x_pixels[selected], y_pixels[selected]))
            y_min = float(points[:, 1].min())
            y_max = float(points[:, 1].max())
            if y_max - y_min < min_span:
                return None

            line = self._fit_line(points, roi_height, y_min, y_max)
            if line is None:
                return None
            fitted.append(line)

        return self._choose_pair(fitted, width, lookahead_y, bottom_y)

    @staticmethod
    def _fit_line(
        points: np.ndarray,
        roi_height: int,
        y_min: float,
        y_max: float,
    ) -> Optional[LineModel]:
        """Fit x(y) with a Huber loss so glare and blur tails have low weight."""

        if len(points) < 2:
            return None
        vx, vy, x0, y0 = cv2.fitLine(
            points.astype(np.float32).reshape(-1, 1, 2),
            cv2.DIST_HUBER,
            0,
            0.01,
            0.01,
        ).reshape(-1)
        if abs(float(vy)) < 1e-5:
            return None
        slope = float(vx / vy)
        intercept = float(x0 - slope * y0)
        residual = points[:, 0] - (slope * points[:, 1] + intercept)
        median_error = float(np.median(np.abs(residual)))
        span = y_max - y_min
        span_score = float(np.clip(span / max(roi_height, 1), 0.0, 1.0))
        support_score = float(
            np.clip(len(points) / max(roi_height * 10.0, 1.0), 0.0, 1.0)
        )
        straightness = float(np.exp(-median_error / 18.0))
        score = 0.50 * span_score + 0.30 * support_score + 0.20 * straightness
        return LineModel(
            slope=slope,
            intercept=intercept,
            score=score,
            y_min=y_min,
            y_max=y_max,
        )

    @staticmethod
    def _pair_geometry(
        left: LineModel,
        right: LineModel,
        width: int,
        lookahead_y: int,
        bottom_y: int,
    ) -> np.ndarray:
        """Return normalized boundary positions at near and far image rows."""

        return np.asarray(
            (
                left.x_at(bottom_y) / width,
                right.x_at(bottom_y) / width,
                left.x_at(lookahead_y) / width,
                right.x_at(lookahead_y) / width,
            ),
            dtype=np.float64,
        )

    def _remember_pair(
        self,
        left: LineModel,
        right: LineModel,
        width: int,
        lookahead_y: int,
        bottom_y: int,
    ) -> None:
        geometry = self._pair_geometry(
            left, right, width, lookahead_y, bottom_y
        )
        if self._locked_pair_geometry is None:
            self._locked_pair_geometry = geometry
            return
        alpha = self.config.lock_update_alpha
        self._locked_pair_geometry = (
            alpha * geometry + (1.0 - alpha) * self._locked_pair_geometry
        )

    def _result_from_pair(
        self,
        left: LineModel,
        right: LineModel,
        width: int,
        height: int,
        roi_y: int,
        lookahead_y: int,
        bottom_y: int,
        mask: np.ndarray,
        value_threshold: float,
        source: str,
    ) -> DetectionResult:
        left_bottom = left.x_at(bottom_y)
        right_bottom = right.x_at(bottom_y)
        left_top = left.x_at(lookahead_y)
        right_top = right.x_at(lookahead_y)
        center_bottom = 0.5 * (left_bottom + right_bottom)
        center_top = 0.5 * (left_top + right_top)
        lateral_error = (center_bottom - 0.5 * width) / (0.5 * width)
        # The slope of the image-space lane center mixes lateral displacement
        # with yaw. The vanishing point of the two boundaries is independent of
        # lateral displacement on a straight track, so it is a much cleaner
        # estimate of camera/track heading.
        slope_delta = left.slope - right.slope
        if abs(slope_delta) > 1e-6:
            vanishing_y = (right.intercept - left.intercept) / slope_delta
            vanishing_x = left.x_at(vanishing_y)
            focal_px = (0.5 * height) / np.tan(
                np.radians(0.5 * self.config.vertical_fov_degrees)
            )
            heading_error = np.arctan2(vanishing_x - 0.5 * width, focal_px)
        else:
            heading_error = np.arctan2(
                center_top - center_bottom, bottom_y - lookahead_y
            )
        confidence = float(np.clip(0.5 * (left.score + right.score), 0.0, 1.0))
        return DetectionResult(
            valid=True,
            lateral_error=float(np.clip(lateral_error, -2.0, 2.0)),
            heading_error_rad=float(heading_error),
            confidence=confidence,
            source=source,
            left_line=left,
            right_line=right,
            roi_y=roi_y,
            lookahead_y=lookahead_y,
            image_width=width,
            image_height=height,
            value_threshold=value_threshold,
            mask=mask,
        )

    @staticmethod
    def annotate(rgb: np.ndarray, result: DetectionResult) -> np.ndarray:
        canvas = rgb.copy()
        height, width = canvas.shape[:2]
        if result.roi_y:
            cv2.line(canvas, (0, result.roi_y), (width - 1, result.roi_y), (80, 80, 255), 1)

        def draw_line(
            line: Optional[LineModel],
            color: tuple[int, int, int],
            label: str,
        ) -> None:
            if line is None:
                return
            y1 = max(result.roi_y, int(round(line.y_min)))
            y2 = min(height - 1, int(round(line.y_max)))
            p1 = (int(round(line.x_at(y1))), y1)
            p2 = (int(round(line.x_at(y2))), y2)
            cv2.line(canvas, p1, p2, color, 5, cv2.LINE_AA)
            text_x = int(np.clip(p1[0] + 8, 4, max(4, width - 150)))
            text_y = int(np.clip(p1[1] + 22, 20, height - 5))
            cv2.putText(
                canvas,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
                cv2.LINE_AA,
            )

        draw_line(
            result.left_line,
            (255, 140, 40),
            "LEFT",
        )
        draw_line(
            result.right_line,
            (40, 230, 80),
            "RIGHT",
        )

        if result.valid and result.left_line and result.right_line:
            y_bottom = height - 1
            y_top = result.lookahead_y
            center_bottom = int(
                round(
                    0.5
                    * (
                        result.left_line.x_at(y_bottom)
                        + result.right_line.x_at(y_bottom)
                    )
                )
            )
            center_top = int(
                round(
                    0.5
                    * (
                        result.left_line.x_at(y_top)
                        + result.right_line.x_at(y_top)
                    )
                )
            )
            cv2.line(
                canvas,
                (center_bottom, y_bottom),
                (center_top, y_top),
                (255, 220, 20),
                3,
                cv2.LINE_AA,
            )
            cv2.line(
                canvas,
                (width // 2, height - 1),
                (width // 2, result.roi_y),
                (220, 80, 220),
                1,
                cv2.LINE_AA,
            )

        status = (
            f"{'VALID' if result.valid else 'LOST'} {result.source} "
            f"conf={result.confidence:.2f} V>={result.value_threshold:.0f}"
        )
        errors = (
            f"offset={result.lateral_error:+.3f} "
            f"heading={np.degrees(result.heading_error_rad):+.2f}deg"
        )
        cv2.rectangle(canvas, (8, 8), (520, 64), (0, 0, 0), -1)
        cv2.putText(
            canvas,
            status,
            (16, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (80, 255, 80) if result.valid else (255, 80, 80),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            errors,
            (16, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return canvas
