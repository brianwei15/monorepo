import cv2
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RaftDetectorConfig:
    # -----------------------------------------------------------------------
    # Orange HSV — calibrated from actual Intex Explorer 300 raft pixels:
    #   H mean ≈ 7-14, full range 3-25
    #   S mean ≈ 167-194, shadow areas down to ~80
    #   V mean ≈ 175-216, shadowed areas down to ~55
    # -----------------------------------------------------------------------
    orange_lower1: np.ndarray = field(
        default_factory=lambda: np.array([3,  80,  55]))
    orange_upper1: np.ndarray = field(
        default_factory=lambda: np.array([22, 255, 255]))

    # Deeper shadow / partially-deflated texture
    orange_lower2: np.ndarray = field(
        default_factory=lambda: np.array([0,  60,  40]))
    orange_upper2: np.ndarray = field(
        default_factory=lambda: np.array([6,  255, 180]))

    # Black trim (confirms raft structure)
    black_lower: np.ndarray = field(
        default_factory=lambda: np.array([0,   0,   0]))
    black_upper: np.ndarray = field(
        default_factory=lambda: np.array([180, 80,  60]))

    # Yellow stripe (secondary ID for this raft model)
    yellow_lower: np.ndarray = field(
        default_factory=lambda: np.array([18, 100, 100]))
    yellow_upper: np.ndarray = field(
        default_factory=lambda: np.array([35, 255, 255]))

    # -----------------------------------------------------------------------
    # Morphology — as FRACTION of min(image_width, image_height).
    # Scales automatically from 640p webcam to 12MP drone images.
    # open_frac:  removes water glints and tiny speckles
    # close_frac: fills gaps inside partially-inflated raft body
    # -----------------------------------------------------------------------
    open_frac:  float = 0.005   # ~15px at 3024px, ~3px at 640px
    close_frac: float = 0.030   # ~91px at 3024px, ~19px at 640px

    # -----------------------------------------------------------------------
    # Contour area — as FRACTION of total frame pixel count.
    # Resolution-independent; replaces hard-coded min/max_area in v1.
    # -----------------------------------------------------------------------
    min_area_frac: float = 0.0015   # 0.15% of frame
    max_area_frac: float = 0.70     # 70%  of frame

    # Bounding box aspect ratio (width / height)
    min_aspect: float = 0.25
    max_aspect: float = 4.0

    # -----------------------------------------------------------------------
    # Confidence scoring (all metrics are resolution-independent ratios)
    # -----------------------------------------------------------------------
    w_fill:      float = 0.40
    w_convexity: float = 0.25
    w_black:     float = 0.20
    w_yellow:    float = 0.15

    score_threshold: float = 0.40

    # -----------------------------------------------------------------------
    # Temporal smoothing (video / live stream)
    # -----------------------------------------------------------------------
    temporal_window: int   = 5
    min_hit_ratio:   float = 0.4

    # -----------------------------------------------------------------------
    # Water mask gate: fraction of frame that must be blue/green before
    # the water suppression layer activates (prevents false suppression on
    # grey riverbanks or indoor test scenes)
    # -----------------------------------------------------------------------
    water_gate_frac: float = 0.05

    # -----------------------------------------------------------------------
    # Thermal detection
    # -----------------------------------------------------------------------
    thermal_min_temp_c: float = 30.0   # hotspot threshold above ambient
    thermal_min_area:   int   = 50     # px² minimum blob size
    thermal_max_area:   int   = 50_000


class RaftDetector:
    def __init__(self, cfg: Optional[RaftDetectorConfig] = None):
        self.cfg = cfg or RaftDetectorConfig()
        self._history: deque = deque(maxlen=self.cfg.temporal_window)
        self._kernel_cache: dict = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect_rgb(self, frame: np.ndarray) -> Optional[dict]:
        """
        Run orange-raft detection on a BGR frame.
        Returns dict with bbox, score, center, or None.
        """
        cfg = self.cfg
        h_img, w_img = frame.shape[:2]
        open_k, close_k = self._get_kernels(h_img, w_img)
        pixel_count = h_img * w_img

        # Adaptive brightness normalisation (CLAHE on V channel)
        enhanced = adaptive_preprocess(frame)
        blurred  = cv2.GaussianBlur(enhanced, (7, 7), 0)
        hsv      = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Orange mask (two ranges to cover shadows)
        orange_mask = (cv2.inRange(hsv, cfg.orange_lower1, cfg.orange_upper1) |
                       cv2.inRange(hsv, cfg.orange_lower2, cfg.orange_upper2))

        # Confirmation masks
        black_mask  = cv2.inRange(hsv, cfg.black_lower,  cfg.black_upper)
        yellow_mask = cv2.inRange(hsv, cfg.yellow_lower, cfg.yellow_upper)

        # Gated water suppression
        water_mask = _build_water_mask(hsv)
        if cv2.countNonZero(water_mask) / max(pixel_count, 1) >= cfg.water_gate_frac:
            orange_mask = cv2.bitwise_and(
                orange_mask, cv2.bitwise_not(water_mask))

        # Morphological cleanup
        orange_clean = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN,  open_k)
        orange_clean = cv2.morphologyEx(orange_clean, cv2.MORPH_CLOSE, close_k)

        contours, _ = cv2.findContours(
            orange_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = pixel_count * cfg.min_area_frac
        max_area = pixel_count * cfg.max_area_frac

        best = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (min_area <= area <= max_area):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / max(h, 1)
            if not (cfg.min_aspect <= aspect <= cfg.max_aspect):
                continue
            score = _score_rgb(cnt, x, y, w, h, area,
                               orange_mask, black_mask, yellow_mask, cfg)
            if score >= cfg.score_threshold:
                if best is None or score > best["score"]:
                    best = {
                        "bbox":   (x, y, w, h),
                        "score":  round(score, 3),
                        "center": (x + w // 2, y + h // 2),
                    }

        self._history.append(best)
        return best

    def detect_rgb_smoothed(self, frame: np.ndarray) -> Optional[dict]:
        """Temporally smoothed RGB detection — reduces video flicker."""
        self.detect_rgb(frame)
        cfg  = self.cfg
        hits = [h for h in self._history if h is not None]
        if len(hits) / max(len(self._history), 1) < cfg.min_hit_ratio or not hits:
            return None
        return {
            "bbox":   tuple(int(np.mean([h["bbox"][i]   for h in hits])) for i in range(4)),
            "score":  round(float(np.mean([h["score"]   for h in hits])), 3),
            "center": tuple(int(np.mean([h["center"][i] for h in hits])) for i in range(2)),
        }

    def detect_thermal(self, temp_celsius: np.ndarray) -> Optional[dict]:
        """
        Detect a warm blob (person on raft) in a float32 temperature array (°C).
        Returns dict with bbox, score, center, or None.
        """
        cfg = self.cfg
        mask = (temp_celsius > cfg.thermal_min_temp_c).astype(np.uint8) * 255
        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (cfg.thermal_min_area <= area <= cfg.thermal_max_area):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            roi      = temp_celsius[y:y+h, x:x+w]
            max_temp = float(np.max(roi))
            # Normalise score: 0 at threshold, 1 at threshold+20°C
            score = float(np.clip((max_temp - cfg.thermal_min_temp_c) / 20.0, 0.0, 1.0))
            if best is None or score > best["score"]:
                best = {
                    "bbox":   (x, y, w, h),
                    "score":  round(score, 3),
                    "center": (x + w // 2, y + h // 2),
                }
        return best

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _get_kernels(self, h_img: int, w_img: int):
        key = (h_img, w_img)
        if key not in self._kernel_cache:
            dim = min(h_img, w_img)
            ok  = max(3,  int(dim * self.cfg.open_frac)  | 1)
            ck  = max(11, int(dim * self.cfg.close_frac) | 1)
            self._kernel_cache[key] = (
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok, ok)),
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck)),
            )
        return self._kernel_cache[key]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def adaptive_preprocess(frame: np.ndarray) -> np.ndarray:
    """CLAHE brightness normalisation for varying lighting conditions."""
    hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def _build_water_mask(hsv: np.ndarray) -> np.ndarray:
    water = (cv2.inRange(hsv, np.array([85,  30,  30]), np.array([135, 255, 200])) |
             cv2.inRange(hsv, np.array([35,  30,  30]), np.array([85,  200, 180])) |
             cv2.inRange(hsv, np.array([0,    0,   0]), np.array([180,  30,  45])))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.dilate(water, k, iterations=1)


def _score_rgb(cnt, x, y, w, h, area,
               orange_mask, black_mask, yellow_mask, cfg) -> float:
    # Convexity — raft is roughly oval
    hull_a = cv2.contourArea(cv2.convexHull(cnt))
    s_conv = float(np.clip(area / max(hull_a, 1) / 0.80, 0.0, 1.0))

    # Orange fill in bounding box (~55% is typical for inflated raft)
    roi_o  = orange_mask[y:y+h, x:x+w]
    s_fill = float(np.clip(cv2.countNonZero(roi_o) / max(w * h, 1) / 0.55, 0.0, 1.0))

    # Black trim shell
    shell_thick = max(4, int(min(w, h) * 0.04))
    shell = np.zeros(orange_mask.shape, dtype=np.uint8)
    cv2.drawContours(shell, [cnt], -1, 255, thickness=shell_thick)
    shell_px = cv2.countNonZero(shell)
    black_in = cv2.countNonZero(cv2.bitwise_and(black_mask, shell))
    s_black  = float(np.clip(black_in / max(shell_px, 1) / 0.15, 0.0, 1.0))

    # Yellow stripe near contour
    dilated = cv2.dilate(shell, np.ones((5, 5), np.uint8), iterations=2)
    dil_px  = cv2.countNonZero(dilated)
    yel_in  = cv2.countNonZero(cv2.bitwise_and(yellow_mask, dilated))
    s_yellow = float(np.clip(yel_in / max(dil_px, 1) / 0.05, 0.0, 1.0))

    return float(np.clip(
        cfg.w_fill * s_fill + cfg.w_convexity * s_conv +
        cfg.w_black * s_black + cfg.w_yellow * s_yellow,
        0.0, 1.0))
