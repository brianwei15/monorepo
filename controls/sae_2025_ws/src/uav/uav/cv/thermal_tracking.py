"""
thermal_tracking.py

Detects and tracks heat sources from a FLIR Lepton 3.5 thermal camera feed.

The Lepton 3.5 outputs a 160x120 grayscale image where pixel intensity
corresponds to temperature (higher = hotter). This module thresholds
on intensity to isolate hot regions, then finds their centroids.

Intended to be used in two ways:
  1. Standalone tuning mode: run this file directly with a recorded video.
  2. As a library: import `find_heat_sources` into a ROS 2 Mode for live drone tracking.
"""

import cv2
import numpy as np
from typing import Optional, Tuple


def find_heat_sources(
    frame: np.ndarray,
    threshold: int = 140,
    min_area: int = 40,
    max_sources: int = 5,
    debug: bool = False,
) -> Tuple[Optional[Tuple[int, int]], list]:
    """
    Detect heat sources in a thermal grayscale frame.

    Args:
        frame:       Grayscale (or BGR) frame from the thermal camera.
                     If BGR, it is converted to grayscale automatically.
        threshold:   Pixel intensity cutoff (0-255). Pixels above this
                     value are considered "hot". Tune this with the video
                     tuning mode below.
        min_area:    Minimum contour area (pixels) to be considered a valid
                     heat source. Filters noise.
        max_sources: Maximum number of heat sources to return, ordered by
                     descending area (largest/hottest blob first).
        debug:       If True, draw contours and centers on the frame in-place
                     and add a text overlay with threshold info.

    Returns:
        primary_center: (cx, cy) pixel coords of the largest heat source,
                        or None if nothing is detected.
        all_centers:    List of (cx, cy) for all detected heat sources,
                        ordered by descending blob area.
    """
    # Normalize to grayscale
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame.copy()

    # Mask out the bottom 5 rows to ignore camera frame artifact
    gray[-5:, :] = 0

    # Threshold: keep only hot pixels
    _, hot_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

    # Clean up noise with morphological ops
    kernel = np.ones((3, 3), np.uint8)
    hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)
    hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter by area and compute centroids
    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        valid.append((area, c, (cx, cy)))

    # Sort by area descending, take top N
    valid.sort(key=lambda x: x[0], reverse=True)
    valid = valid[:max_sources]

    all_centers = [v[2] for v in valid]
    primary_center = all_centers[0] if all_centers else None

    if debug and len(frame.shape) == 3:
        for _, contour, center in valid:
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 1)
            cv2.circle(frame, center, 3, (0, 0, 255), -1)
        if primary_center is not None:
            cv2.circle(frame, primary_center, 6, (255, 0, 0), 2)
        cv2.putText(
            frame,
            f"thresh={threshold}  sources={len(all_centers)}",
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )

    return primary_center, all_centers


# ---------------------------------------------------------------------------
# Standalone tuning mode
# ---------------------------------------------------------------------------
# Live tuning:
#   python3 thermal_tracking.py path/to/video.mp4
#
# Render annotated output video (no display):
#   python3 thermal_tracking.py path/to/video.mp4 --output out.mp4 [--threshold 200]
#
# Live tuning controls:
#   [ / ]   decrease / increase threshold by 5
#   space   pause / resume
#   q       quit
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="Thermal heat-source tracker")
    parser.add_argument("video", help="Path to input video file")
    parser.add_argument("--output", "-o", default=None, help="Path to save annotated output video")
    parser.add_argument("--threshold", "-t", type=int, default=140, help="Initial intensity threshold (0-255, default 140)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Error: could not open video '{args.video}'")
        sys.exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    threshold = args.threshold

    # --- Output-only mode: render and save, no display ---
    if args.output:
        writer = cv2.VideoWriter(
            args.output,
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width * 2, height),  # side-by-side: annotated | hot mask (colorized)
        )
        print(f"Rendering to '{args.output}'  threshold={threshold}  ({total_frames} frames)")
        frame_idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            frame_idx += 1

            debug_frame = frame.copy()
            primary, centers = find_heat_sources(debug_frame, threshold=threshold, debug=True)

            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame.copy()
            _, hot_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
            hot_color = cv2.cvtColor(hot_mask, cv2.COLOR_GRAY2BGR)

            combined = np.hstack([debug_frame, hot_color])
            writer.write(combined)

            if primary:
                err_x = primary[0] - width // 2
                err_y = primary[1] - height // 2
                print(
                    f"  frame {frame_idx}/{total_frames}  primary={primary}  err=({err_x:+d}, {err_y:+d})  sources={len(centers)}",
                    end="\r",
                )
        cap.release()
        writer.release()
        print(f"\nDone. Saved to '{args.output}'")

    # --- Interactive tuning mode ---
    else:
        paused = False
        frame = None

        # Create a control window with trackbars for reliable parameter tuning.
        # Key events require window focus and are unreliable; trackbars are not.
        CTRL_WIN = "Controls (drag sliders to tune)"
        cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CTRL_WIN, 400, 100)
        cv2.createTrackbar("Threshold", CTRL_WIN, threshold, 254, lambda _: None)
        cv2.createTrackbar("Min Area", CTRL_WIN, 40, 500, lambda _: None)

        print("Drag sliders in the Controls window to tune.  q = quit  space = pause/resume")

        while True:
            threshold = cv2.getTrackbarPos("Threshold", CTRL_WIN)
            min_area  = cv2.getTrackbarPos("Min Area",  CTRL_WIN)
            # Avoid zero — threshold=0 would match everything
            threshold = max(1, threshold)
            min_area  = max(1, min_area)

            if not paused:
                ret, next_frame = cap.read()
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                frame = next_frame

            if frame is None:
                continue

            debug_frame = frame.copy()
            primary, centers = find_heat_sources(
                debug_frame, threshold=threshold, min_area=min_area, debug=True
            )

            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame.copy()
            _, hot_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

            cv2.imshow("Thermal Feed (debug)", debug_frame)
            cv2.imshow("Hot Mask", hot_mask)

            if primary:
                err_x = primary[0] - width // 2
                err_y = primary[1] - height // 2
                print(
                    f"  threshold={threshold:3d}  min_area={min_area:3d}  primary={primary}  "
                    f"err=({err_x:+d}, {err_y:+d})  sources={len(centers)}",
                    end="\r",
                )

            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                paused = not paused

        cap.release()
        cv2.destroyAllWindows()
