"""
thermal_tracking.py

Detects and tracks heat sources from a FLIR Lepton 3.5 thermal camera
via a GroupGets PureThermal board.

Two operating modes:
  - Radiometric (uint16 frame): PureThermal streams Y16 format.
    Each pixel is temperature in centikelvins. threshold is in Celsius.
    Immune to AGC rescaling — a fixed Celsius threshold always means
    the same physical temperature.
  - Legacy 8-bit (uint8/BGR frame): for tuning against pre-recorded videos.
    threshold is a 0-255 pixel intensity value.

Mode is auto-detected from frame dtype — no configuration needed.

Usage:
  Video file tuning:
    python3 thermal_tracking.py path/to/video.mp4

  Render annotated output video:
    python3 thermal_tracking.py path/to/video.mp4 --output out.mp4

  Live camera feed (PureThermal via USB):
    python3 thermal_tracking.py --live
"""

import glob
import os
import cv2
import numpy as np
from typing import Optional, Tuple


def find_purethermal_device() -> str:
    """
    Find the /dev/videoN path for the PureThermal board by reading
    kernel sysfs device names. This is stable regardless of enumeration
    order — the name is tied to the USB device identity, not boot order.

    Returns:
        Device path string, e.g. '/dev/video0'

    Raises:
        RuntimeError if no PureThermal device is found.
    """
    for sysfs_name in glob.glob("/sys/class/video4linux/video*/name"):
        with open(sysfs_name) as f:
            name = f.read().strip()
        if "PureThermal" not in name:
            continue
        dev = "/dev/" + os.path.basename(os.path.dirname(sysfs_name))
        # The image stream node supports Y16; the metadata node has no formats.
        # Check by trying to enumerate formats via v4l2-ctl.
        result = os.popen(f"v4l2-ctl -d {dev} --list-formats 2>/dev/null").read()
        if "Y16" in result:
            return dev
    raise RuntimeError(
        "PureThermal device not found. Is it plugged in?"
    )


def open_purethermal_capture() -> cv2.VideoCapture:
    """
    Open the PureThermal camera in Y16 radiometric mode.

    Returns:
        An opened cv2.VideoCapture configured for 16-bit radiometric output.
    """
    device = find_purethermal_device()
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', '1', '6', ' '))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open PureThermal device at {device}")
    return cap


def find_heat_sources(
    frame: np.ndarray,
    threshold: float = 33.0,
    min_area: int = 40,
    max_sources: int = 5,
    debug: bool = False,
) -> Tuple[Optional[Tuple[int, int]], list, Optional[np.ndarray]]:
    """
    Detect heat sources in a thermal frame.

    Automatically detects operating mode from frame dtype:
      - uint16: radiometric Y16 from PureThermal. threshold is in Celsius.
                Strips the 2 telemetry rows at the bottom of 160x122 frames.
      - uint8 / BGR: legacy 8-bit AGC-scaled frame. threshold is 0-255 intensity.

    Args:
        frame:       Frame from thermal camera or video file.
        threshold:   Celsius cutoff in radiometric mode (default 33.0°C,
                     safely above typical outdoor background and below human
                     body temp of ~37°C). Pixel intensity cutoff in 8-bit mode.
        min_area:    Minimum contour area in pixels to filter noise.
        max_sources: Max heat sources to return, ordered by descending area.
        debug:       If True, returns an annotated BGR visualization as the
                     third return value.

    Returns:
        primary_center: (cx, cy) of the largest heat source, or None.
        all_centers:    List of (cx, cy) for all sources, largest first.
        vis_frame:      Annotated BGR image if debug=True, else None.
    """
    radiometric = frame.dtype == np.uint16

    if radiometric:
        # Strip telemetry rows. Lepton 3.5: 160x122 → 160x120 (2 rows).
        # Lepton 2: 80x63 → 80x60 (3 rows). Detect by raw frame height.
        img_height = {122: 120, 63: 60}.get(frame.shape[0], frame.shape[0])
        data = frame[:img_height, :]
        celsius = data / 100.0 - 273.15
        hot_mask = (celsius > threshold).astype(np.uint8) * 255
        if debug:
            norm = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            vis = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
    else:
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame.copy()
        # Mask bottom 5 rows to suppress camera frame artifact
        gray[-5:, :] = 0
        _, hot_mask = cv2.threshold(gray, int(threshold), 255, cv2.THRESH_BINARY)
        if debug:
            vis = frame if len(frame.shape) == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # Clean up noise
    kernel = np.ones((3, 3), np.uint8)
    hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)
    hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

    valid.sort(key=lambda x: x[0], reverse=True)
    valid = valid[:max_sources]

    all_centers = [v[2] for v in valid]
    primary_center = all_centers[0] if all_centers else None

    vis_frame = None
    if debug:
        for _, contour, center in valid:
            cv2.drawContours(vis, [contour], -1, (0, 255, 0), 1)
            cv2.circle(vis, center, 3, (0, 0, 255), -1)
        if primary_center is not None:
            cv2.circle(vis, primary_center, 6, (255, 0, 0), 2)
        label = (
            f"thresh={threshold:.1f}C (radiometric)"
            if radiometric
            else f"thresh={int(threshold)} (8-bit)"
        )
        cv2.putText(
            vis,
            f"{label}  sources={len(all_centers)}",
            (5, 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )
        vis_frame = vis

    return primary_center, all_centers, vis_frame


# ---------------------------------------------------------------------------
# Standalone modes
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    import argparse

    parser = argparse.ArgumentParser(description="Thermal heat-source tracker")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("video", nargs="?", help="Path to input video file")
    group.add_argument("--live", action="store_true", help="Stream live from PureThermal camera")
    parser.add_argument("--output", "-o", default=None, help="Path to save annotated output video (video file mode only)")
    parser.add_argument("--threshold", "-t", type=float, default=33.0,
                        help="Threshold: Celsius in live/radiometric mode (default 33.0), "
                             "intensity 0-255 in video file mode")
    args = parser.parse_args()

    # --- Live camera mode ---
    if args.live:
        try:
            device = find_purethermal_device()
            print(f"Found PureThermal at {device}")
            cap = open_purethermal_capture()
        except RuntimeError as e:
            print(e)
            sys.exit(1)

        threshold = args.threshold
        CTRL_WIN = "Controls"
        cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CTRL_WIN, 400, 80)
        # Slider range 0–60°C, stored as int (actual value used directly as Celsius)
        cv2.createTrackbar("Threshold C", CTRL_WIN, int(threshold), 60, lambda _: None)
        cv2.createTrackbar("Min Area",    CTRL_WIN, 40, 500,         lambda _: None)

        print("Drag sliders to tune.  q = quit")

        while True:
            threshold = float(cv2.getTrackbarPos("Threshold C", CTRL_WIN))
            min_area  = max(1, cv2.getTrackbarPos("Min Area", CTRL_WIN))

            ret, frame = cap.read()
            if not ret:
                continue

            primary, centers, vis = find_heat_sources(
                frame, threshold=threshold, min_area=min_area, debug=True
            )

            # Also show the raw hot mask
            img_height = {122: 120, 63: 60}.get(frame.shape[0], frame.shape[0])
            data = frame[:img_height, :]
            celsius = data / 100.0 - 273.15
            hot_mask = ((celsius > threshold).astype(np.uint8) * 255)

            cv2.imshow("Thermal Feed (debug)", vis)
            cv2.imshow("Hot Mask", hot_mask)

            if primary:
                h, w = vis.shape[:2]
                err_x = primary[0] - w // 2
                err_y = primary[1] - h // 2
                print(
                    f"  thresh={threshold:.1f}°C  primary={primary}  "
                    f"err=({err_x:+d}, {err_y:+d})  sources={len(centers)}",
                    end="\r",
                )

            if cv2.waitKey(30) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    # --- Video file mode (8-bit, for tuning against recorded footage) ---
    else:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            print(f"Error: could not open video '{args.video}'")
            sys.exit(1)

        fps          = cap.get(cv2.CAP_PROP_FPS) or 30.0
        width        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height       = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        threshold    = args.threshold

        # --- Render to output file ---
        if args.output:
            writer = cv2.VideoWriter(
                args.output,
                cv2.VideoWriter_fourcc(*"mp4v"),
                fps,
                (width * 2, height),
            )
            print(f"Rendering to '{args.output}'  threshold={threshold}  ({total_frames} frames)")
            frame_idx = 0
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                frame_idx += 1

                debug_frame = frame.copy()
                primary, centers, vis = find_heat_sources(
                    debug_frame, threshold=threshold, debug=True
                )

                if len(frame.shape) == 3:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    gray = frame.copy()
                _, hot_mask = cv2.threshold(gray, int(threshold), 255, cv2.THRESH_BINARY)
                hot_color = cv2.cvtColor(hot_mask, cv2.COLOR_GRAY2BGR)

                combined = np.hstack([vis, hot_color])
                writer.write(combined)

                if primary:
                    err_x = primary[0] - width // 2
                    err_y = primary[1] - height // 2
                    print(
                        f"  frame {frame_idx}/{total_frames}  primary={primary}"
                        f"  err=({err_x:+d}, {err_y:+d})  sources={len(centers)}",
                        end="\r",
                    )
            cap.release()
            writer.release()
            print(f"\nDone. Saved to '{args.output}'")

        # --- Interactive tuning ---
        else:
            paused = False
            frame  = None

            CTRL_WIN = "Controls (drag sliders to tune)"
            cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(CTRL_WIN, 400, 80)
            cv2.createTrackbar("Threshold", CTRL_WIN, int(threshold), 254, lambda _: None)
            cv2.createTrackbar("Min Area",  CTRL_WIN, 40,             500, lambda _: None)

            print("Drag sliders to tune.  space = pause/resume  q = quit")

            while True:
                threshold = float(max(1, cv2.getTrackbarPos("Threshold", CTRL_WIN)))
                min_area  = max(1, cv2.getTrackbarPos("Min Area", CTRL_WIN))

                if not paused:
                    ret, next_frame = cap.read()
                    if not ret:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    frame = next_frame

                if frame is None:
                    continue

                debug_frame = frame.copy()
                primary, centers, vis = find_heat_sources(
                    debug_frame, threshold=threshold, min_area=min_area, debug=True
                )

                if len(frame.shape) == 3:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    gray = frame.copy()
                _, hot_mask = cv2.threshold(gray, int(threshold), 255, cv2.THRESH_BINARY)

                cv2.imshow("Thermal Feed (debug)", vis)
                cv2.imshow("Hot Mask", hot_mask)

                if primary:
                    err_x = primary[0] - width // 2
                    err_y = primary[1] - height // 2
                    print(
                        f"  threshold={int(threshold):3d}  min_area={min_area:3d}"
                        f"  primary={primary}  err=({err_x:+d}, {err_y:+d})"
                        f"  sources={len(centers)}",
                        end="\r",
                    )

                key = cv2.waitKey(30) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord(" "):
                    paused = not paused

            cap.release()
            cv2.destroyAllWindows()
