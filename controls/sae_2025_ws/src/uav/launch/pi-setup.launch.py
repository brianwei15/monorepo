import glob
import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


_EXCLUDE_NAMES = ("PureThermal", "codec", "bcm2835", "isp", "unicam")


def find_rgb_camera() -> str:
    """
    Find the /dev/videoN path for the RGB camera by scanning sysfs.
    Excludes the PureThermal thermal board and Pi internal devices
    (codec, ISP, unicam). Returns the first device that reports MJPG
    or YUYV capture support.

    Raises RuntimeError if no suitable device is found.
    """
    import subprocess
    for sysfs_name in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        with open(sysfs_name) as f:
            name = f.read().strip()
        if any(x.lower() in name.lower() for x in _EXCLUDE_NAMES):
            continue
        dev = "/dev/" + os.path.basename(os.path.dirname(sysfs_name))
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", dev, "--list-formats"],
                capture_output=True, text=True, timeout=3,
            )
            formats = result.stdout
        except Exception:
            continue
        if "MJPG" in formats or "YUYV" in formats:
            return dev
    raise RuntimeError("No RGB camera device found.")


_TELEM_DEVICES = {
    "telem1": "/dev/ttyAMA1",
    "telem2": "/dev/serial0",
    "telem3": "/dev/ttyAMA2",
}


def launch_setup(context, *args, **kwargs):
    proxy_ip = LaunchConfiguration("proxy_ip").perform(context)
    udp_port = LaunchConfiguration("udp_port").perform(context)
    px4_id = LaunchConfiguration("px4_id").perform(context)
    thermal = LaunchConfiguration("thermal").perform(context)
    telem = LaunchConfiguration("telem").perform(context)
    ns = f"/px4_{px4_id}" if px4_id else ""

    use_camera = LaunchConfiguration("use_camera").perform(context)
    use_mavproxy = LaunchConfiguration("use_mavproxy").perform(context)
    use_xrce = LaunchConfiguration("use_xrce").perform(context)

    telem_dev = _TELEM_DEVICES.get(telem, telem)  # allow raw device path as fallback

    middleware = ExecuteProcess(
        cmd=["MicroXRCEAgent", "serial", "--dev", telem_dev, "-b", "921600"],
        output="screen",
        name="middleware",
    )

    maxproxy = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master=/dev/ttyACM0",
            "--baudrate",
            "115200",
            "--out",
            f"udp:{proxy_ip}:{udp_port}",
        ],
        output="screen",
        name="mavproxy",
    )

    v4l2_cam = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "v4l2_camera",
            "v4l2_camera_node",
            "--ros-args",
            "-p",
            "image_size:=[640,480]",
            "--remap",
            f"/image_raw:={ns}/camera",
            "--remap",
            f"/image_raw/compressed:={ns}/camera/compressed",
            "--remap",
            f"/camera_info:={ns}/camera_info",
        ],
        output="screen",
        name="v4l2_cam",
    )

    thermal_cam = ExecuteProcess(
        cmd=[
            "ros2", "run", "uav", "thermal_camera_node",
            "--ros-args",
            "-p", f"px4_id:=px4_{px4_id}",
            "-p", "threshold:=26.0",
            "-p", "min_area:=40",
        ],
        output="screen",
        name="thermal_cam",
    )

    rgb_device_arg = LaunchConfiguration("rgb_device").perform(context)
    if rgb_device_arg:
        rgb_device = rgb_device_arg
    else:
        try:
            rgb_device = find_rgb_camera()
            import logging
            logging.info(f"RGB camera autodetected: {rgb_device}")
        except RuntimeError as e:
            import logging
            logging.warning(f"RGB camera autodetect failed: {e} — no device found, rgb_cam will likely fail")
            rgb_device = ""

    rgb_cam = ExecuteProcess(
        cmd=[
            "ros2", "run", "v4l2_camera", "v4l2_camera_node",
            "--ros-args",
            "-p", f"video_device:={rgb_device}",
            "-p", "image_size:=[640,480]",
            "-p", "pixel_format:=YUYV",
            "-p", "framerate:=30",
            "--remap", f"/image_raw:={ns}/rgb/image_raw",
            "--remap", f"/image_raw/compressed:={ns}/rgb/image_raw/compressed",
            "--remap", f"/camera_info:={ns}/rgb/camera_info",
        ],
        output="screen",
        name="rgb_cam",
    )

    actions = []

    if use_xrce.lower() == "true":
        actions.append(middleware)
    if use_mavproxy.lower() == "true":
        actions.append(maxproxy)
    if use_camera.lower() == "true":
        if thermal.lower() == "true":
            actions += [thermal_cam, rgb_cam]
        else:
            actions.append(v4l2_cam)

    return actions


# MicroXRCEAgent serial --dev /dev/serial0 -b 921600
# mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out udp:10.42.0.1:14550
# mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --out udp:<LAPTOP_IP>:14550


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("proxy_ip", default_value="10.42.0.1"),
            DeclareLaunchArgument("udp_port", default_value="14550"),
            DeclareLaunchArgument("px4_id", default_value="1"),
            DeclareLaunchArgument("telem", default_value="telem2"),
            DeclareLaunchArgument("thermal", default_value="false"),
            DeclareLaunchArgument("rgb_device", default_value=""),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_mavproxy", default_value="true"),
            DeclareLaunchArgument("use_xrce", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
