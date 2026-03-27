from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    proxy_ip = LaunchConfiguration("proxy_ip").perform(context)
    udp_port = LaunchConfiguration("udp_port").perform(context)
    px4_id = LaunchConfiguration("px4_id").perform(context)
    ns = f"/px4_{px4_id}" if px4_id else ""

    middleware = ExecuteProcess(
        cmd=["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
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

    actions = [
        middleware,
        maxproxy,
        v4l2_cam,
    ]

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
            OpaqueFunction(function=launch_setup),
        ]
    )
