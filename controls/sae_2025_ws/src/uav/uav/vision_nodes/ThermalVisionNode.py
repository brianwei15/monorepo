import numpy as np
from sensor_msgs.msg import Image
from uav.vision_nodes.VisionNode import VisionNode
import rclpy


class ThermalVisionNode(VisionNode):
    """
    Vision node that subscribes to /thermal_camera and processes
    16-bit thermal images (L16 format, values = temperature in Kelvin * 100).
    """

    def __init__(self):
        super().__init__(custom_service=None)

        self.thermal_image = None  # latest temperature array in Celsius (H x W float32)

        self.create_subscription(Image, "/thermal_camera", self.thermal_callback, 10)

    def thermal_callback(self, msg: Image):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        temp_celsius = raw / 100.0 - 273.15
        self.thermal_image = temp_celsius

        max_temp = float(np.max(temp_celsius))
        mean_temp = float(np.mean(temp_celsius))
        if self.debug:
            self.get_logger().info(
                f"Thermal frame: max={max_temp:.1f}°C  mean={mean_temp:.1f}°C"
            )

    def get_hotspot(self, threshold_celsius: float = 40.0):
        """
        Returns (cx, cy) pixel of the hottest region above threshold, or None.
        Call this from an autonomous mode or service callback.
        """
        if self.thermal_image is None:
            return None
        mask = self.thermal_image > threshold_celsius
        if not np.any(mask):
            return None
        ys, xs = np.where(mask)
        cx = int(np.mean(xs))
        cy = int(np.mean(ys))
        return cx, cy


def main():
    rclpy.init()
    node = ThermalVisionNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()
    rclpy.shutdown()
