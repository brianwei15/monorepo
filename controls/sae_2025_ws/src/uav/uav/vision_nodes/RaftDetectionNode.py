import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from uav.vision_nodes.VisionNode import VisionNode
from uav.cv.raft_detection import RaftDetector
import rclpy


class RaftDetectionNode(VisionNode):
    """
    Publishes raft detections from RGB and thermal cameras as PointStamped topics.

    Topics published:
      /raft_detection/rgb      — x,y = pixel centre; z = confidence (0-1); z=-1 if not detected
      /raft_detection/thermal  — same convention; z = normalised temperature score

    Topics subscribed (in addition to VisionNode's /camera, /camera_info):
      /thermal_camera          — sensor_msgs/Image, L16 encoding (K * 100)
    """

    def __init__(self):
        super().__init__(None)

        self._detector = RaftDetector()

        self._thermal_image: np.ndarray | None = None

        self.create_subscription(Image, "/thermal_camera", self._thermal_callback, 10)

        self._rgb_pub     = self.create_publisher(PointStamped, "/raft_detection/rgb",     10)
        self._thermal_pub = self.create_publisher(PointStamped, "/raft_detection/thermal", 10)

        self.create_timer(0.1, self._update)  # 10 Hz

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _thermal_callback(self, msg: Image):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self._thermal_image = raw / 100.0 - 273.15

    # ------------------------------------------------------------------
    # Timer
    # ------------------------------------------------------------------

    def _update(self):
        self._run_rgb()
        self._run_thermal()

    def _run_rgb(self):
        if self.image is None:
            return

        frame = self.convert_image_msg_to_frame(self.image)
        det = self._detector.detect_rgb_smoothed(frame)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        if det is not None:
            cx, cy = det["center"]
            msg.point.x = float(cx)
            msg.point.y = float(cy)
            msg.point.z = float(det["score"])
            if self.debug:
                self.get_logger().info(
                    f"RGB raft detected: centre=({cx},{cy}) score={det['score']:.2f}"
                )
        else:
            msg.point.x = -1.0
            msg.point.y = -1.0
            msg.point.z = -1.0
        self._rgb_pub.publish(msg)

    def _run_thermal(self):
        if self._thermal_image is None:
            return

        det = self._detector.detect_thermal(self._thermal_image)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "thermal_camera"
        if det is not None:
            cx, cy = det["center"]
            msg.point.x = float(cx)
            msg.point.y = float(cy)
            msg.point.z = float(det["score"])
            if self.debug:
                self.get_logger().info(
                    f"Thermal raft detected: centre=({cx},{cy}) score={det['score']:.2f}"
                )
        else:
            msg.point.x = -1.0
            msg.point.y = -1.0
            msg.point.z = -1.0
        self._thermal_pub.publish(msg)


def main():
    rclpy.init()
    node = RaftDetectionNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()
    rclpy.shutdown()
