import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2

from uav.cv.thermal_tracking import open_purethermal_capture, find_heat_sources


class ThermalCameraNode(Node):
    """
    Captures frames from the FLIR Lepton via PureThermal board in Y16
    radiometric mode and publishes them to /thermal_camera as a 16UC1 Image.

    Each pixel value is temperature in centikelvins (divide by 100 - 273.15
    to get Celsius). Telemetry rows at the bottom of the frame are stripped,
    so published frames contain only image data (no metadata rows).

    Lepton 2 (80x60):  raw frame is 80x63, 3 telemetry rows stripped.
    Lepton 3.5 (160x120): raw frame is 160x122, 2 telemetry rows stripped.
    """

    # Maps raw frame height → (image_height, image_width)
    FRAME_DIMS = {
        63:  (60,  80),
        122: (120, 160),
    }

    def __init__(self):
        super().__init__("thermal_camera_node")

        self.declare_parameter("px4_id", "")
        self.declare_parameter("topic", "/thermal_camera")
        self.declare_parameter("fps", 9.0)
        self.declare_parameter("threshold", 33.0)
        self.declare_parameter("min_area", 40)

        px4_id = self.get_parameter("px4_id").value
        prefix = f"/{px4_id}" if px4_id else ""
        topic = prefix + self.get_parameter("topic").value
        fps = self.get_parameter("fps").value

        self.publisher = self.create_publisher(Image, topic, 10)
        self.heatmap_publisher = self.create_publisher(Image, topic + "/heatmap", 10)
        self.mask_publisher = self.create_publisher(Image, topic + "/mask", 10)
        self.debug_publisher = self.create_publisher(Image, topic + "/debug", 10)
        self.heatmap_compressed_publisher = self.create_publisher(CompressedImage, topic + "/heatmap/compressed", 10)
        self.mask_compressed_publisher = self.create_publisher(CompressedImage, topic + "/mask/compressed", 10)
        self.debug_compressed_publisher = self.create_publisher(CompressedImage, topic + "/debug/compressed", 10)
        self.img_height = None
        self.img_width = None

        try:
            self.cap = open_purethermal_capture()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            raise

        # Detect image dimensions from the first frame
        self._detect_dims()

        self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(
            f"ThermalCameraNode started — {self.img_width}x{self.img_height}, "
            f"publishing to {topic}"
        )

    def _detect_dims(self):
        for _ in range(10):
            ret, frame = self.cap.read()
            if ret:
                dims = self.FRAME_DIMS.get(frame.shape[0])
                if dims:
                    self.img_height, self.img_width = dims
                    return
        raise RuntimeError(
            f"Could not detect frame dimensions from PureThermal. "
            f"Check camera connection."
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from PureThermal")
            return

        # Strip telemetry rows — keep only image rows
        image_data = frame[:self.img_height, :]
        self.get_logger().info(f"Frame dtype: {frame.dtype}, shape: {frame.shape}", once=True)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "thermal_camera"
        msg.height = self.img_height
        msg.width = self.img_width
        msg.encoding = "16UC1"
        msg.is_bigendian = False
        msg.step = self.img_width * 2  # 2 bytes per pixel
        msg.data = image_data.astype(np.uint16).tobytes()

        self.publisher.publish(msg)

        # Heatmap: normalize to 0-255 and apply colormap
        norm = cv2.normalize(image_data, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        heatmap = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        heatmap_msg = Image()
        heatmap_msg.header = msg.header
        heatmap_msg.height = self.img_height
        heatmap_msg.width = self.img_width
        heatmap_msg.encoding = "bgr8"
        heatmap_msg.is_bigendian = False
        heatmap_msg.step = self.img_width * 3
        heatmap_msg.data = heatmap.tobytes()
        self.heatmap_publisher.publish(heatmap_msg)

        _, heatmap_encoded = cv2.imencode(".jpg", heatmap)
        heatmap_compressed_msg = CompressedImage()
        heatmap_compressed_msg.header = msg.header
        heatmap_compressed_msg.format = "jpeg"
        heatmap_compressed_msg.data = heatmap_encoded.tobytes()
        self.heatmap_compressed_publisher.publish(heatmap_compressed_msg)

        # Detection mask — run find_heat_sources and use its internal mask
        threshold = self.get_parameter("threshold").value
        min_area = self.get_parameter("min_area").value
        celsius = image_data / 100.0 - 273.15
        hot_mask = ((celsius > threshold).astype(np.uint8) * 255)
        kernel = np.ones((3, 3), np.uint8)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, kernel)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_CLOSE, kernel)

        mask_msg = Image()
        mask_msg.header = msg.header
        mask_msg.height = self.img_height
        mask_msg.width = self.img_width
        mask_msg.encoding = "mono8"
        mask_msg.is_bigendian = False
        mask_msg.step = self.img_width
        mask_msg.data = hot_mask.tobytes()
        self.mask_publisher.publish(mask_msg)

        _, mask_encoded = cv2.imencode(".png", hot_mask)
        mask_compressed_msg = CompressedImage()
        mask_compressed_msg.header = msg.header
        mask_compressed_msg.format = "png"
        mask_compressed_msg.data = mask_encoded.tobytes()
        self.mask_compressed_publisher.publish(mask_compressed_msg)

        # Debug: annotated BGR frame with contours and heat source centers
        _, _, debug_vis = find_heat_sources(
            image_data, threshold=threshold, min_area=min_area, debug=True
        )
        if debug_vis is not None:
            debug_msg = Image()
            debug_msg.header = msg.header
            debug_msg.height = self.img_height
            debug_msg.width = self.img_width
            debug_msg.encoding = "bgr8"
            debug_msg.is_bigendian = False
            debug_msg.step = self.img_width * 3
            debug_msg.data = debug_vis.tobytes()
            self.debug_publisher.publish(debug_msg)

            _, debug_encoded = cv2.imencode(".jpg", debug_vis)
            debug_compressed_msg = CompressedImage()
            debug_compressed_msg.header = msg.header
            debug_compressed_msg.format = "jpeg"
            debug_compressed_msg.data = debug_encoded.tobytes()
            self.debug_compressed_publisher.publish(debug_compressed_msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = ThermalCameraNode()
    except RuntimeError:
        rclpy.shutdown()
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
