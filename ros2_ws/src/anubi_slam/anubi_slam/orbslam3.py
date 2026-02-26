"""Code for ORB-SLAM3 integration for the RealSense stereo camera d435i.

Authors: Lorenzo Ortolani
"""

from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from python_orb_slam3 import ORBExtractor
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class ExtractorRealSenseData(ORBExtractor):
    """ORB extractor for the RealSense stereo camera d435i."""

    def __init__(self, n_features=1000, scale_factor=1.2, n_levels=8,
                 ini_fast_threshold=20, min_fast_threshold=7, mask=None):
        """Initialize the ORB extractor."""
        super().__init__(n_features, scale_factor, n_levels,
                         ini_fast_threshold, min_fast_threshold)
        self.n_features = n_features
        self.scale_factor = scale_factor
        self.n_levels = n_levels
        self.ini_fast_threshold = ini_fast_threshold
        self.min_fast_threshold = min_fast_threshold
        self.mask = mask


class ORBExtractorD435i(ORBExtractor):
    """ORB extractor for the RealSense stereo camera d435i."""

    def __init__(self, n_features=1000, scale_factor=1.2, n_levels=8,
                 ini_fast_threshold=20, min_fast_threshold=7, mask=None):
        """Initialize the ORB extractor."""
        super().__init__(n_features, scale_factor, n_levels,
                         ini_fast_threshold, min_fast_threshold)
        self.n_features = n_features
        self.scale_factor = scale_factor
        self.n_levels = n_levels
        self.ini_fast_threshold = ini_fast_threshold
        self.min_fast_threshold = min_fast_threshold
        self.mask = mask


class PointCloud2ImageSubscriberNode(Node):
    """ROS2 node that subscribes to the colored PointCloud2 published by the C++ node.

    Reconstructs the organized RGB image from the packed float RGB field
    and exposes it for downstream processing.

    The C++ node packs RGB as::

        uint32_t bits = (R << 16) | (G << 8) | B;
        float rgb;
        memcpy(&rgb, &bits, sizeof(float));

    This class reverses that packing to recover an H x W x 3 uint8 BGR image
    ready for OpenCV / ORB-SLAM3.
    """

    def __init__(
        self,
        topic: str = '/camera/depth/color/points',
        on_image_callback=None,
    ):
        """Initialize the subscriber node.

        Parameters
        ----------
        topic:
            PointCloud2 topic published by the RealSense node.
        on_image_callback:
            Optional callable ``f(bgr_image, timestamp_sec)`` invoked each
            time a new frame arrives.
        """
        super().__init__('realsense_pointcloud_image_subscriber')
        self._on_image = on_image_callback
        self._sub = self.create_subscription(
            PointCloud2,
            topic,
            self._pointcloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.get_logger().info(f'Subscribed to {topic}')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _unpack_rgb_float(rgb_float_array: np.ndarray) -> np.ndarray:
        """Convert packed float RGB → uint8 [R, G, B] channels.

        Parameters
        ----------
        rgb_float_array:
            1-D float32 array of packed RGB values.

        Returns
        -------
        np.ndarray
            Shape (N, 3) uint8 array with columns [R, G, B].
        """
        bits = rgb_float_array.view(np.uint32)
        r = ((bits >> 16) & 0xFF).astype(np.uint8)
        g = ((bits >> 8) & 0xFF).astype(np.uint8)
        b = (bits & 0xFF).astype(np.uint8)
        return np.stack([r, g, b], axis=-1)

    def _pointcloud_callback(self, msg: PointCloud2):
        """Reconstruct the BGR image from the organized PointCloud2."""
        h, w = msg.height, msg.width
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Read only the 'rgb' field from every point
        rgb_gen = pc2.read_points(msg, field_names=('rgb',), skip_nans=False)
        rgb_floats = np.fromiter(
            (p[0] for p in rgb_gen), dtype=np.float32, count=h * w
        )

        # Unpack to (H, W, 3) RGB then convert to BGR for OpenCV
        rgb_image = self._unpack_rgb_float(rgb_floats).reshape(h, w, 3)
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        if self._on_image is not None:
            self._on_image(bgr_image, timestamp)


class ORBSLAM3StereoNode(Node):
    """ROS2 node that runs ORB-SLAM3 feature extraction on a stereo image pair.

    Subscribes to two synchronized ``sensor_msgs/Image`` topics (left and
    right cameras), extracts ORB keypoints and descriptors from each frame,
    and publishes the results via an optional callback.

    For a single-camera RGB-D setup (e.g. when using
    ``PointCloud2ImageSubscriberNode``) you can call
    :meth:`process_mono` directly instead.
    """

    def __init__(
        self,
        left_topic: str = '/camera/infra1/image_rect_raw',
        right_topic: str = '/camera/infra2/image_rect_raw',
        n_features: int = 1000,
        scale_factor: float = 1.2,
        n_levels: int = 8,
        ini_fast_threshold: int = 20,
        min_fast_threshold: int = 7,
        sync_slop: float = 0.05,
        on_features_callback=None,
    ):
        """Initialize the stereo ORB-SLAM3 node.

        Parameters
        ----------
        left_topic:
            Topic name for the left (or reference) camera image.
        right_topic:
            Topic name for the right camera image.
        n_features:
            Number of ORB features to extract per image.
        scale_factor:
            Pyramid scale factor.
        n_levels:
            Number of pyramid levels.
        ini_fast_threshold:
            Initial FAST corner detection threshold.
        min_fast_threshold:
            Minimum FAST threshold.
        sync_slop:
            Maximum allowed time difference (seconds) between left/right
            frames for approximate time synchronisation.
        on_features_callback:
            Optional callable invoked after each stereo frame::

                f(
                    kpts_left, desc_left,   # left keypoints / descriptors
                    kpts_right, desc_right, # right keypoints / descriptors
                    timestamp_sec,
                )
        """
        super().__init__('orbslam3_stereo_node')
        self._bridge = CvBridge()
        self._on_features = on_features_callback

        self._extractor_left = ORBExtractorD435i(
            n_features, scale_factor, n_levels,
            ini_fast_threshold, min_fast_threshold,
        )
        self._extractor_right = ORBExtractorD435i(
            n_features, scale_factor, n_levels,
            ini_fast_threshold, min_fast_threshold,
        )

        self._pub_left = self.create_publisher(Image, '/orbslam3/keypoints/left', 1)
        self._pub_right = self.create_publisher(Image, '/orbslam3/keypoints/right', 1)
        self._pub_mono = self.create_publisher(Image, '/orbslam3/keypoints/mono', 1)

        left_sub = Subscriber(self, Image, left_topic)
        right_sub = Subscriber(self, Image, right_topic)
        self._sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=sync_slop,
        )
        self._sync.registerCallback(self._stereo_callback)

        self.get_logger().info(
            f'ORB-SLAM3 stereo node ready — left: {left_topic}, right: {right_topic}'
        )

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def _publish_keypoints(self, bgr_image: np.ndarray, keypoints, publisher, header=None):
        """Draw keypoints on image and publish as sensor_msgs/Image."""
        vis = cv2.drawKeypoints(
            bgr_image, keypoints, None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        if header is not None:
            msg.header = header
        publisher.publish(msg)

    def process_mono(
        self, bgr_image: np.ndarray, timestamp: float
    ) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
        """Run ORB extraction on a single BGR image.

        Useful when feeding frames from ``PointCloud2ImageSubscriberNode``
        directly, without a second camera.

        Parameters
        ----------
        bgr_image:
            H x W x 3 uint8 BGR image.
        timestamp:
            Frame timestamp in seconds.

        Returns
        -------
        keypoints, descriptors
        """
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self._extractor_left.detectAndCompute(gray)
        self.get_logger().debug(
            f'[mono] t={timestamp:.3f}  kpts={len(keypoints)}'
        )
        self._publish_keypoints(bgr_image, keypoints, self._pub_mono)
        return keypoints, descriptors

    # ------------------------------------------------------------------
    # Internal callbacks
    # ------------------------------------------------------------------

    def _stereo_callback(self, left_msg: Image, right_msg: Image):
        """Process a synchronised stereo pair through ORB-SLAM3."""
        im_left = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        im_right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        gray_left = cv2.cvtColor(im_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(im_right, cv2.COLOR_BGR2GRAY)

        timestamp = (left_msg.header.stamp.sec
                     + left_msg.header.stamp.nanosec * 1e-9)

        kpts_left, desc_left = self._extractor_left.detectAndCompute(gray_left)
        kpts_right, desc_right = self._extractor_right.detectAndCompute(gray_right)

        self.get_logger().debug(
            f'[stereo] t={timestamp:.3f}  '
            f'kpts_left={len(kpts_left)}  kpts_right={len(kpts_right)}'
        )

        self._publish_keypoints(im_left, kpts_left, self._pub_left, left_msg.header)
        self._publish_keypoints(im_right, kpts_right, self._pub_right, right_msg.header)

        if self._on_features is not None:
            self._on_features(
                kpts_left, desc_left,
                kpts_right, desc_right,
                timestamp,
            )


def main(args=None):
    """Entry point: run the stereo ORB-SLAM3 node with PointCloud2 image source."""
    rclpy.init(args=args)

    stereo_node = ORBSLAM3StereoNode()

    # Wire the PointCloud2 subscriber so it feeds mono frames into the stereo node
    subscriber_node = PointCloud2ImageSubscriberNode(
        on_image_callback=lambda img, ts: stereo_node.process_mono(img, ts)
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(stereo_node)
    executor.add_node(subscriber_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        stereo_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()
