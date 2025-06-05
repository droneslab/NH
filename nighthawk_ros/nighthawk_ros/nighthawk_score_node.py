#!/usr/bin/env python3
import sys, os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
from nighthawk_ros import score as scr

# --- ROS2 Node for Scoring ---
class NighthawkScoreNode(Node):
    def __init__(self):
        super().__init__("nighthawk_score_node")
        self.IMAGE_WIDTH = self.declare_parameter("IMAGE_WIDTH", 640).value
        self.IMAGE_HEIGHT = self.declare_parameter("IMAGE_HEIGHT", 400).value
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            SensorImage,
            "/flir_camera/image_raw",
            self.image_callback,
            10)
        # Publisher to publish the computed score
        self.score_pub = self.create_publisher(Float32, "nighthawk/score", 10)
        self.bridge = CvBridge()
        
        # Setup the R2D2 model for scoring
        self.R2D2_CHECKPOINT = self.declare_parameter("R2D2_CHECKPOINT", "/home/spottop/yt-nighthawk/r2d2/models/faster2d2_WASF_N16.pt").value
        self.R2D2_GPU_INDICES = self.declare_parameter("R2D2_GPU_INDICES", [0]).value
        self.R2D2_MODEL, self.R2D2_DEVICE = scr.setup_r2d2(checkpoint_path=self.R2D2_CHECKPOINT,
                                                        gpu_indices=self.R2D2_GPU_INDICES)
        self.get_logger().info("NighthawkScoreNode initialized and R2D2 model loaded.")

    def image_callback(self, msg: SensorImage):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
        
        # Compute the score using the R2D2 model
        score = scr.compute_score(cv_image, self.R2D2_MODEL, self.R2D2_DEVICE, resize=(self.IMAGE_WIDTH,self.IMAGE_HEIGHT))
        self.get_logger().info(f"Computed score: {score}")
        
        # Publish the score
        score_msg = Float32()
        score_msg.data = float(score)
        self.score_pub.publish(score_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NighthawkScoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()