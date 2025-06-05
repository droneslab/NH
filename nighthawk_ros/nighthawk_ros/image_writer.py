#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from multiprocessing import shared_memory

class ImageWriter(Node):
    def __init__(self):
        super().__init__('image_writer')

        self.SHARED_MEMORY_NAME = self.declare_parameter("SHARED_MEMORY_NAME", "shared_img").value
        self.IMAGE_SHAPE = tuple(self.declare_parameter("IMAGE_SHAPE", [1200, 1920, 3]).value)
        self.IMAGE_DTYPE = np.uint8  # Assuming uint8, as retrieved types are typically strings
        
        self.subscription = self.create_subscription( # TODO : change topic name to parameter
            Image,
            '/flir_camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Shared memory initialization
        self.shared_mem = None
        # self.IMAGE_SHAPE = (1200, 1920, 3)  # Adjust as needed
        # self.IMAGE_DTYPE = np.uint8
        # self.SHARED_MEMORY_NAME = "shared_img"

        # Create shared memory and write an empty image initially
        self.create_shared_memory()

    def create_shared_memory(self):
        """Initialize shared memory for the image."""
        image_size = np.prod(self.IMAGE_SHAPE) * np.dtype(self.IMAGE_DTYPE).itemsize
        try:
            self.shared_mem = shared_memory.SharedMemory(name=self.SHARED_MEMORY_NAME, create=True, size=image_size)
            self.get_logger().info("✅ Shared memory created successfully.")

            # Initialize shared memory with an empty image
            empty_image = np.zeros(self.IMAGE_SHAPE, dtype=self.IMAGE_DTYPE)
            np_array = np.ndarray(self.IMAGE_SHAPE, dtype=self.IMAGE_DTYPE, buffer=self.shared_mem.buf)
            np_array[:] = empty_image[:]

        except FileExistsError:
            self.shared_mem = shared_memory.SharedMemory(name=self.SHARED_MEMORY_NAME, create=False)
            self.get_logger().info("🔄 Attached to existing shared memory.")

    def image_callback(self, msg):
        """Callback function to process and store the latest image in shared memory."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to convert image: {e}')
            return

        # Ensure the image size matches the shared memory
        if cv_image.shape != self.IMAGE_SHAPE:
            self.get_logger().error(f"❌ Image shape mismatch! Expected {self.IMAGE_SHAPE}, but got {cv_image.shape}.")
            return

        # Write the image to shared memory
        np_array = np.ndarray(self.IMAGE_SHAPE, dtype=self.IMAGE_DTYPE, buffer=self.shared_mem.buf)
        np_array[:] = cv_image[:]

        self.get_logger().info("✅ Updated shared memory with the latest image.")

    def destroy_node(self):
        """Cleanup shared memory before shutting down."""
        if self.shared_mem is not None:
            self.shared_mem.close()
            self.shared_mem.unlink()
            self.get_logger().info("🗑️ Shared memory cleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = ImageWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🚀 Shutting down ImageWriter node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()