#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from builtin_interfaces.msg import Time as TimeMsg


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, 'camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera/color/camera_info', self.cam_info_callback, 10)

        self.color_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

       
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pub = self.create_publisher(PoseStamped, '/detected_cup_pose', 10)

        
        self.img_pub = self.create_publisher(Image, '/detected_cup_image', 10)

        self.create_timer(0.1, self.detect_red_cup)

    def cam_info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def image_callback(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32) * 0.001
        self.depth_image = depth

    def get_3d_point(self, u, v):
        if self.depth_image is None or self.fx is None:
            return None
        depth_val = self.depth_image[v, u]
        if depth_val <= 0 or np.isnan(depth_val) or np.isinf(depth_val):
            return None
        x = (u - self.cx) * depth_val / self.fx
        y = (v - self.cy) * depth_val / self.fy
        z = depth_val
        return x, y, z

    def publish_tf(self, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'detected_cup'   
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0        
        self.tf_broadcaster.sendTransform(t)

    def detect_red_cup(self):
        if self.color_image is None or self.depth_image is None or self.fx is None:
            return

        vis = self.color_image.copy()

        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0 or cv2.contourArea(max(contours, key=cv2.contourArea)) < 100:
          
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            return

        largest = max(contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(largest)

        u = int(bx)
        v = int(by + bh / 2)

        point_3d = self.get_3d_point(u, v)
        if point_3d is None:
            self.get_logger().warn('invalid depth at cup center pixel')
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            return

        px, py, pz = point_3d

      
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.circle(vis, (u, v), 5, (0, 0, 255), -1)
        


        self.img_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))

        
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = 'camera_rgb_world'
        pose_cam.header.stamp = TimeMsg(sec=0, nanosec=0)
        pose_cam.pose.position.x = float(px)
        pose_cam.pose.position.y = float(py)
        pose_cam.pose.position.z = float(pz)
        pose_cam.pose.orientation.w = 1.0

        try:
            pose_base = self.tf_buffer.transform(
                pose_cam, 'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
           
            self.pub.publish(pose_base)

            
            self.publish_tf(
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z
            )

            self.get_logger().info(
                f'cup wrt base_link: x={pose_base.pose.position.x:.3f} 'f'y={pose_base.pose.position.y:.3f} 'f'z={pose_base.pose.position.z:.3f}')
        except Exception as e:
            self.get_logger().error(f'TF failed: {e}')


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()