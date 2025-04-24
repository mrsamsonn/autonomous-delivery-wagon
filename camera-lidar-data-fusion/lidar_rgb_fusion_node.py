#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import struct

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import PointField

class LiDARRGBFusion:
    def __init__(self):
        rospy.init_node('lidar_rgb_fusion_node')

        self.bridge = CvBridge()
        self.intrinsics_received = False

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.cam_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.cam_info_callback)
        self.lidar_sub = rospy.Subscriber("/points_raw", PointCloud2, self.lidar_callback)

        self.fused_pub = rospy.Publisher("/colored_pointcloud", PointCloud2, queue_size=1)

        self.latest_image = None
        self.K = None

    def cam_info_callback(self, msg):
        if not self.intrinsics_received:
            self.K = np.array(msg.K).reshape(3, 3)
            self.intrinsics_received = True
            rospy.loginfo("Camera intrinsics received.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")

    def lidar_callback(self, cloud_msg):
        if self.latest_image is None or not self.intrinsics_received:
            return

        height, width, _ = self.latest_image.shape
        colored_points = []

        for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            if z <= 0:  # behind camera
                continue

            # Project to 2D
            uv = self.K @ np.array([[x], [y], [z]])
            u, v = int(uv[0]/uv[2]), int(uv[1]/uv[2])

            if 0 <= u < width and 0 <= v < height:
                b, g, r = self.latest_image[v, u]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                colored_points.append([x, y, z, rgb])

        # Define new fields (xyz + rgb)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = cloud_msg.header.frame_id

        fused_msg = point_cloud2.create_cloud(header, fields, colored_points)
        self.fused_pub.publish(fused_msg)


if __name__ == "__main__":
    try:
        fusion = LiDARRGBFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
