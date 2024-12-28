#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

# 相机内参
K = np.array([[477.278, 0, 320.033],
              [0, 477.278, 202.374],
              [0, 0, 1]])

class DepthToPointCloud:
    def __init__(self):
        rospy.init_node("depth_to_point_cloud")
        
        # 订阅深度图话题
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        
        # 发布点云话题
        self.point_cloud_pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
        
        # CvBridge 用于图像格式转换
        self.bridge = CvBridge()

    def depth_callback(self, depth_msg):
        try:
            # 使用 CvBridge 将 ROS 图像消息转换为 OpenCV 格式
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            # 验证深度图格式是否为 16UC1
            if depth_image.dtype != np.uint16:
                rospy.logerr("Expected depth image format is 16UC1, but received different format.")
                return

            # 将深度图从毫米 (mm) 转换为米 (m)
            depth_image = depth_image.astype(np.float32) / 5000.0  #单位为0.2mm

            # 转换深度图为点云
            points = self.depth_to_point_cloud(depth_image, K)

            # 发布点云
            self.publish_point_cloud(points, depth_msg.header)
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def depth_to_point_cloud(self, depth_image, K):
        """
        将深度图像转换为点云
        :param depth_image: 深度图 (numpy array, 单位为米)
        :param K: 相机内参矩阵
        :return: 点云数组 (N, 3)
        """
        h, w = depth_image.shape
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # 生成像素网格
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        u = u.astype(np.float32)
        v = v.astype(np.float32)

        # 筛选有效深度点
        valid = depth_image > 0
        z = depth_image[valid]
        x = (u[valid] - cx) * z / fx
        y = (v[valid] - cy) * z / fy

        # 将有效点组合为点云
        points = np.stack((x, y, z), axis=-1)
        return points

    def publish_point_cloud(self, points, header):
        """
        发布点云为 PointCloud2 格式
        :param points: 点云数组 (N, 3)
        :param header: 原始深度图的 ROS 消息头
        """
        # 定义 PointCloud2 消息的字段
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]

        # 更新消息头的 frame_id
        header.frame_id = "map"

        # 创建 PointCloud2 消息
        point_cloud_msg = pc2.create_cloud(header, fields, points)
        self.point_cloud_pub.publish(point_cloud_msg)

if __name__ == "__main__":
    try:
        node = DepthToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down depth to point cloud node.")
