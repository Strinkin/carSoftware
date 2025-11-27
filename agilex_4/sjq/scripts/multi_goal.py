#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

def publish_poses_xyzw():
    # 初始化节点
    rospy.init_node('pose_publisher_xyzw', anonymous=True)
    
    # 创建发布者
    pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    rospy.sleep(1)  # 等待发布者建立连接
    
    # 定义位姿序列: [x, y, z, qx, qy, qz, qw]
    pose_sequence = [
        # 位置坐标(x,y,z) + 四元数(x,y,z,w)
        
        #[0.0, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707], # 位置2，绕Z轴旋转90度
        #[0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],     # 位置3，绕Z轴旋转180度
        #[0.0, 0.0, 0.0, 0.0, 0.0, -0.707,  0.707], # 位置3，绕Z轴旋转270度
	#[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],       # 位置1，无旋转
        [0.0, -0.3, 0.0, 0.0, 0.0, 0.0, 1.0],      # 位置1，无旋转
    ]
    
    rospy.loginfo("开始发布目标位姿序列 (XYZ + XYZW格式)...")
    
    for i, pose_data in enumerate(pose_sequence):
        if rospy.is_shutdown():
            break
            
        # 解包数据
        x, y, z, qx, qy, qz, qw = pose_data
        
        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        
        # 设置header
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        # 设置位置 (XYZ)
        pose_msg.pose.position = Point(x, y, z)
        
        # 设置方向 (XYZW四元数)
        pose_msg.pose.orientation = Quaternion(qx, qy, qz, qw)
        
        # 发布消息
        pose_pub.publish(pose_msg)
        rospy.loginfo("发布第 %d 个位姿: Pos(%.2f,%.2f,%.2f) Quat(%.2f,%.2f,%.2f,%.2f)", 
                     i+1, x, y, z, qx, qy, qz, qw)
        
        # 等待一段时间
        rospy.sleep(10.0)
    
    rospy.loginfo("所有位姿发布完成！")

if __name__ == '__main__':
    try:
        publish_poses_xyzw()
    except rospy.ROSInterruptException:
        pass
