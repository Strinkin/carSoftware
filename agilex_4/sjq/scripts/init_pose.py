#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

def set_init_pose():
    rospy.init_node('init_pose_publisher')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    # 等待RViz和AMCL启动
    rospy.sleep(2.0)
    
    # 创建初始位姿消息
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = rospy.Time.now()
    
    # 设置初始位置 (x, y, z)
    init_pose.pose.pose.position.x = 0.0
    init_pose.pose.pose.position.y = 0.0
    init_pose.pose.pose.position.z = 0.0
    
    # 设置初始朝向 (四元数)
    init_pose.pose.pose.orientation.x = 0
    init_pose.pose.pose.orientation.y = 0
    init_pose.pose.pose.orientation.z = 0
    init_pose.pose.pose.orientation.w = 1
    
    # 设置协方差矩阵 (6x6矩阵，对应x, y, z, roll, pitch, yaw)
    # 索引顺序: 
    # 0: x方差    1: xy协方差  2: xz协方差  3: x_roll协方差  4: x_pitch协方差  5: x_yaw协方差
    # 6: yx协方差 7: y方差     8: yz协方差  9: y_roll协方差 10: y_pitch协方差 11: y_yaw协方差
    # 12: zx协方差13: zy协方差 14: z方差    15: z_roll协方差 16: z_pitch协方差 17: z_yaw协方差
    # ... 以此类推
    
    covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,   # x方差: 0.25m² (标准差约0.5m)
                  0.0, 0.25, 0.0, 0.0, 0.0, 0.0,   # y方差: 0.25m² (标准差约0.5m)
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # z方差: 0.0 (2D定位中z通常固定)
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # roll方差
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # pitch方差
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]  # yaw方差: 约15度的方差
    
    # 或者使用更简单的设置方式：
    # covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,   # x
    #               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,   # y
    #               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # z
    #               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # roll
    #               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # pitch
    #               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]  # yaw
    
    init_pose.pose.covariance = covariance
    
    # 发布初始位姿
    pub.publish(init_pose)
    rospy.loginfo("Initial pose published with covariance")
    
    # 清除AMCL的粒子云（可选）
    """rospy.wait_for_service('/global_localization')
    try:
        global_loc = rospy.ServiceProxy('/global_localization', Empty)
        global_loc()
        rospy.loginfo("Particle cloud cleared")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)"""

if __name__ == '__main__':
    try:
        set_init_pose()
    except rospy.ROSInterruptException:
        pass
