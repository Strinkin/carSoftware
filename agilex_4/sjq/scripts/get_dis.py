#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math

# 全局变量存储位置
robot_pose = None
nav_goal = None

def pose_callback(msg):
    global robot_pose
    print("amcl")
    robot_pose = msg.pose.pose

def goal_callback(msg):
    global nav_goal
    nav_goal = msg.pose

def quaternion_to_yaw(q):
    """通过四元数计算 yaw"""
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def compute_distance(p1, p2):
    return math.sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)

def main():
    global robot_pose, nav_goal
    rospy.init_node('pose_distance_node')

    # 订阅机器人位置和目标点
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    rate = rospy.Rate(10)  # 2 Hz

    while not rospy.is_shutdown():
        if robot_pose and nav_goal:
            # 机器人坐标
            rx = robot_pose.position.x
            ry = robot_pose.position.y
            ryaw = quaternion_to_yaw(robot_pose.orientation)

            # 目标点坐标
            gx = nav_goal.position.x
            gy = nav_goal.position.y
            gyaw = quaternion_to_yaw(nav_goal.orientation)

            # 欧式距离
            distance = compute_distance(robot_pose, nav_goal)

            # 打印
            print("Robot: x={:.2f}, y={:.2f}, yaw={:.2f}".format(rx, ry, ryaw))
            print("Goal : x={:.2f}, y={:.2f}, yaw={:.2f}".format(gx, gy, gyaw))
            print("Distance: {:.2f}".format(distance))
            print("---")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


