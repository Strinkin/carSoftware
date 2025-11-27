#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Header
import math

# =====================================
#   全局变量
# =====================================
received_values = []      # 保存收到的不同值
TARGET_COUNT = 2          # 需要两个不同值
collected_flag = False    # 是否收集完成标志

current_robot_pose = None  # AMCL 实时位置


# =====================================
#   YOLO 回调函数
# =====================================
def yolo_callback(msg):
    global received_values, collected_flag
    v = msg.data

    if v not in received_values:
        received_values.append(v)
        rospy.loginfo("收到新值: %s (当前进度: %d/%d)" %
                       (v, len(received_values), TARGET_COUNT))

    if len(received_values) >= TARGET_COUNT:
        collected_flag = True


# =====================================
#   AMCL 回调函数：更新机器人位置
# =====================================
def amcl_callback(msg):
    global current_robot_pose
    print("amcl callback")
    current_robot_pose = msg.pose.pose


# =====================================
#   发布路径点（增加距离检测 + 超时跳点）
# =====================================
def publish_pose_sequence(pose_pub, pose_sequence,
                          arrive_threshold=0.30,
                          timeout_sec=20.0):
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    #rospy.Subscriber("/amcl_pose", PoseStamped, amcl_callback)

    rate = rospy.Rate(5)  # 5 Hz 检查距离

    for idx, pose_data in enumerate(pose_sequence):

        x_goal, y_goal, z_goal, qx, qy, qz, qw = pose_data

        # =============================
        # 发布目标点
        # =============================
        pose_msg = PoseStamped()
        pose_msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
        pose_msg.pose.position = Point(x_goal, y_goal, z_goal)
        pose_msg.pose.orientation = Quaternion(qx, qy, qz, qw)

        pose_pub.publish(pose_msg)
        rospy.loginfo("发布第 %d 个目标点: (%.2f, %.2f, %.2f, %.2f)" %
                      (idx + 1, x_goal, y_goal, qz, qw))

        start_time = rospy.Time.now()

        # =============================
        # 等待进入目标点（距离检测 + 超时）
        # =============================
        while not rospy.is_shutdown():

            # ------ 超时检测 ------
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout_sec:
                rospy.logwarn("第 %d 个目标点超时（%.1f 秒），跳过。" %
                              (idx + 1, timeout_sec))
                break

            # ------ AMCL 尚未就绪 ------
            if current_robot_pose is None:
                rate.sleep()
                continue

            # ------ 计算欧式距离 ------
            dx = current_robot_pose.position.x - x_goal
            dy = current_robot_pose.position.y - y_goal
            dist = (dx * dx + dy * dy) ** 0.5

            rospy.loginfo_throttle(1.0, "距离目标点 %.2fm (阈值 %.2fm)" %
                                   (dist, arrive_threshold))

            # ------ 到达判定 ------
            if dist < arrive_threshold:
                rospy.loginfo("到达目标点 %d（距离 %.2fm）" %
                              (idx + 1, dist))
                break

            rate.sleep()

    rospy.loginfo("所有路径点处理完毕。")


# =====================================
#   ROS 主节点
# =====================================
def collector_node():
    global collected_flag, received_values

    rospy.init_node("read_yolo_res", anonymous=True)

    rospy.Subscriber("/yolo_result", Int32, yolo_callback, queue_size=10)
    pose_pub = rospy.Publisher('/move_base_simple/goal',
                               PoseStamped, queue_size=10)

    rospy.loginfo("等待来自 YOLO 的两个不同值...")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not collected_flag:
        rate.sleep()

    rospy.loginfo("已收到两个值: %s" % received_values)

    # =====================================
    #             定义路径
    # =====================================
    go_A = [
        [1.5710, 2.1666, 0.0, 0, 0, 0.0139, 1.0],
        [5.0516, 0.8779, 0.0, 0, 0, -0.7098, 0.7043]
    ]
    back_A = [
        [4.9944, 0.8647, 0.0, 0, 0, 0.9322, 0.3619],
        [1.6919, 2.2727, 0.0, 0, 0, 0.9950, -0.0996],
        [0.0, 0.0, 0.0, 0, 0, 0, 1]
    ]

    go_B = [
        [2.8411, 0.4473, 0.0, 0, 0, -0.1811, 0.9834],
        [4.8675, 0.3234, 0.0, 0.0, 0.0, 0.0122, 0.9999]
    ]
    back_B = [
        [3.2185, 0.4004, 0.0, 0.0, 0.0, 0.9930, 0.1181],
        [1.1671, 0.5723, 0.0, 0.0, 0.0, 0.9283, -0.3717],
        [0.0, 0.0, 0.0, 0, 0, 0, 1]
    ]

    go_C = [
        [2.2358, -0.7021, 0.0, 0.0, 0.0, 0.0013, 1.0000],
        [4.9801, -0.3756, 0.0, 0.0, 0.0, 0.6579, 0.7531]
    ]
    back_C = [
        [4.1607, -0.7007, 0.0, 0.0, 0.0, 1.0000, 0.0026], 
        [2.3035, -0.6904, 0.0, 0.0, 0.0, 1.0000, -0.0012],
        [0.0, 0.0, 0.0, 0, 0, 0, 1]
    ]

    # =====================================
    #           根据 YOLO 值选择路径
    # =====================================
    first, second = received_values[0], received_values[1]

    # 出发点
    if first == 0:
        pose_sequence = go_A
        rospy.loginfo("出发：A")
    elif first == 1:
        pose_sequence = go_B
        rospy.loginfo("出发：B")
    else:
        pose_sequence = go_C
        rospy.loginfo("出发：C")

    # 返回点
    if second == 0:
        pose_sequence += back_A
        rospy.loginfo("返回：A")
    elif second == 1:
        pose_sequence += back_B
        rospy.loginfo("返回：B")
    else:
        pose_sequence += back_C
        rospy.loginfo("返回：C")

    rospy.loginfo("最终路径点序列: %s" % pose_sequence)

    rospy.sleep(5.0)

    # =====================================
    #           发布路径点（带超时与距离判断）
    # =====================================
    publish_pose_sequence(pose_pub, pose_sequence,
                          arrive_threshold=0.30,
                          timeout_sec=30.0)


if __name__ == '__main__':
    try:
        collector_node()
    except rospy.ROSInterruptException:
        pass

