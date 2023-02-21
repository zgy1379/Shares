#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-


import json
import rospy
import tf
import math
from geometry_msgs.msg import PolygonStamped, Point32,  PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import dynamic_reconfigure.client
import time
import numpy as np

# 定义 ANSI 转义序列
COLORS = {
    "red": "\033[31m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "blue": "\033[34m",
    "magenta": "\033[35m",
    "cyan": "\033[36m",
    "white": "\033[37m",
    "reset": "\033[0m"
}


def print_color(text, color):
    # 输出带颜色字体的文本
    print(COLORS[color] + text + COLORS["reset"])


class ConfigChanger(object):

    def __init__(self):
        rospy.init_node('config_changer')

        # Var declear
        self.odom = Odometry()
        self.is_triggered = False
        # self.trigger_region = PolygonStamped()
        self.position = Point32()

        # get params from parameter server
        self.odom_frame_id = rospy.get_param(
            "~odom_frame_id", "odom")
        self.global_frame_id = rospy.get_param(
            "~global_frame_id", "map")
        self.robot_frame_id = rospy.get_param(
            "~robot_frame_id", "base_link")
        self.new_config_path = rospy.get_param(
            "~new_config_path", "/home/ulrica/catkin_ws/src/config_changer/config/new_param.json")
        self.old_config_path = rospy.get_param(
            "~old_config_path", "/home/ulrica/catkin_ws/src/config_changer/config/new_param.json")
        self.set_curvity = rospy.get_param("~curvity", default=0.0)
        self.interval = rospy.get_param("~interval", default=0.0)
        self.point_num = rospy.get_param("~point_num", default=0)
        self.turn_delay = rospy.get_param("~turn_delay", default=0.0)
        try:
            with open(self.new_config_path, 'r') as f1:
                text1 = json.loads(f1.read())
            self.new_param = text1['new_param']
            with open(self.old_config_path, 'r') as f2:
                text2 = json.loads(f2.read())
            self.old_param = text2['old_param']
        except KeyError as e:
            print("KeyError when parseing config file: {}".format(e))
        self.rate = rospy.get_param(
            "~rate", 10)

        self.now = time.time()
        self.counter = 0
        self.list = [[0.0, 0.0] for i in range(self.point_num)]
        self.curvity = 0.0
        self.delay = 0.0

        self.tf_listener = tf.TransformListener()
        np.seterr(all='ignore')

    def _get_robot_pose(self):
        # lookup tf and get robot pose in map frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            if (time.time() - self.now) > self.interval:

                print_color(f"curvity = {self.curvity:8.5f}", "green")

                if self.delay > 0:
                    self.delay -= (time.time() - self.now)
                    print_color(f"delay = {self.delay:8.5f}", "blue")
                elif self.delay < 0:
                    self.delay = 0.0
                self.now = time.time()
                self.counter += 1
                self.counter %= self.point_num
                self.position.x = trans[0]
                self.position.y = trans[1]
                self.list[self.counter][0] = self.position.x
                self.list[self.counter][1] = self.position.y
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("tf lookup failed, may not switch config")

    def update_to_new_param(self):
        client = dynamic_reconfigure.client.Client(
            '/move_base/TebLocalPlannerROS')
        config = client.update_configuration(
            self.new_param['/move_base/TebLocalPlannerROS'])

        client = dynamic_reconfigure.client.Client(
            '/move_base/local_costmap')
        config = client.update_configuration(
            self.new_param['/move_base/local_costmap'])

    def return_to_old_param(self):
        client = dynamic_reconfigure.client.Client(
            '/move_base/TebLocalPlannerROS')
        config = client.update_configuration(
            self.old_param['/move_base/TebLocalPlannerROS'])

        client = dynamic_reconfigure.client.Client(
            '/move_base/local_costmap')
        config = client.update_configuration(
            self.old_param['/move_base/local_costmap'])

    def check_curve(self):

        coords_list = self.list
        degree = 2
        curvity = self.set_curvity
        """
        用二维坐标拟合多项式并计算曲率

        参数：
        coords_list：包含二维坐标的列表，形如[[x1, y1], [x2, y2], ..., [xn, yn]]
        degree：多项式的阶数
        curvity：曲率阈值，当曲率大于该值时，返回True

        返回：
        True或False，表示曲线是否弯曲程度大于曲率阈值
        """
        # 将坐标列表转换为NumPy数组
        data = np.array(coords_list)

        # 检查列表是否全空
        check = False
        for i in data[:, 1]:
            if i != 0:
                check = True
        if check == False:
            return False

        # 多项式回归拟合
        x = data[:, 0]
        y = data[:, 1]
        p = np.polyfit(x, y, degree)

        # 计算拟合曲线的曲率
        dp = np.polyder(p)
        ddp = np.polyder(dp)
        k = np.abs(ddp) / np.power(1 + np.power(dp, 2), 1.5)

        # 判断曲率是否大于曲率阈值
        self.curvity = np.max(k)
        if 0.0001 < self.curvity < curvity:
            return True
        else:
            return False

    def is_in_triger_region(self):
        if self.check_curve():
            return True
        else:
            return False


def main():
    node = ConfigChanger()
    rate = rospy.Rate(node.rate)
    while not rospy.is_shutdown():
        node._get_robot_pose()
        if node.is_in_triger_region() & (not node.is_triggered):
            node.is_triggered = True
            print_color("The path curve reaches a critical radius", "red")
            print_color("Start to change config", "red")
            node.update_to_new_param()
            print_color("Config changed", "red")
            node.delay = node.turn_delay
        elif (not node.is_in_triger_region()) & node.is_triggered & (not node.delay):
            node.is_triggered = False
            print_color(
                "The path curve separates from the critical radius", "magenta")
            print_color("Start to return config", "magenta")
            node.return_to_old_param()
            print_color("Config returned", "magenta")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
