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
        self.set_radius = rospy.get_param("radius", 0.0)
        self.interval = rospy.get_param("interval", 0.0)
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
        self.list = [[0, 0], [0, 0], [0, 0]]

        # pt1 = Point32()
        # pt2 = Point32()
        # pt3 = Point32()
        # pt4 = Point32()

        # pt1.x = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[0][0]
        # pt1.y = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[0][1]
        # pt2.x = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[1][0]
        # pt2.y = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[1][1]
        # pt3.x = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[2][0]
        # pt3.y = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[2][1]
        # pt4.x = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[3][0]
        # pt4.y = rospy.get_param(
        #     "~trigger_region", [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])[3][1]

        # self.trigger_region.header.frame_id = self.global_frame_id
        # self.trigger_region.polygon.points.append(pt1)
        # self.trigger_region.polygon.points.append(pt2)
        # self.trigger_region.polygon.points.append(pt3)
        # self.trigger_region.polygon.points.append(pt4)

        # ROS Infastructure
        self.tf_listener = tf.TransformListener()
        # self.pub_trigger_region = rospy.Publisher(
        #     "trigger_region", PolygonStamped, queue_size=1)

    def _get_robot_pose(self):
        # lookup tf and get robot pose in map frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            if time.time() - self.now > self.interval:

                # print_color(f"angle  = {self.angle:8.5f}","green")
                print_color(f"radius = {self.radius:8.5f}","magenta")
                
                self.now = time.time()
                self.counter += 1
                self.counter %= 3
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

    def curvature(self):
        # 计算三个点的向量
        dx1 = self.list[1][0] - self.list[0][0]
        dy1 = self.list[1][1] - self.list[0][1]
        dx2 = self.list[2][0] - self.list[1][0]
        dy2 = self.list[2][1] - self.list[1][1]

        # 计算向量的长度
        mag1 = math.sqrt(dx1**2 + dy1**2)
        mag2 = math.sqrt(dx2**2 + dy2**2)

        # 计算向量的单位向量
        if mag1 != 0:
            ux1 = dx1 / mag1
            uy1 = dy1 / mag1
        else:
            ux1 = 0
            uy1 = 0

        if mag2 != 0:
            ux2 = dx2 / mag2
            uy2 = dy2 / mag2
        else:
            ux2 = 0
            uy2 = 0

        # 计算向量的夹角
        dotprof = ux1 * ux2 + uy1 * uy2
        self.angle = math.acos(dotprof)

        # 如果夹角为0，则曲率半径为无限大
        if self.angle == 0:
            return float('inf')

        # 计算曲率半径
        self.radius = mag1 / (2.0 * math.sin(self.angle / 2.0))

        return self.radius > self.set_radius

    def is_in_triger_region(self):
        # # check is robot in trigger region
        # x_min = self.trigger_region.polygon.points[0].x
        # x_max = self.trigger_region.polygon.points[0].x
        # y_min = self.trigger_region.polygon.points[0].y
        # y_max = self.trigger_region.polygon.points[0].y
        # # find the x_min, y_min, x_max, y_max of the trigger region
        # for point in self.trigger_region.polygon.points:
        #     if point.x < x_min:
        #         x_min = point.x
        #     if point.x > x_max:
        #         x_max = point.x
        #     if point.y < y_min:
        #         y_min = point.y
        #     if point.y > y_max:
        #         y_max = point.y

        # # check is robot in trigger region
        # if self.position.x >= x_min and \
        #         self.position.x <= x_max and \
        #         self.position.y >= y_min and \
        #         self.position.y <= y_max:
        if self.curvature():
            return True
        else:
            return False


def main():
    node = ConfigChanger()
    rate = rospy.Rate(node.rate)
    while not rospy.is_shutdown():
        # node.pub_trigger_region.publish(node.trigger_region)
        node._get_robot_pose()
        if node.is_in_triger_region() & (not node.is_triggered):
            node.is_triggered = True
            # print("robot is in trigger region")
            print("The path curve reaches a critical radius")
            print("Start to change config")
            node.update_to_new_param()
            print("Config changed")
        elif (not node.is_in_triger_region()) & node.is_triggered:
            node.is_triggered = False
            print("The path curve separates from the critical radius")
            print("Start to return config")
            node.return_to_old_param()
            print("Config returned")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
