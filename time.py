# -*- coding: utf-8 -*-
#
# Python2 Python 2 Python2 重要的事情说三遍！
# ROS Melodic 不支持 Python3，使用 Python3 调用 ROS 提供的 API 有可能报错（如本程序）

from __future__ import print_function
from math import sqrt
from ros_module import ROSNavNode
import time


def main():

    node = ROSNavNode()                                             # 启动节点

    # start = time.time()                                           # 开始比赛计时
    start = node.curr_time.secs
    n_start = node.curr_time.nsecs
    print("开始发送终点坐标...\n")
    node.send_goal()

    period = 30                                                     # 记录周期 30s
    interval = 0.2                                                  # 记录周期 30s
    # 保证刚开始导航时记录一次 Topic list
    log_start = node.curr_time.secs - period
    log_begin = time.time()
    now_x = node.last_pose.x
    now_y = node.last_pose.y
    vel_begin = 0
    while not node.get_state():
        # 每隔一个 period 记录一次 Topic list
        if int(node.curr_time.secs - log_start) > period:
            # print(node.get_topic())
            log_start = node.curr_time.secs
        # print(node.client.get_goal_status_text())
        timespace = time.time() - log_begin
        if timespace > interval:
            vel = sqrt(pow((now_x-node.last_pose.x)*10000, 2)+pow((now_y -
                       node.last_pose.y)*10000, 2))/((timespace)*10000)
            acc = (vel-vel_begin)/timespace
            print("velocity     = ", vel)
            print("acceleration = ", acc)
            log_begin = time.time()
            now_x = node.last_pose.x
            now_y = node.last_pose.y
            vel_begin = vel
        time.sleep(0.01)

    end = node.curr_time.secs
    n_end = node.curr_time.nsecs
    cost = end - start
    if n_end > n_start:
        n_cost = n_end - n_start
    else:
        n_cost = n_end + 1000000000 - n_start

    n_cost = float(n_cost) / 1000000000.0

    print('成功到达目标！一共执行了' + str(cost+n_cost) + ' 秒')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\node')
        exit(0)
