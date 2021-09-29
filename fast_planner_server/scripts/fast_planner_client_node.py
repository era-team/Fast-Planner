#!/usr/bin/env python
# coding=utf-8

import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from fast_planner_server.msg import FastPlannerAction


class FastPlannerActionCient:
    """Класс action-клиента для fast-planner'а. Нужен для подачи 2D Nav Goal в action-сервер"""

    # Имя action-клиента
    _client_name = "fast_planner_client"

    def __init__(self):
        # Создаем action-клиента
        self.client = SimpleActionClient("fast_planner_server", FastPlannerAction)
        # Ожидаем сервер
        self.client.wait_for_server()
        # Подписываемся на топик цели от move_base
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_cb
        )

    def goal_cb(self, msg):
        """Callback цели

        Args:
            msg (geometry_msgs.msg.PoseStamped): цель
        """
        # Задаем высоту всем целям в 1.5 метра
        msg.pose.position.z = 1.5
        # Отправляем цель в action-сервер
        self.client.send_goal(msg)
        # Ждем результата
        self.client.wait_for_result()
        # Запрашиваем результат
        self.client.get_result()


if __name__ == "__main__":
    node_name = "fast_planner_client_node"
    rospy.init_node(node_name)
    rospy.loginfo("{0} was launched!".format(node_name))
    fp_client = FastPlannerActionCient()
    rospy.spin()
