#!/usr/bin/env python
# coding=utf-8

import rospy
from actionlib import SimpleActionServer
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from takeoff_common.takeoffpy import MavController, AutoPilot

from fast_planner_server.msg import (
    FastPlannerAction,
    FastPlannerFeedback,
    FastPlannerGoal,
    FastPlannerResult,
)


class FastPlannerActionServer:
    """Класс-обертка в виде action-сервера для fast-planner'а"""

    # Сообщение с обратной связью
    _feedback = FastPlannerFeedback()
    # Сообщение с результатом
    _result = FastPlannerResult()

    # TODO: сделать параметры ros-параметрами
    # TODO: придумать понятные названия параметрам
    def load_params(self):
        """Загружает и устанавливает параметры"""
        # Имя action-server'а
        self._action_name = "fast_planner_server"
        # Тип автопилота
        self.ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        # Использовать визуальную одометрию или нет
        self.use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        # Удерживать высоту при получении нулевой скорости
        self.enable_althold = rospy.get_param("~enable_althold", True)
        #  Высота, на которой летает дрон в метрах
        self.altitude = rospy.get_param("~altitude", 1.5)

        # Предыдущее расстояние до цели
        self.old_goal_dist = float("inf")
        # Считает, как долго дрон не трогается с места
        self.paralysis_counter = 0
        # Максимальное количество итераций простоя для дрона
        self.paralysis_threshold = 400
        # Минимальное расстояние до цели, чтобы считать ее достигнутой
        self.goal_min_dist = 0.2
        # Минимальный прирост расстояния до цели, чтобы не увеличивать счетчик простоя
        self.error_dist_threshold = 0.01

    def send_traj_point(self):
        """Отправляет точку траектории дрону"""
        self.drone.take_pos(
            self.traj_msg.position.x,
            self.traj_msg.position.y,
            self.traj_msg.position.z,
            self.traj_msg.yaw,
        )

    def goal_cb(self):
        """Callback при получении точки цели"""
        # Взлетаем, если дрон на земле
        if hasattr(self, "drone"):
            if self.drone.is_landed:
                self.drone.takeoff(height=self.altitude)
        # Запоминаем предыдущее положение дрона
        self.old_pos = self.drone.local_position.pose.position
        # Запоминаем цель
        self.goal = self.server.accept_new_goal()
        # Отправляем цель в fast-planner
        self.goal_pub.publish(self.goal)

    def preempt_cb(self):
        """Callback отмены цели"""
        self.server.set_preempted()

    def timer_cb(self, event):
        """Callback таймера удержания высоты

        Args:
            event (rospy.TimerEvent): событие таймера
        """
        # Если:
        # - нет активной цели
        # - дрон находится в воздухе
        # - включен режим удержания высоты
        if not (self.server.is_active() and self.drone.is_landed):
            if self.enable_althold:
                # Удерживаем высоту
                self.drone.take_altitude(self.altitude)

    def track_error(self, goal_dist):
        """Отслеживает накопление ошибки

        Args:
            goal_dist (float): расстояние до цели

        Returns:
            bool: возникла ли ошибка при следовании к цели
        """
        if abs(self.old_goal_dist - goal_dist) < self.error_dist_threshold:
            self.paralysis_counter += 1
        else:
            self.paralysis_counter = 0
        return self.paralysis_counter > self.paralysis_threshold

    def trajectory_cb(self, traj_msg):
        """Callback-метод топика с точками траектории дрона

        Args:
            traj_msg (quadrotor_msgs.msg.PositionCommand): точка траектории и скорость в ней
        """
        # Если цель получена и не отменена
        if self.server.is_active() and not self.server.is_preempt_requested():
            # Сохраняем полученное сообщение точки траектории
            self.traj_msg = traj_msg
            # Проверяем достиг ли дрон цели
            goal_dist = self.drone.distance_2d(self.goal.pose.position)
            reach_goal = goal_dist <= self.goal_min_dist
            # Проверяем произошла ли ошибка
            is_error = self.track_error(goal_dist)
            # Проверяем достижение цели
            if reach_goal:  # Если достиг цели
                self._result.success = True
                self.server.set_succeeded(self._result)
            elif is_error:  # Если произошла ошибка
                self._result.success = False
                self.server.set_succeeded(self._result)
            else:  # Летим в штатном режиме
                # Публикуем feedback
                self._feedback.goal_distance = goal_dist
                self.server.publish_feedback(self._feedback)
                # Отправляем точку дрону
                self.send_traj_point()
            # Запоминаем расстояние до цели
            self.old_goal_dist = goal_dist
            # Обновляем значение предыдущего положения
            self.old_pos = traj_msg.position

    def __init__(self):
        # Загружаем параметры
        self.load_params()

        # Создаем action-server
        self.server = SimpleActionServer(
            self._action_name, FastPlannerAction, auto_start=False
        )
        # Задаем callback'и при получении цели и ее отмене
        self.server.goal_callback = self.goal_cb
        self.server.preempt_callback = self.preempt_cb

        # Создаем дрона
        self.drone = MavController.create_controller(
            self.ap_type, use_vision_odometry=self.use_vision_odometry
        )

        # Создаем publisher цели для fast-planner'а
        self.goal_pub = rospy.Publisher("/fast_planner/goal", PoseStamped, queue_size=4)

        # Подписываемся на точки траектории от fast-planner'а
        self.pos_cmd_sub = rospy.Subscriber(
            "/planning/pos_cmd", PositionCommand, self.trajectory_cb
        )

        # Запускаем action-server
        self.server.start()

        # Создаем таймер для удержания высоты
        self.althold_timer = rospy.Timer(rospy.Duration(0.3), self.timer_cb)


if __name__ == "__main__":
    node_name = "fast_planner_server_node"
    rospy.init_node(node_name)
    rospy.loginfo("{0} was launched!".format(node_name))
    fp_server = FastPlannerActionServer()
    rospy.spin()