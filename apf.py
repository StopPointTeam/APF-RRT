from vision import Robot, Vision
from zss_debug_pb2 import Debug_Msgs

import math
import numpy


class APF:

    def __init__(self, vision: Vision):
        self.vision = vision
        self.robot = self.vision.yellow_robot
        self.my_robot = self.vision.my_robot

        self.alpha = 1
        self.beta = 1
        self.step = 10

        self.waypoint_list = []
        self.waypoint_list.append((self.my_robot.x, self.my_robot.y))

        self.current_x, self.current_y = self.waypoint_list[0]
        # print(self.waypoint_list)

    def get_waypoint_list(self):
        return self.waypoint_list

    def set_para(self, alpha=None, beta=None, step=None):
        if alpha is not None:
            self.alpha = alpha
        if beta is not None:
            self.beta = beta
        if step is not None:
            self.step = step

    def plan(self):

        for i in range(200):
            # 寻找最近的障碍物
            distance_min = 10000000.0
            nearest_bot: Robot = self.robot[0]
            for bot in self.robot:
                distance = pow(pow(self.current_x - bot.x, 2) +
                               pow(self.current_y - bot.y, 2), 1 / 2)
                if distance < distance_min:
                    distance_min = distance
                    nearest_bot = bot

            # 计算目标点带来的吸引力
            dest_force_x = self.alpha * (-2400 - self.current_x)
            dest_force_y = self.alpha * (-1500 - self.current_y)

            # 计算最近障碍物带来的斥力
            nearest_force_x = self.beta * \
                (self.current_x - nearest_bot.x) / pow(distance_min, 4)
            nearest_force_y = self.beta * \
                (self.current_y - nearest_bot.y) / pow(distance_min, 4)

            # 计算合力
            force_x = dest_force_x + nearest_force_x
            force_y = dest_force_y + nearest_force_y

            # 计算
            self.current_x += self.step * math.cos(math.atan(force_y / force_x)) * numpy.sign(force_x)
            self.current_y += self.step * math.sin(math.atan(force_y / force_x)) * numpy.sign(force_y)
            self.waypoint_list.append((self.current_x, self.current_y))
