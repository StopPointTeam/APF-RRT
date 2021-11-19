from typing import List
from vision import Vision
from zss_debug_pb2 import Debug_Msgs


class APF:

    def __init__(self, vision: Vision) -> None:
        self.vision = vision
        self.obs_robot = self.vision.yellow_robot
        self.my_robot = self.vision.my_robot

        self.alpha = 1
        self.beta = 100000000000
        self.k = 0.1
        self.force_limit = 40
        self.time_limit = 100

        self.waypoint_list = []
        self.waypoint_list.append((self.my_robot.x, self.my_robot.y))

        self.current_x, self.current_y = self.waypoint_list[0]

    def set_para(self, alpha=None, beta=None, k=None, force_limit=None, time_limit=None) -> None:
        if alpha is not None:
            self.alpha = alpha
        if beta is not None:
            self.beta = beta
        if k is not None:
            self.k = k
        if force_limit is not None:
            self.force_limit = force_limit
        if time_limit is not None:
            self.time_limit = time_limit

    def plan(self, dest_x: float, dest_y: float) -> list:
        start_x = self.my_robot.x
        start_y = self.my_robot.y

        for i in range(self.time_limit):
            # 计算目标点带来的吸引力
            distance = self._point_distance(
                self.current_x, self.current_y, dest_x, dest_y)
            if (distance > 500):
                dest_force_x = self.alpha * (dest_x - self.current_x)
                dest_force_y = self.alpha * (dest_y - self.current_y)
            elif (distance > 100):  # 当距离目标较近时，增加吸引力，防止无法到达目标
                dest_force_x = 10 * self.alpha * (dest_x - self.current_x)
                dest_force_y = 10 * self.alpha * (dest_y - self.current_y)
            else:  # 完成
                break

            # 计算障碍物带来的斥力
            obs_force_x = 0
            obs_force_y = 0

            for bot in self.obs_robot:
                distance = self._point_distance(
                    self.current_x, self.current_y, bot.x, bot.y)

                if distance > 2000:  # 忽略距离过远的障碍物斥力
                    continue

                start_distance = self._point_distance(
                    bot.x, bot.y, start_x, start_y)

                if distance < 1000 and start_distance < 800:  # 优化起点，增加起点区域斥力，防止无法离开起点
                    obs_force_x += 5 * self.beta * \
                        (self.current_x - bot.x) / pow(distance, 4)
                    obs_force_y += 5 * self.beta * \
                        (self.current_y - bot.y) / pow(distance, 4)
                else:  # 正常情况
                    obs_force_x += self.beta * \
                        (self.current_x - bot.x) / pow(distance, 4)
                    obs_force_y += self.beta * \
                        (self.current_y - bot.y) / pow(distance, 4)

            # 计算合力
            force_x = dest_force_x + obs_force_x
            force_y = dest_force_y + obs_force_y

            # 限制合力大小
            if (force_x > self.force_limit or force_y > self.force_limit):
                if (force_x <= force_y):
                    force_x = force_x / force_y * self.force_limit
                    force_y = self.force_limit
                else:
                    force_y = force_y / force_x * self.force_limit
                    force_x = self.force_limit

            # 迭代
            self.current_x += 0.1 * force_x
            self.current_y += 0.1 * force_y
            self.waypoint_list.append((self.current_x, self.current_y))

        # 最大迭代次数后还未到达，直接连接
        self.waypoint_list.append((dest_x, dest_y))

        return self.waypoint_list

    def _point_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5)
