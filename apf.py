from vision import Robot, Vision
from zss_debug_pb2 import Debug_Msgs


class APF:

    def __init__(self, vision: Vision):
        self.vision = vision
        self.robot = self.vision.yellow_robot
        self.my_robot = self.vision.my_robot

        self.alpha = 1
        self.beta = 1
        self.step = 10
        self.max_single_step = 400

        self.waypoint_list = []
        self.waypoint_list.append((self.my_robot.x, self.my_robot.y))

        self.current_x, self.current_y = self.waypoint_list[0]
        # print(self.waypoint_list)

    def get_waypoint_list(self):
        return self.waypoint_list

    def set_para(self, alpha=None, beta=None, step=None, max_single_step=None):
        if alpha is not None:
            self.alpha = alpha
        if beta is not None:
            self.beta = beta
        if step is not None:
            self.step = step
        if max_single_step is not None:
            self.max_single_step = max_single_step

    def plan(self):

        for i in range(400):
            # 计算目标点带来的吸引力
            distance = pow(pow(-2400 - self.current_x, 2) +
                           pow(-1500 - self.current_y, 2), 1 / 2)
            if (distance > 500):
                dest_force_x = self.alpha * (-2400 - self.current_x)
                dest_force_y = self.alpha * (-1500 - self.current_y)
            elif (distance > 100):
                dest_force_x = 10 * self.alpha * (-2400 - self.current_x)
                dest_force_y = 10 * self.alpha * (-1500 - self.current_y)
            else:
                break

            # 计算障碍物带来的斥力
            obs_force_x = 0
            obs_force_y = 0

            for bot in self.robot:
                distance = pow(pow(self.current_x - bot.x, 2) +
                               pow(self.current_y - bot.y, 2), 1 / 2)

                if distance > 2000:
                    continue

                start_distance = pow(pow(bot.x - 2400, 2) +
                                     pow(bot.x - 1500, 2), 1 / 2)

                if distance < 1000 and start_distance < 800:
                    obs_force_x += 5 * self.beta * \
                        (self.current_x - bot.x) / pow(distance, 4)
                    obs_force_y += 5 * self.beta * \
                        (self.current_y - bot.y) / pow(distance, 4)
                else:
                    obs_force_x += self.beta * \
                        (self.current_x - bot.x) / pow(distance, 4)
                    obs_force_y += self.beta * \
                        (self.current_y - bot.y) / pow(distance, 4)

            # 计算合力
            force_x = dest_force_x + obs_force_x
            force_y = dest_force_y + obs_force_y

            print(force_x, force_y)

            if (force_x > self.max_single_step or force_y > self.max_single_step):
                if (force_x <= force_y):
                    force_x = force_x / force_y * self.max_single_step
                    force_y = self.max_single_step
                else:
                    force_y = force_y / force_x * self.max_single_step
                    force_x = self.max_single_step

            self.current_x += self.step * force_x
            self.current_y += self.step * force_y
            self.waypoint_list.append((self.current_x, self.current_y))

        # 400 步还未到达，直接连接
        self.waypoint_list.append((-2400, -1500))
