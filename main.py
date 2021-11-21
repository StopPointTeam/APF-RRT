from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg

from apf import APF
from collision import Collision
from simplifier import Simplifier

import time


def draw_waypoint(waypoint_list: list,
                  collision: Collision,
                  debugger: Debugger,
                  good_way_color=Debug_Msg.GREEN,
                  bad_way_color=Debug_Msg.RED):
    for i in range(len(waypoint_list) - 1):
        line_x1, line_y1 = waypoint_list[i]
        line_x2, line_y2 = waypoint_list[i + 1]

        # 检测是否发生碰撞
        if collision.is_collided_check_all(line_x1, line_y1, line_x2, line_y2) == False:
            debugger.draw_line(package, line_x1, line_y1,
                               line_x2, line_y2, good_way_color)
        else:
            debugger.draw_line(package, line_x1, line_y1,
                               line_x2, line_y2, bad_way_color)


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    collision = Collision(vision)
    simplifier = Simplifier(vision)

    time.sleep(0.1)  # 防止未连接上仿真环境

    while True:
        package = Debug_Msgs()

        planner = APF(vision)
        planner.set_para(time_limit=200)

        # 规划路径
        waypoint_list = planner.plan(-2400, -1500)
        draw_waypoint(waypoint_list, collision, debugger, good_way_color=Debug_Msg.YELLOW)

        # 化简路径
        waypoint_list = simplifier.simplify(waypoint_list)
        draw_waypoint(waypoint_list, collision, debugger)

        debugger.draw_circle(package, 2400, 1500, 100)  # 绘制起点
        debugger.draw_circle(package, -2400, -1500, 100)  # 绘制终点

        debugger.send(package)
