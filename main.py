from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg

from apf import APF
from collision import Collision

import time


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    collision = Collision(vision)

    time.sleep(0.1)  # 防止未连接上仿真环境

    while True:
        planner = APF(vision)
        planner.set_para(time_limit=200)

        # 规划路径
        waypoint_list = planner.plan(-2400, -1500)

        package = Debug_Msgs()
        debugger.draw_circle(package, 2400, 1500, 100)  # 绘制起点
        debugger.draw_circle(package, -2400, -1500, 100)  # 绘制终点

        for i in range(len(waypoint_list) - 1):  # 绘制路径
            line_x1, line_y1 = waypoint_list[i]
            line_x2, line_y2 = waypoint_list[i + 1]

            # 检测是否发生碰撞
            if collision.is_collided_check_all(line_x1, line_y1, line_x2, line_y2) == False:
                debugger.draw_line(package, line_x1, line_y1,
                                   line_x2, line_y2, Debug_Msg.GREEN)
            else:
                debugger.draw_line(package, line_x1, line_y1,
                                   line_x2, line_y2, Debug_Msg.RED)  # 将发生碰撞的线段标为红色

        debugger.send(package)

    # while True:
    #     # 1. path planning & velocity planning
    #     planner.plan()

    #     # 2. send command
    #     action.sendCommand(vx=0, vw=0)

    #     # 3. draw debug msg
    #     package = Debug_Msgs()
    #     debugger.draw_circle(package, 2400, 1500, 100)
    #     debugger.draw_circle(package, -2400, -1500, 100)
    #     debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
    #     debugger.send(package)

    #     time.sleep(0.1)
