from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs

from apf import APF

import time


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()

    time.sleep(0.1)  # 防止未连接上仿真环境

    while (True):
        planner = APF(vision)
        planner.set_para(time_limit=200)

        # 规划路径
        waypoint_list = planner.plan(-2400, -1500)

        package = Debug_Msgs()
        debugger.draw_circle(package, 2400, 1500, 100)  # 绘制起点
        debugger.draw_circle(package, -2400, -1500, 100)  # 绘制终点
        for i in range(len(waypoint_list) - 1):  # 绘制路径
            debugger.draw_line(
                package, waypoint_list[i][0], waypoint_list[i][1], waypoint_list[i + 1][0], waypoint_list[i + 1][1])

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
