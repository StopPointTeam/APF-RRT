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

    time.sleep(1)

    planner = APF(vision)
    planner.set_para(beta=100000000000, step=200)

    planner.plan()
    waypoint_list = planner.get_waypoint_list()

    package = Debug_Msgs()
    debugger.draw_circle(package, 2400, 1500, 100)
    debugger.draw_circle(package, -2400, -1500, 100)
    for i in range(len(waypoint_list) - 1):
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
