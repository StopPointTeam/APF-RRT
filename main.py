from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()

    while True:
        # 1. path planning & velocity planning
        # Do something

        # 2. send command
        action.sendCommand(vx=100, vy=0, vw=50)

        # 3. draw debug msg
        package = Debug_Msgs()
        debugger.draw_circle(package, 2400, 1500, 100)
        debugger.draw_circle(package, -2400, -1500, 100)
        debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
        debugger.send(package)

        time.sleep(0.01)
