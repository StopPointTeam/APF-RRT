from vision import Vision
import math

from zss_debug_pb2 import Point


class Move:
    def __init__(self, vision: Vision, waypoint: list) -> None:
        self.vision = vision
        self.my_robot = self.vision.my_robot
        self.cur_x = self.vision.my_robot.x
        self.cur_y = self.vision.my_robot.y
        self.orientation = self.vision.my_robot.orientation
        self.waypoint = waypoint
        self.num_waypoint = len(waypoint)
        self.cur_waypoint = 0
        self.k1 = 3
        self.k2 = 3

    def move_plan(self):
        flag = 0
        self.cur_x = self.vision.my_robot.x
        self.cur_y = self.vision.my_robot.y
        next_distance = self._point_distance(
            self.cur_x, self.cur_y, self.waypoint[self.cur_waypoint+1][0], self.waypoint[self.cur_waypoint+1][1])
        pre_distance = self._point_distance(
            self.cur_x, self.cur_y, self.waypoint[self.cur_waypoint][0], self.waypoint[self.cur_waypoint][1])
        if next_distance < 20:
            self.cur_waypoint += 1
            if self.cur_waypoint == self.num_waypoint-1:
                vx = vw = 0
                flag = 1
                return vx, vw, flag
            else:
                next_distance = self._point_distance(
                    self.cur_x, self.cur_y, self.waypoint[self.cur_waypoint+1][0], self.waypoint[self.cur_waypoint+1][1])
                pre_distance = self._point_distance(
                    self.cur_x, self.cur_y, self.waypoint[self.cur_waypoint][0], self.waypoint[self.cur_waypoint][1])
        way_distance = self._point_distance(self.waypoint[self.cur_waypoint][0], self.waypoint[self.cur_waypoint]
                                            [1], self.waypoint[self.cur_waypoint+1][0], self.waypoint[self.cur_waypoint+1][1])

        theta = self.orientation = self.vision.my_robot.orientation
        beta = self._point_arccos(
            self.cur_x, self.cur_y, self.waypoint[self.cur_waypoint+1][0], self.waypoint[self.cur_waypoint+1][1])
        if self.orientation < 0:
            theta = 2*math.pi+self.orientation
        alpha = beta-theta
        if alpha < -math.pi:
            alpha = 2*math.pi+alpha
        elif alpha > math.pi:
            alpha = alpha-2*math.pi

        if pre_distance < (way_distance/5):
            vx = self.k1*pre_distance+50
        elif next_distance < (way_distance/5):
            vx = self.k1*next_distance+50
        else:
            vx = self.k1*way_distance/5+50

        vw = self.k2*alpha
        print(vx, vw)
        return vx, vw, flag

    def _point_distance(self, x1: float, y1: float, x2: float, y2: float):
        return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5)

    def _point_arccos(self, x1: float, y1: float, x2: float, y2: float):
        if y2-y1 >= 0:
            beta = math.acos((x2-x1)/self._point_distance(x1, y1, x2, y2))
        else:
            beta = 2*math.pi - \
                math.acos((x2-x1)/self._point_distance(x1, y1, x2, y2))
        return beta
