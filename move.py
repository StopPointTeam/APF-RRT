import math
from datetime import datetime
from vision import Vision


MAX_VX = 3000
MAX_AX = 3000
MAX_VW = 5
MAX_AW = 5
ACC = 0
DEC = 1


class Move:
    def __init__(self, vision: Vision, waypoint: list, vx, vw) -> None:
        self.vision = vision
        self.my_robot = self.vision.my_robot

        self.cur_x = self.my_robot.x
        self.cur_y = self.my_robot.y
        self.orientation = self.my_robot.orientation
        self.cur_vx = vx
        self.cur_vw = vw

        self.waypoint = waypoint
        self.num_waypoint = len(waypoint)
        self.cur_waypoint = 0

        self.cur_point_angle = self._turn_angle(self.cur_waypoint)
        self.next_point_angle = self._turn_angle(self.cur_waypoint+1)
        self.cur_point_speed = self._turn_speed(self.cur_point_angle)
        self.next_point_speed = self._turn_speed(self.next_point_angle)
        self.is_angle_update = False

        self.k1 = 2
        self.k21 = 3
        self.k22 = 0.5
        
        self.cur_time = datetime.now()

        self.cur_status = ACC

    def move_plan(self) -> tuple:
        flag = 0
        self.cur_x = self.my_robot.x
        self.cur_y = self.my_robot.y

        next_distance = self._point_distance(self.cur_x, self.cur_y,
                                             self.waypoint[self.cur_waypoint + 1][0],
                                             self.waypoint[self.cur_waypoint + 1][1])

        pre_distance = self._point_distance(self.cur_x, self.cur_y,
                                            self.waypoint[self.cur_waypoint][0],
                                            self.waypoint[self.cur_waypoint][1])

        if next_distance < 20:
            self.cur_waypoint += 1
            if self.cur_waypoint == self.num_waypoint - 1:
                vx = vw = 0
                flag = 1
                return vx, vw, flag
            else:
                self.cur_point_angle = self._turn_angle(self.cur_waypoint)
                self.next_point_angle = self._turn_angle(self.cur_waypoint+1)
                self.cur_point_speed = self._turn_speed(self.cur_point_angle)
                self.next_point_speed = self._turn_speed(self.next_point_angle)
                self.cur_status = ACC
                
                next_distance = self._point_distance(self.cur_x, self.cur_y,
                                                     self.waypoint[self.cur_waypoint + 1][0],
                                                     self.waypoint[self.cur_waypoint + 1][1])

                pre_distance = self._point_distance(self.cur_x, self.cur_y,
                                                    self.waypoint[self.cur_waypoint][0],
                                                    self.waypoint[self.cur_waypoint][1])
                

        way_distance = self._point_distance(self.waypoint[self.cur_waypoint][0],
                                            self.waypoint[self.cur_waypoint][1],
                                            self.waypoint[self.cur_waypoint + 1][0],
                                            self.waypoint[self.cur_waypoint + 1][1])

        theta = self.orientation = self.my_robot.orientation
        beta = self._point_arccos(self.cur_x, self.cur_y,
                                  self.waypoint[self.cur_waypoint+1][0],
                                  self.waypoint[self.cur_waypoint+1][1])

        if self.orientation < 0:
            theta = 2 * math.pi + self.orientation

        alpha = beta - theta 

        if alpha < - math.pi:
            alpha = 2 * math.pi + alpha
        elif alpha > math.pi:
            alpha = alpha - 2 * math.pi
        
        delta_t = (datetime.now() - self.cur_time).microseconds * 0.000001
        self.cur_time = datetime.now()

        # print(self.cur_vx)
        # print(pre_distance,next_distance)
        # print(self.cur_point_angle, self.next_point_angle)

        # vx规划
        if self.cur_status == ACC:
            if self.cur_vx - self.next_point_speed >= self.k1 * next_distance:
                self.cur_status = DEC
                vx = self.next_point_speed + self.k1 * next_distance
            else:
                vx = self.cur_point_speed + self.k1 * pre_distance
        elif self.cur_status == DEC:
            vx = self.next_point_speed + self.k1 * next_distance

        # print(self.cur_status)
        # print('------------------------')
        
        # 限制vx范围
        vx = self.limit_vx_for_alpha(vx, alpha)
        vx_min,vx_max = self._vx_space(delta_t)
        if vx < vx_min:
            vx = vx_min
        elif vx > vx_max:
            vx = vx_max 
        self.cur_vx = vx

        # vw规划
        if alpha < math.pi/2 and alpha > -math.pi/2:
            vw = self.k21 * alpha
        elif alpha > math.pi/2:
            vw = self.k21 * math.pi/2 + self.k22 * (alpha - math.pi/2)
        elif alpha < -math.pi/2:
            vw = -self.k21 * math.pi/2 + self.k22 * (-alpha + math.pi/2)
        
        vw_min,vw_max = self._vw_space(delta_t)
        if vw < vw_min:
            vw = vw_min
        elif vw > vw_max:
            vw = vw_max 
        self.cur_vw = vw

        # print(self.cur_waypoint,self.next_point_angle,self.next_point_speed)
        return vx, vw, flag

    def _point_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5)

    def _point_arccos(self, x1: float, y1: float, x2: float, y2: float) -> float:
        if y2 - y1 >= 0:
            beta = math.acos((x2-x1) / self._point_distance(x1, y1, x2, y2))
        else:
            beta = 2 * math.pi - \
                math.acos((x2-x1) / self._point_distance(x1, y1, x2, y2))
        return beta

    def _turn_speed(self, alpha: float) -> float:
        degree = abs(alpha / (2 * math.pi) * 360)
        # if degree < 30:
        #     turn_speed = 14/9 * pow((degree - 30), 2) + 600
        # elif degree < 85:
        #     turn_speed = -10 * degree + 900
        if degree < 70:
            turn_speed = -10 * degree + 750
        else:
            turn_speed = 50
        return turn_speed

    def _vx_space(self, delta_t: float) -> tuple:
        vx_max = min(MAX_VX, self.cur_vx+delta_t*MAX_AX)
        vx_min = max(0, self.cur_vx-delta_t*MAX_AX)
        return vx_min, vx_max

    def _vw_space(self, delta_t: float) -> tuple:
        vw_max = min(MAX_VW, self.cur_vw+delta_t*MAX_AW)
        vw_min = max(-MAX_VW, self.cur_vw-delta_t*MAX_AW)
        return vw_min, vw_max
    
    def get_speed(self) -> tuple:
        return self.cur_vx, self.cur_vw
    
    def get_location(self) -> int:
        return self.cur_waypoint
    
    def _turn_angle(self, seq: int) -> float:
        if seq == self.num_waypoint -1:
            angle = 180
            return angle

        if seq == 0:
            pre_angle = self.orientation
            if pre_angle < 0:
                pre_angle = 2 * math.pi + pre_angle
        else:
            pre_angle = self._point_arccos(self.waypoint[seq-1][0],
                                           self.waypoint[seq-1][1],
                                           self.waypoint[seq][0],
                                           self.waypoint[seq][1])
        
        next_angle = self._point_arccos(self.waypoint[seq][0],
                                        self.waypoint[seq][1],
                                        self.waypoint[seq+1][0],
                                        self.waypoint[seq+1][1])
        angle = next_angle-pre_angle
        if angle < - math.pi:
            alpha = 2 * math.pi + angle
        elif angle > math.pi:
            alpha = angle - 2 * math.pi
        return angle
    
    def limit_vx_for_alpha(self, vx0:float, alpha: float) -> float:
        degree = abs(alpha / (2 * math.pi) * 360)
        if degree >= 70:
            vx = 0
        elif degree >= 30:
            vx = vx0 / 3
        elif degree >= 10:
            vx = vx0 / 2
        else:
            vx = vx0
        return vx
