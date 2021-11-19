class Collision:

    def __init__(self):
        self.my_r = 80
        self.obs_r = 80

    def set_para(self, my_r=None, obs_r=None):
        if my_r is None:
            self.my_r = my_r
        if obs_r is None:
            self.obs_r = obs_r

    def is_collision(self, line_x1: float, line_y1: float, line_x2: float, line_y2: float, obs_x: float, obs_y: float) -> bool:
        cross = (line_x2 - line_x1) * (obs_x - line_x1) + \
            (line_y2 - line_y1) * (obs_y - line_y1)
        if cross <= 0:
            distance = self._point_distance(line_x1, line_y1, obs_x, obs_y)
        else:
            d2 = (line_x2 - line_x1) * (line_x2 - line_x1) + \
                (line_y2 - line_y1) * (line_y2 - line_y1)
            if cross >= d2:
                distance = self._point_distance(line_x2, line_y2, obs_x, obs_y)
            else:
                r = cross / d2
                px = line_x1 + (line_x2 - line_x1) * r
                py = line_y1 + (line_y2 - line_y1) * r
                distance = self._point_distance(obs_x, py, px, obs_y)

        if distance < self.my_r + self.obs_r:
            return True
        else:
            return False

    def _point_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5)
