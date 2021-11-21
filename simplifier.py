from vision import Vision
from collision import Collision


class Simplifier:

    def __init__(self, vision: Vision):
        self.vision = vision
        self.collision = Collision(self.vision)

    def simplify(self, waypoint_list: list) -> list:
        new_waypoint_list = []
        new_waypoint_list.append(waypoint_list[0])

        i = 0
        while i < len(waypoint_list) - 1:
            line_x1, line_y1 = waypoint_list[i]

            j = len(waypoint_list) - 1
            while j != i:
                if self.collision.is_collided_check_all(line_x1, line_y1,
                                                        waypoint_list[j][0], waypoint_list[j][1]) == False:
                    new_waypoint_list.append(waypoint_list[j])
                    i = j - 1
                    break

                j -= 1

            if (j == i):
                new_waypoint_list.append(waypoint_list[i + 1])

            i += 1

        return new_waypoint_list
