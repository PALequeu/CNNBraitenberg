from numpy import sign, Inf, exp, array, sum
import pygame
import math
import torch

MAXSPEED = 0.0004 * 3779.52  # meters/s
MINSPEED = 0.0001 * 3779.52  # meters/s
# MAXSPEED = 0
# MINSPEED = 0


def distance(point1, point2):
    point1 = array(point1)
    point2 = array(point2)

    return math.sqrt(sum((point1 - point2) ** 2))


class Robot:
    def __init__(self, startpos, target, width=0.01 * 3779.52):
        self.m2p = 3779.52  # meter to pixel
        # robot dims
        self.width = width

        # position and heading
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        # speed
        self.vl = 0  # meters/s
        self.vr = 0  # meters/s

        self.maxspeed = MAXSPEED  # meters/s
        self.minspeed = MINSPEED  # meters/s

        self.min_obj_dist = 100
        self.countdown = 5

    def move_braitenberg(
        self,
        detection,
        isInLoop,
        command=[0, 0],
    ):
        # command array
        new_command = [0, 0]

        # coefficients of the braitenberg algorithm
        braitenbergL = [0.05, 0.4, 0.9, 1.4, 1.7, -1.55, -1.4, -1.2, -0.8, -0.6]
        braitenbergR = [-0.6, -0.8, -1.2, -1.4, -1.55, 1.7, 1.4, 0.9, 0.4, 0.05]

        K = -20

        for i in range(10):
            braitenbergL[i] = K * braitenbergL[i]
            braitenbergR[i] = K * braitenbergR[i]

        if isInLoop:
            # Robot is in a loop to turn on itself

            # removing the extrem sensors
            detection_without_extrem = detection[3:7]
            # removing the 0 values from the detection list
            detectionWithoutZero = [x for x in detection_without_extrem if x != 0]
            minDetection = min(detectionWithoutZero, default=Inf)
            # print(minDetection)
            if minDetection > 10:
                print("out of loop")
                isInLoop = False
            # self.vl = command[0]
            # self.vr = command[1]
            print("command in loop", command)
            return command, isInLoop

        # new_command = [self.minspeed, self.minspeed]

        for i in range(10):
            if detection[i] != 0:
                new_command[0] += braitenbergL[i] * 1000 * exp(-detection[i])
                new_command[1] += braitenbergR[i] * 1000 * exp(-detection[i])

        detectionWithoutZero = [x for x in detection if x != 0]
        minDetection = min(detectionWithoutZero, default=Inf)

        if minDetection < 5:
            isInLoop = True
            print("in loop")
            minIndex = detection.index(minDetection)
            if minIndex < 4:
                command[0] = -8
                command[1] = 8
            else:
                command[0] = 8
                command[1] = -8
            return command, isInLoop

        # self.vl = command[0]
        # self.vr = command[1]

        # print(command)

        return new_command, isInLoop

    def kinematics(self, dt):
        # equations cinematiques du robot
        # if self.vr > self.maxspeed:
        #     self.vr = self.maxspeed
        # elif self.vr < -self.maxspeed:
        #     self.vr = -self.maxspeed
        # if self.vl > self.maxspeed:
        #     self.vl = self.maxspeed
        # elif self.vl < -self.maxspeed:
        #     self.vl = -self.maxspeed

        # if abs(self.vr) < self.minspeed:
        #     self.vr = sign(self.vr) * self.minspeed

        # if abs(self.vl) < self.minspeed:
        #     self.vl = sign(self.vl) * self.minspeed

        if self.vr > self.maxspeed:
            self.vr = self.maxspeed
        if self.vl > self.maxspeed:
            self.vl = self.maxspeed
        if self.vr < self.minspeed:
            self.vr = self.minspeed
        if self.vl < self.minspeed:
            self.vl = self.minspeed

        self.x += (self.vl + self.vr) / 2 * math.cos(self.heading) * dt
        self.y += (self.vl + self.vr) / 2 * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.width * dt

        if self.heading > 2 * math.pi or self.heading < -2 * math.pi:
            self.heading = 0


class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        # init colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.yel = (255, 255, 0)

        ##map##

        self.robot = pygame.image.load(robot_img_path)
        self.map_image = pygame.image.load(map_img_path)

        # dimensions

        self.width, self.height = dimensions[0], dimensions[1]

        # Window settings
        pygame.display.set_caption("Pionner Robot Controler")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_image, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(-heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)


class UltraSonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map = map
        self.map_width, self.map_height = pygame.display.get_surface().get_size()

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = float(heading - self.sensor_range[1])
        finish_angle = float(heading + self.sensor_range[1])

        # 0 à gauche, 9 à droite
        detection = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        index = 0  # index du capteur, de 0 à 9
        for angle in torch.linspace(start_angle, finish_angle, 10):
            # On calcule les potentielles coordonnées du point d'intersection avec le mur
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 + self.sensor_range[0] * math.sin(angle)

            for i in range(0, 100):
                # On découpe le segment en 100 parties et on regarde si il y a un mur à chaque point
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    # On récupère la couleur du pixel sur le point.
                    # Si c'est noir, on a un mur, on arrête la boucle et on ajoute
                    # le point à la liste des obstacles
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if color[0] == 0 and color[1] == 0 and color[2] == 0:
                        # il y a un mur
                        detection[index] = distance([x, y], [x1, y1]) / 10
                        obstacles.append([x, y])
                        break
            index += 1
        # print(detection)
        return obstacles, detection
