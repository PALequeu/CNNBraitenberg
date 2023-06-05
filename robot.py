from numpy import sign
import pygame
import math

MAXSPEED = 0.0004 * 3779.52  # meters/s
MINSPEED = 0.0001 * 3779.52  # meters/s
# MAXSPEED = 0
# MINSPEED = 0


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
