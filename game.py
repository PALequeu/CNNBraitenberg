import pygame
import random
from mapGenerator import generate_map
from collections import namedtuple
from robot import Robot, Graphics, UltraSonic, MAXSPEED
import numpy as np
import matplotlib.pyplot as plt

pygame.init()
font = pygame.font.Font("arial.ttf", 25)

START_POS = (100, 100)
Point = namedtuple("Point", "x, y")

SPEED = 20


def distance(p1, p2):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


class RobotGame:
    def __init__(self, training, w=1400, h=600, Empty=False):
        self.w = w
        self.h = h
        self.sensor_range = 200, np.radians(60)
        self.reset(training)

    def reset(self, training):
        self.score = 0
        self.target = None
        self._place_target()
        self.robot = Robot(START_POS, self.target)
        self.initial_distance = distance(self.robot, self.target)
        self.frame_iteration = 0
        self.start_ticks = pygame.time.get_ticks()
        self.clock = pygame.time.Clock()
        self.seconds_array = []
        self.vl_array = []
        self.vr_array = []

        self.map = generate_map(self.w, self.h, self.target, training)
        self.gfx = Graphics((self.w, self.h), "robot.png", "test_map.png")
        self.gfx.draw_robot(self.robot.x, self.robot.y, self.robot.heading)
        self.ultrasonic = UltraSonic(self.sensor_range, self.gfx.map)

    def _place_target(self):
        x = random.randint(200, (self.w - 40))
        y = random.randint(40, (self.h - 40))
        self.target = Point(x, y)
        # TODO: check if robot is close enough to target

    def play_step(self, action, last_distance):
        seconds = (pygame.time.get_ticks() - self.start_ticks) / 1000
        self.frame_iteration += 1
        # 1. collect user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        # print("position", self.robot.x, self.robot.y)
        # print("speed", self.robot.vl, self.robot.vr)
        # i dont know if this is needed
        self.gfx.map.blit(self.gfx.map_image, (0, 0))

        # 2. move
        self._move(action)
        self.robot.kinematics(SPEED / 10)

        # draw robot
        self.gfx.draw_robot(self.robot.x, self.robot.y, self.robot.heading)

        # update display
        pygame.display.update()

        # 3. check if game over
        reward = 0
        game_over = False

        dist = distance(self.robot, self.target)

        # reward for getting closer to target
        closer_reward = (last_distance - dist) * 10 - 3  # - 3 penalize for staying put

        # penalize the difference between the left and right wheel velocities to avoid spinning on itself
        difference_reward = (
            (abs(self.robot.vl - self.robot.vr)) / self.robot.maxspeed
        ) * 2

        # reward for being faster
        faster_reward = abs((self.robot.vl + self.robot.vr) / 2) / self.robot.maxspeed

        reward += closer_reward - difference_reward + faster_reward

        self.seconds_array.append(seconds)
        self.vl_array.append(self.robot.vl)
        self.vr_array.append(self.robot.vr)

        if distance(self.robot, self.target) < 40:
            game_over = True
            reward += 5
            if reward < 0:
                reward = reward * 0.5
            self.score = (1 / dist) * 100 + 1 / seconds * 10 + 5
            print(reward)
            return reward, game_over, self.score, dist

        if seconds > 15:
            game_over = True
            if reward < 0:
                reward = reward * 0.5
            self.score = 1 / dist * 100
            print("time out")

            return reward, game_over, self.score, dist

        self.clock.tick(SPEED)

        if reward < 0:
            reward = reward * 0.5
        self.score = 1 / dist * 100

        return reward, game_over, self.score, dist

    def play_step2(self, action, isInLoop, counter, command=[0, 0]):
        game_over = False
        seconds = (pygame.time.get_ticks() - self.start_ticks) / 1000
        print("seconds", seconds)
        self.frame_iteration += 1
        # 1. collect user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        # print("position", self.robot.x, self.robot.y)
        # print("speed", self.robot.vl, self.robot.vr)
        # i dont know if this is needed
        self.gfx.map.blit(self.gfx.map_image, (0, 0))

        self.gfx.draw_robot(self.robot.x, self.robot.y, self.robot.heading)

        # 2. find move with CNN

        CNN_command = self._move2(action)  # command = changes only

        # 3. find move with braitenberg

        point_cloud, detection = self.ultrasonic.sense_obstacles(
            self.robot.x, self.robot.y, self.robot.heading
        )

        command, isInLoop = self.robot.move_braitenberg(detection, isInLoop, command)
        self.gfx.draw_sensor_data(point_cloud)

        Braitenberg_command = command  # command = changes only

        # 4. move with CNN and Braitenberg
        # print(CNN_command, Braitenberg_command)
        if isInLoop:
            # if in loop, only use braitenberg
            self.robot.vl = Braitenberg_command[0]
            self.robot.vr = Braitenberg_command[1]
        else:
            self.robot.vl = (
                self.robot.minspeed
                + 0.4 * Braitenberg_command[0]
                + 1.3 * CNN_command[0]
            )
            self.robot.vr = (
                self.robot.minspeed
                + 0.4 * Braitenberg_command[1]
                + 1.3 * CNN_command[1]
            )

        self.robot.kinematics(SPEED / 50)

        # print(linspacing)
        # print(seconds in linspacing)
        # update display
        pygame.display.update()

        if counter == 10:
            print("nice")
            self.seconds_array.append(seconds)
            self.vl_array.append(self.robot.vl)
            self.vr_array.append(self.robot.vr)
            counter = 0
        else:
            counter += 1

        # check if game over
        if distance(self.robot, self.target) < 40 or seconds > 20:
            game_over = True

        return isInLoop, game_over, counter

    def _move(self, action):
        speed = 0.1
        if np.array_equal(action, [1, 0, 0, 0, 0]):
            self.robot.vl += speed
        elif np.array_equal(action, [0, 1, 0, 0, 0]):
            self.robot.vl -= speed
        elif np.array_equal(action, [0, 0, 1, 0, 0]):
            self.robot.vr += speed
        elif np.array_equal(action, [0, 0, 0, 1, 0]):
            self.robot.vr -= speed
        # print(self.robot.vl, self.robot.vr)

    def _move2(self, action):
        # return the modified velocity of the robot
        speed = 0.1
        if np.array_equal(action, [1, 0, 0, 0, 0]):
            return [speed, 0]
        elif np.array_equal(action, [0, 1, 0, 0, 0]):
            return [-speed, 0]
        elif np.array_equal(action, [0, 0, 1, 0, 0]):
            return [0, speed]
        elif np.array_equal(action, [0, 0, 0, 1, 0]):
            return [0, -speed]
        else:
            return [0, 0]

        # print(self.robot.vl, self.robot.vr)

    def _update_ui(self):
        # not sure if this is needed
        pass


if __name__ == "__main__":
    game = RobotGame()
    for i in range(3000):
        pygame.display.update()
