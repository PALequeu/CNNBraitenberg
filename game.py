import pygame
import random
from mapGenerator import generate_map
from collections import namedtuple
from robot import Robot, Graphics, MAXSPEED
import numpy as np

pygame.init()
font = pygame.font.Font("arial.ttf", 25)

START_POS = (100, 100)
Point = namedtuple("Point", "x, y")

SPEED = 20


def distance(p1, p2):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


class RobotGame:
    def __init__(self, w=1400, h=600, Empty=False):
        self.w = w
        self.h = h
        self.reset()

    def reset(self):
        self.score = 0
        self.target = None
        self._place_target()
        self.robot = Robot(START_POS, self.target)
        self.initial_distance = distance(self.robot, self.target)
        self.frame_iteration = 0
        self.start_ticks = pygame.time.get_ticks()
        self.clock = pygame.time.Clock()
        self.map = generate_map(
            self.w,
            self.h,
            self.target,
        )
        self.gfx = Graphics((self.w, self.h), "robot.png", "test_map.png")
        self.gfx.draw_robot(self.robot.x, self.robot.y, self.robot.heading)

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
        closer_reward = (last_distance - dist) * 10 - 3  # penalize for staying put

        # reward for being close to target
        # close_reward = (self.initial_distance - dist) / self.initial_distance * 10

        # penalize the difference between the left and right wheel velocities
        difference_reward = (
            (abs(self.robot.vl - self.robot.vr)) / self.robot.maxspeed
        ) * 2

        # reward for being faster
        faster_reward = abs((self.robot.vl + self.robot.vr) / 2) / self.robot.maxspeed

        reward += closer_reward - difference_reward + faster_reward
        # print(reward, closer_reward, difference_reward, faster_reward)
        # print(
        #     (1 / dist) * 150,
        #     (abs(self.robot.vl - self.robot.vr)) / self.robot.maxspeed,
        #     (abs((self.robot.vl + self.robot.vr) / 2)) / self.robot.maxspeed,
        # )
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

    def _update_ui(self):
        # not sure if this is needed
        pass


if __name__ == "__main__":
    game = RobotGame()
    for i in range(3000):
        pygame.display.update()
