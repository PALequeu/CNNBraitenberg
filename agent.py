from math import inf, atan2, radians
import torch
import random
import numpy as np

# deque is a list optimized for append and pop operations
from collections import deque
from robot import MAXSPEED
from model import Linear_QNet, QTrainer
from game import RobotGame
import matplotlib.pyplot as plt
from IPython import display
from torchviz import make_dot


MAX_MEMORY = 100_000
BATCH_SIZE = 1000
LR = 0.01

plt.ion()


def plot(scores, mean_scores):
    display.clear_output(wait=True)
    display.display(plt.gcf())
    plt.clf()
    plt.title("Training...")
    plt.xlabel("Number of games")
    plt.ylabel("Score")
    plt.plot(scores)
    plt.plot(mean_scores)
    plt.ylim(ymin=0)
    plt.text(len(scores) - 1, scores[-1], str(scores[-1]))
    plt.text(len(mean_scores) - 1, mean_scores[-1], str(mean_scores[-1]))
    plt.pause(0.001)


def distance(p1, p2):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


class Agent:
    def __init__(self):
        self.n_games = 0  # number of games played
        self.epsilon = 0  # randomness
        self.gamma = 0.95  # discount rate, must be < 1
        # deque : popleft() if len(memory) > MAX_MEMORY
        self.memory = deque(maxlen=MAX_MEMORY)
        # TODO
        self.model = Linear_QNet(5, 30, 5)
        self.trainer = QTrainer(self.model, lr=LR, gamma=self.gamma)

    def get_state(self, game):
        # state = [robot left speed, robot right speed, robot heading, target left, target right, target up, target down]
        robot = game.robot
        # print(robot.heading)
        robot_target_angle = (
            atan2(game.target.y - robot.y, game.target.x - robot.x) - robot.heading
        )

        state = [
            robot.x / game.w,
            robot.y / game.h,
            robot.heading / (2 * np.pi),
            robot_target_angle / (2 * np.pi),
            distance(robot, game.target) / (game.w + game.h),
        ]

        # print(state)

        # print(np.array(state, dtype=float))
        return np.array(state, dtype=float)

    def remember(self, state, action, reward, next_state, done):
        # TODO
        self.memory.append((state, action, reward, next_state, done))

    def train_long_memory(self):
        if len(self.memory) > BATCH_SIZE:
            # get random sample from memory
            mini_sample = random.sample(self.memory, BATCH_SIZE)
        else:
            mini_sample = self.memory

        states, actions, rewards, next_states, dones = zip(*mini_sample)
        # print(actions)

        self.trainer.train_step(states, actions, rewards, next_states, dones)

    def train_short_memory(self, state, action, reward, next_state, done):
        self.trainer.train_step(state, action, reward, next_state, done)

    def get_action(self, state):
        self.epsilon = 80 - self.n_games
        # accelerate left, decelerate left, accelerate right, decelerate right, do nothing
        final_move = [0, 0, 0, 0, 0]
        if random.randint(0, 100) < self.epsilon:
            idx = random.randint(0, 4)
            final_move[idx] = 1
        else:
            state0 = torch.tensor(state, dtype=torch.float)
            prediction = self.model(state0)
            move = torch.argmax(prediction).item()
            final_move[move] = 1

        return final_move


def train():
    plot_scores = []
    plot_mean_scores = []
    total_score = 0
    record = -inf
    agent = Agent()
    game = RobotGame()
    last_distance = 0
    while True:
        # get old state
        state_old = agent.get_state(game)

        # get move
        final_move = agent.get_action(state_old)

        # perform move and get new state
        reward, done, score, last_distance = game.play_step(final_move, last_distance)

        state_new = agent.get_state(game)

        # train short memory
        agent.train_short_memory(state_old, final_move, reward, state_new, done)

        # remember
        agent.remember(state_old, final_move, reward, state_new, done)

        # print("test 1")
        if done:
            # train long memory/experience replay
            # it trains on all the previous games done
            game.reset()

            agent.n_games += 1
            agent.train_long_memory()

            if score > record:
                record = score
                agent.model.save()

            print(
                "Game",
                agent.n_games,
                "Score",
                score,
                "reward",
                reward,
                "Record",
                record,
            )
            # print("test 4")

            # plot
            plot_scores.append(score)
            total_score += score
            mean_score = total_score / agent.n_games
            plot_mean_scores.append(mean_score)
            plot(plot_scores, plot_mean_scores)


def start():
    agent = Agent()
    game = RobotGame()
    sensor_range = 200, radians(60)
    # ultrasonic = UltraSonic(sensor_range, game.map)
    while True:
        # get state
        state = agent.get_state(game)

        # get move
        final_move = agent.get_action(state)
        done = game.play_step2(final_move)

        if done:
            game.reset()
            agent.n_games += 1


if __name__ == "__main__":
    # Si l'on veut entainer le robot
    # train()

    # si l'on veut tester le robot
    start()
    print("oui")
    pass
