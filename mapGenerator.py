import numpy as np
import random
from PIL import Image as im
from collections import namedtuple

Point = namedtuple("Point", "x, y")


def place_target(w, h):
    x = random.randint(200, (w - 40))
    y = random.randint(40, (h - 40))
    target = Point(x, y)
    return target


def generate_map(WIDTH, HEIGHT, target, Empty=False):
    # init map
    map = np.zeros((HEIGHT, WIDTH))

    # draw walls
    for i in range(HEIGHT):
        for j in range(WIDTH):
            if not (i < 40 or i > HEIGHT - 40 or j < 40 or j > WIDTH - 40):
                map[i][j] = 255

    print("new target", target)
    # draw target
    for i in range(target.x - 5, target.x + 5):
        for j in range(target.y - 5, target.y + 5):
            map[j][i] = 150

    # draw obstacles
    if not Empty:
        number_of_obstacles = random.randint(5, 8)
        for i in range(number_of_obstacles):
            height = random.randint(40, 250)
            width = random.randint(40, 250)
            x = random.randint(250, WIDTH - width - 1)
            y = random.randint(0, HEIGHT - height - 1)
            for x_id in range(x, x + width):
                for y_id in range(y, y + height):
                    map[y_id][x_id] = 0

    data = im.fromarray(map)
    if data.mode != "RGB":
        data = data.convert("RGB")

    data.save("test_map.png")
    return map


if __name__ == "__main__":
    w = 400
    h = 800
    target = place_target(w, h)
    generate_map(w, h, target)
    # plot(plot_scores, plot_mean_scores)
    pass
