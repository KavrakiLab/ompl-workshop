from functools import partial

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from ompl import base as ob
from ompl import geometric as og


# ----------------------------
# Image map
# ----------------------------
class ImageEnv:
    def __init__(self, filename):
        img = Image.open(filename).convert("L")  # grayscale
        self.data = np.array(img)

        # obstacle = 1, free = 0
        self.occ = self.data < 128
        self.occ = np.flipud(self.occ)

        self.height, self.width = self.occ.shape

    def is_free(self, x, y):
        # Continuous to pixel coordinates
        px = int(x)
        py = int(y)

        # Check bounds
        if px < 0 or px >= self.width or py < 0 or py >= self.height:
            return False

        return not self.occ[py, px]


# ----------------------------
# Visualization
# ----------------------------
def plot_result(path, img_map):
    plt.figure(figsize=(6, 6))

    # Show map
    plt.imshow(img_map.occ, cmap="gray_r", origin="lower")

    # Plot path
    if path is not None:
        xs, ys = path
        plt.plot(xs, ys, linewidth=2)

        # Start / goal markers
        plt.scatter(xs[0], ys[0], s=50)
        plt.scatter(xs[-1], ys[-1], s=50)

    plt.title("OMPL Planning on Image")
    plt.xlim(0, img_map.width)
    plt.ylim(0, img_map.height)
    plt.gca().set_aspect('equal')

    plt.show()


# ----------------------------
# Planning
# ----------------------------
def plan(img_env, start, goal):
    # TODO Return path
    # xs, ys = ..., ...
    # return (xs, ys)
    return None


if __name__ == "__main__":
    # Load map
    img_env = ImageEnv("map.png")
    start = (img_env.width- 5, img_env.height - 5, 0)
    goal = (5, 5, 0)
    # Plan
    path = plan(img_env, start, goal)
    # Visualize
    plot_result(path, img_env)
