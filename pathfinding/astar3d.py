#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D space path planning by A* algorithm
"""

import numpy as np

def setEnvironment(width, height, depth):
    env = np.zeros([width, height, depth])
    for w in range(width):
        for h in range(height):
            for d in range(depth):
                r = np.random(randint(0, 99))
                if r < 30: # obstacle, 30%
                    env[w][h][d] = a
    return env

class Astar3D(object):
    def __init__(self, env_map, start, goal):
        self.env_map = env_map
        self.start = start
        self.goal = goal

if __name__ == '__main__':
    pass
