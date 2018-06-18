#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D plane path planning by A* algorithm
"""

import numpy as np

def setEnvironment(width, height):
    env = np.zeros([width, height])
    for w in range(width):
        for h in range(height):
            r = np.random.randint(0, 99)
            if r < 30: # obstacle, 30%
                env[w][h] = 1
    return env

class Astar2D(object):
    def __init__(self, env_map, start, goal):
        self.env_map = env_map
        self.start = start
        self.goal = goal

if __name__ == '__main__':
    print(setEnvironment(30, 50))
