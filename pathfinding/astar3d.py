#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D space path planning by A* algorithm
"""

import numpy as np
import heapq
# functools.total_ordering is used for comparing objects by __eq__ and __lt__
from functools import total_ordering
import itertools

@total_ordering
class Node(object):
    """
    node class for open set as priority queue
    """
    def __init__(self, pos, cost=0):
        self.pos = pos
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.cost == other.cost

def setDemoEnvironment(width, height, depth):
    """
    Return 3d-array for 3d environment where random obstacles are
    """
    env = np.zeros([width, height, depth], dtype=np.int8)
    for w in range(width):
        for h in range(height):
            for d in range(depth):
                r = np.random.randint(0, 99)
                if r < 30: # obstacle, 30%
                    env[w][h][d] = 1
    return env

class Astar3D(object):
    """
    class である必要がない
    opn_cls_ntyt_map:ある[x,y,z]のnodeがopen:1, closed:2, notyet:0のどれかを示す
    parent_map:ある[x,y,z]のnodeがどの隣接nodeを親とするかを示す
    priority_queue:open listのnodeを評価値に基づいてpick upするため
    """
    def __init__(self, env_map, start, goal):
        self.env_map = env_map
        self.start = start
        self.goal = goal
        self.open_queue = []

    def __push(self, node):
        heapq.heappush(self.open_queue, node)

    def __pop(self):
        return heapq.heappop(self.open_queue)

    def __h(self, node):
        return np.int16(np.linalg.norm(np.array(self.goal)-np.array(node.pos))*10)

    def __g(self, node):
        return np.int16(node.cost - self.__h(node))

    def __c(self, node, next_node):
        return np.int16(np.linalg.norm(node.pos-next_node.pos))

    def __constructPath(self, goal_pos):
        # return path by following parents
        dir_dict = {}
        # direction[-1, -1, -1] is No.1, dir[0, -1, -1] is No 2, dir[1, 1, 1] is No.26
        num = 1
        for z in range(-1, 2):
            for y in range(-1, 2):
                for x in range(-1, 2):
                    if x==0 and y==0 and z==0:
                        dir_dict[0] = np.array([x, y, z])
                        continue
                    dir_dict[num] = np.array([x, y, z])
                    num += 1
        p = np.array(goal_pos)
        dir_No = self.parent_map[p[0]][p[1]][p[2]]
        path = []
        while dir_No != 0:
            n = p
            path.append(n)
            dir_No = self.parent_map[p[0]][p[1]][p[2]]
            p = n + dir_dict[dir_No]

        # reverse backpointer's path
        path.reverse()

        # return a sequence of positions as the path
        return path

    def __succeedNodes(self, pos):
        # return nodes that are adjacent to the position
        nodes = []
        for z in range(-1, 2):
            for y in range(-1, 2):
                for x in range(-1, 2):
                    if x == 0 and y == 0 and z == 0:
                        continue
                    next_p = pos + np.array([x, y, z])
                    env_s = self.env_map.shape
                    # if next position is out of environment,
                    if next_p[0] < 0 or env_s[0] <= next_p[0] or \
                       next_p[1] < 0 or env_s[1] <= next_p[1] or \
                       next_p[2] < 0 or env_s[2] <= next_p[2]:
                        continue
                    else:
                        n = Node(next_p, 0) # cost will be reset after
                        nodes.append(n)
        return nodes

    def __setParent(self, mother_node, daughter_node):
        """
        Set parent relationship to self.parent_map.
        26 directions are 3D Voxel is decoded as integer from 1 to 26.
        0 means root node.(__setParent(nodeA, nodeA) == 0)
        """
        diff = mother_node.pos - np.array(daughter_node.pos)
        p = daughter_node.pos
        if (diff==np.zeros(diff.shape)).all():
            self.parent_map[p[0]][p[1]][p[2]] = 0
            return
        # direction[-1, -1, -1] is No.1, dir[0, -1, -1] is No 2, dir[1, 1, 1] is No.26
        num = 1
        for z in range(-1, 2):
            for y in range(-1, 2):
                for x in range(-1, 2):
                    if x==0 and y==0 and z==0:
                        continue
                    if (diff == np.array([x, y, z])).all():
                        self.parent_map[p[0]][p[1]][p[2]] = num
                        return
                    num += 1

        #return -1 # mother and daughter is not adjacent

    def solve(self):
        self.opn_cls_ntyt_map = np.zeros(self.env_map.shape, dtype=np.int16)
        self.cost_map = np.zeros(self.env_map.shape, dtype=np.int16)
        self.parent_map = np.zeros(self.env_map.shape, dtype=np.int16)
        self.open_queue = []
        n = Node(self.start)
        n.cost = self.__h(n)
        self.__push(n)
        self.opn_cls_ntyt_map[n.pos[0]][n.pos[1]][n.pos[2]] = 1 #open
        self.cost_map[n.pos[0]][n.pos[1]][n.pos[2]] = n.cost
        self.__setParent(n, n)

        while len(self.open_queue) != 0:
            m = self.__pop() # select node with minimum cost from open list
            if (np.array(m.pos) == np.array(self.goal)).all():
                return self.__constructPath(m.pos)

            self.opn_cls_ntyt_map[m.pos[0]][m.pos[1]][m.pos[2]] = 2 #close

            for n in self.__succeedNodes(m.pos):
                f_n_m = self.__g(m) + self.__c(m, n) + self.__h(n)
                # n is not in open and closed.
                if self.opn_cls_ntyt_map[n.pos[0]][n.pos[1]][n.pos[2]] == 0:
                    # if n is obstacle...
                    self.opn_cls_ntyt_map[n.pos[0]][n.pos[1]][n.pos[2]] = 1 #open
                    self.cost_map[n.pos[0]][n.pos[1]][n.pos[2]] = f_n_m
                    n.cost = f_n_m
                    self.__push(n)
                    self.__setParent(m, n)
                # n is in open or closed list.
                elif f_n_m < self.cost_map[n.pos[0]][n.pos[1]][n.pos[2]]:
                    self.cost_map[n.pos[0]][n.pos[1]][n.pos[2]] = f_n_m
                    n.cost = f_n_m
                    self.__setParent(m, n)
                    #if in open list, update the cost
                    if self.opn_cls_ntyt_map[n.pos[0]][n.pos[1]][n.pos[2]] == 1:
                        next(itertools.filterfalse(lambda x: not (x.pos==np.array(n.pos)).all(),\
                                self.open_queue), None).cost = n.cost
                    #if in closed, set back it to open list
                    elif self.opn_cls_ntyt_map[n.pos[0]][n.pos[1]][n.pos[2]] == 2:
                        self.opn_cls_ntyt_map[n.pos[0]][n.pos[1]][n.pos[2]] = 1 #open
                        self.__push(n)
                if (n.cost < 0):
                    print("found negative cost")

        return None # the path is not found

def demo():
    env = setDemoEnvironment(100, 100, 100)
    found_demo_position = False
    start = None
    goal = None
    while not found_demo_position:
       start = [np.random.randint(0, 99), np.random.randint(0, 99), np.random.randint(0, 99)]
       goal = [np.random.randint(0, 99), np.random.randint(0, 99), np.random.randint(0, 99)]
       if env[start[0]][start[1]][start[2]] == 0 and env[goal[0]][goal[1]][goal[2]] == 0:
           found_demo_position = True
    print("start", start)
    print("goal", goal)
    solver = Astar3D(env, start, goal)
    path = solver.solve()
    if path == None:
        print("not found")
    else:
        print("path is found")

if __name__ == '__main__':
    demo()
