#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A-star algorithm
"""

import numpy as np

class Node(object):
    def __init__(self, pose, index):
        self.pose = pose # information of node
        self.index = index

class Edge(object):
    def __init__(self, from_index, to_index, cost):
        self.from_i = from_index
        self.to_i = to_index
        self.cost = cost

class Graph(object):
    def __init__(self, size):
        self.nodes = [None for i in range(size)]
        self.edges = []

def adjacentIndices(index, size):
    """
    Depends on your problem
    """
    adjacent = [0]*(3**3-1)
    i = 0
    for dz in range(-1, 2):
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                if dx == dy == dz == 0:
                    continue
                adjacent[i] = index+dx+dy*size+dz*(size**2)
                i += 1
    return adjacent

def adjacentCost():
    """
    Depends on your problem
    """
    adjacent = [0]*(3**3-1)
    i = 0
    for dz in range(-1, 2):
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                if dx == dy == dz == 0:
                    continue
                adjacent[i] = np.linalg.norm([dx, dy, dz])
                i += 1
    return adjacent

def makeGraph():
    """
    Depends on your problem.
    This example returns 3D space grids.
    (Difficult to prepare array for nodes and edged for large 3D space,
     for example, 1mm grid for 1m x 1m x 1m)
    """

    index = 0
    size = 50 # careful for the number of node
    g = Graph(size**3)
    for z in range(size):
        for y in range(size):
            for x in range(size):
                n = Node([x, y, z], index)
                g.nodes[index] = n
                ad_i = adjacentIndices(index, size)
                ad_c = adjacentCost()
                for j, a_i in enumerate(ad_i):
                    if a_i < 0: # out of index
                        continue
                    e = Edge(index, a_i, ad_c[j])
                    g.edges.append(e)
                index += 1
    return g

def Astar(graph, init, end):

if __name__ == '__main__':
    makeGraph()
    print(adjacentIndices(13, 3)) # test
