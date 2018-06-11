#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

class Particle(object):
    # Initial global best is decided after initialization of particles
    gbest = float("inf") # This variable is shared by all instances
    gbest_x = np.array([])
    def __init__(self, x_ini, v_ini, pbest_ini):
        self.x = x_ini
        self.v = v_ini
        self.pbest = pbest_ini
        self.pbest_x = x_ini

    def calc_state(self, value):
        # Update his best
        if self.pbest > value:
            self.pbest_x = self.x

        w = 0.8 # inertia constant
        c1 = 0.9
        c2 = 0.9
        r1 = np.random.rand()
        r2 = np.random.rand()
        self.v = w*self.v + c1*r1*(self.pbest_x - self.x) + c2*r2*(Particle.gbest_x - self.x)
        self.x = self.x + self.v

def evaluateFunction(x):
    # example v=x^Tx, sphere
    # region area: All
    # minimum value: 0
    # minimum state: Zero vector
    #value = np.dot(x, x)

    # example Ackley function
    # region area: -32.768 <= xi <= + 32.768
    # minimum value: 0
    # minimum state: Zero vector
    value = 20 - 20*np.exp(-0.2*np.sqrt(np.sum(x*x)/np.shape(x)[0])) +\
            np.e - np.exp(np.sum(np.cos(2*np.pi*x))/np.shape(x)[0])


    return value

def executePSO(particles, max_iter_num, eva_func):
    for i in range(max_iter_num):
        for p in particles:
            p.calc_state(eva_func(p.x))

        # update global best
        gidx = -1
        for idx, p in enumerate(particles):
            tmp_gbest = Particle.gbest
            if p.pbest < tmp_gbest:
                gidx = idx
                tmp_gbest = p.pbest
        if gidx != -1: # Found new global bet
            Particle.gbest = particles[gidx].pbest
            Particle.gbest_x = particles[gidx].pbest_x

    return

if __name__ == '__main__':
    """
    Test script
    """

    # Initialize Particles
    particle_num = 10000
    particles = []
    ## Set random position, and Find
    for i in range(particle_num):
        x1 = np.random.rand()*100 - 50 # Example of 2d particles
        x2 = np.random.rand()*100 - 50
        x_ini = np.array([x1, x2])
        v_ini = np.array([0, 0]) # Set zero as velocity
        pbest_ini = evaluateFunction(x_ini)
        particles.append(Particle(x_ini, v_ini, pbest_ini))

    ## Update initial global best
    gidx = -1
    for idx, p in enumerate(particles):
        tmp_gbest = Particle.gbest # initial value is set as +inf
        if p.pbest < tmp_gbest:
            gidx = idx
            tmp_gbest = p.pbest
    if gidx != -1: # Found new global bet
        Particle.gbest = particles[gidx].pbest
        Particle.gbest_x = particles[gidx].pbest_x

    # PSO
    max_iter = 200
    executePSO(particles, max_iter, evaluateFunction)

    # Show results
    print("minimum value")
    print Particle.gbest
    print("state")
    print Particle.gbest_x

