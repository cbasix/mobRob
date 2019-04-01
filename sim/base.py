import numpy as np

from sim.vector import Vector2D

class Simulation(object):
    def __init__(self, size=np.array([800, 600])):
        self.size = size
        self.things = []

    def step(self):
        for thing in self.things:
            thing.simulate(simulation=self)

            # prevent driving out of world
            thing.position.data[0] = np.clip(thing.position.data[0], 0, self.size[0])
            thing.position.data[1] = np.clip(thing.position.data[1], 0, self.size[1])


class Strategy(object):
    def do(self, robot, sensor_data):
        pass


class CircularThing(object):
    def __init__(self, position=Vector2D(0, 0), radius=0):
        self.position = position
        self.radius = radius

    def simulate(self, simulation):
        pass