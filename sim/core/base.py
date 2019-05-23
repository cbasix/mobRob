import os
from tkinter import filedialog

import dill
import numpy as np

from sim.core.vector import Vector2D

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

    def save(self, destination=None):
        from sim.core.obstacles import Obstacle

        # save only obstacles (else the robot and all its code, strategies etc. would be saved too, which we dont want)
        self.things = [i for i in self.things if isinstance(i, Obstacle)]

        if destination is None:
            destination = filedialog.asksaveasfilename()

        if destination:
            with open(destination, 'wb') as output_file:
                dill.dump(self, output_file)

    @staticmethod
    def loadFrom(world=None):
        if world:
            input_file_path = os.path.join("worlds", world)
        else:
            input_file_path = filedialog.askopenfilename()

        if input_file_path:
            with open(input_file_path, 'rb') as input_file:
                return dill.load(input_file)

        raise ValueError("No valid world file selected")


class Strategy(object):
    def do(self, robot, sensor_data):
        pass


class CircularThing(object):
    def __init__(self, position=Vector2D(0, 0), radius=0):
        self.position = position
        self.radius = radius

    def simulate(self, simulation):
        pass


class Sensor(object):
    def __init__(self, name, offset):
        self.name = name
        self.robot = None
        self.offset = offset

    def get_position(self):
        # rotate sensor around robot center before adding it's pos
        return self.robot.position.data + self.offset.get_rotated(self.robot.angle)

    def measure(self, world):
        pass


class Marker(CircularThing):
    def __init__(self, position=Vector2D(0, 0), radius=0):
        super().__init__(position, radius)
        self.type = "marker"


