import random

import numpy as np

from sim.core.base import CircularThing
from sim.core.vector import Vector2D, normalize_angle


class CircularRobot(CircularThing):
    def __init__(self, strategy, position=Vector2D(0, 0), angle=0, radius=5, name="unknown"):
        super().__init__(position=position, radius=radius)
        self.angle = angle
        self.sensors = []
        self.strategy = strategy
        self.speed = 0
        self.max_speed = 3
        self.name = name
        self.type = "robot"

    def set_speed(self, speed):
        if speed < self.max_speed:
            self.speed = speed
        else:
            self.speed = self.max_speed

    def change_angle(self, angle_diff):
        self.angle = normalize_angle(angle_diff + self.angle)



    def attach_sensor(self, sensor):
        sensor.robot = self
        self.sensors.append(sensor)

    def get_sensor_data(self, simulation):
        sensor_data = {}
        for sensor in self.sensors:
            sensor_data[sensor.name] = sensor.measure(simulation)
        return sensor_data


    def simulate(self, simulation):
        self.strategy.do(robot=self, sensor_data=self.get_sensor_data(simulation))
        self.position.data += Vector2D.create_by(self.speed, self.angle).data
        # put in some unpredictability
        #if random.randint(0,50) == 0:
        #    self.position.data += np.random.randint(-1, 1, (2,))
        #if random.randint(0, 50) == 0:
        #    self.angle += np.deg2rad(random.randint(-1, 1))

    def __str__(self):
        return self.name