import collections
import random

import numpy as np

from sim.core import sim_visualizer
from sim.core.base import Strategy, Simulation
from math import pi as PI

from sim.core.robots import CircularRobot
from sim.core.sensors import FieldIntensitySensor, DistanceSensor
from sim.core.vector import Vector2D


def main():
    sim = Simulation.loadFrom("world.sim")
    # build a robot
    robot = build_robot()
    # add it to the simulation
    sim.things.append(robot)
    # start visualisation and sim loop
    sim_visualizer.loop(sim)


def build_robot():
    # Build ROBOT C
    robot = CircularRobot(AvoidanceStrategy(), radius=5, position=Vector2D(230, 200), name="C")
    robot.attach_sensor(
        DistanceSensor("dist_front", offset=Vector2D(3, 0), angle_offset=0, field_of_view_angle=np.deg2rad(15)))

    return robot


class AvoidanceStrategy(Strategy):
    def __init__(self):
        self.last_seen = collections.deque(maxlen=800)

    def do(self, robot, sensor_data):
        robot.set_speed(robot.max_speed/2)

        angle_change = 0 if sensor_data["dist_front"] > 10 else PI/4

        seen = self.last_seen.count(int(sensor_data["dist_front"]))
        if seen > 15:
            angle_change = np.deg2rad(random.randint(0, 360))
        print("Seen: ", seen)

        self.last_seen.append(int(sensor_data["dist_front"]))

        robot.change_angle(angle_change)


if __name__ == '__main__':
    main()