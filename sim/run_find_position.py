import collections
import random

import numpy as np

from sim.core import sim_visualizer
from sim.core.base import Strategy, Simulation, Marker
from math import pi as PI

from sim.core.robots import CircularRobot
from sim.core.sensors import FieldIntensitySensor, DistanceSensor
from sim.core.vector import Vector2D


def main():
    sim = Simulation.loadFrom("world.sim")
    # build a robot
    robot = build_robot(sim)
    # add it to the simulation
    sim.things.append(robot)
    # start visualisation and sim loop
    sim_visualizer.loop(sim)


def build_robot(sim):
    # Build ROBOT C
    robot = CircularRobot(FindPositionStrategy(sim), radius=5, position=Vector2D(230, 200), name="C")

    # add 36 distance sensors
    for angle in range(0, 360, 10):
        robot.attach_sensor(
            DistanceSensor(name=str(angle), offset=Vector2D(0, 0), angle_offset=angle, field_of_view_angle=np.deg2rad(5)))

    return robot


class FindPositionStrategy(Strategy):
    def __init__(self, sim):
        self.last_seen = collections.deque(maxlen=800)
        self.mapdata = None
        self.simulation = sim
        self.marker = Marker(radius=2)
        sim.things.append(self.marker)

    def do(self, robot, sensor_data):

        self.estimate_position(robot, sensor_data)

        robot.set_speed(robot.max_speed/2)

        angle_change = 0 if sensor_data["0"] > 10 else PI/4

        seen = self.last_seen.count(int(sensor_data["0"]))
        if seen > 15:
            angle_change = np.deg2rad(random.randint(0, 360))
        print("Seen: ", seen)

        self.last_seen.append(int(sensor_data["0"]))

        robot.change_angle(angle_change)


    def estimate_position(self, robot, sensor_data):

        # on first run generate or load map data
        import dill
        with open("mapdata.dill", 'rb') as input_file:
            self.mapdata = dill.load(input_file)

        if self.mapdata is None:
            self.mapdata = {}

            # virtually teleport roboter to generate mapdata
            real_position, real_angle = robot.position, robot.angle

            robot.angle = 0

            for x in range(0, self.simulation.size[0], 25):
                for y in range(0, self.simulation.size[1], 25):
                    robot.position = np.array([x, y])
                    sensdata = robot.get_sensor_data(self.simulation)

                    data = []
                    for key, value in sensdata.items():
                        data.append(value)
                    data = sorted(data)

                    self.mapdata[(x, y)] = data
            # save it
            import dill
            with open("mapdata.dill", 'wb') as output_file:
                dill.dump(self.mapdata, output_file)

            # reset robot to real position
            robot.position, robot.angle = real_position, real_angle

        else:

            # build data array for current sensors
            data = []
            for key, value in sensor_data.items():
                data.append(value)
            data = sorted(data)

            min_dist = None
            min_key = None
            for key, value in self.mapdata.items():
                dist = np.linalg.norm(np.array(data) - np.array(value))
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                    min_key = key

            print(min_key)

            # set marker
            self.marker.position = Vector2D(min_key[0], min_key[1])











if __name__ == '__main__':
    main()

