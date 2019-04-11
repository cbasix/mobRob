import collections
import sys
import time
from math import pi as PI

import tkinter as tk
import random
from tkinter import filedialog
import dill

import numpy as np
import pygame

from sim.base import Strategy, Simulation
from sim.obstacles import FieldSource, Obstacle
from sim.robots import CircularRobot
from sim.sensors import FieldIntensitySensor, DistanceSensor
from sim.vector import Vector2D
from sim.base import Strategy

black = 0, 0, 0
red = 255, 0, 0
green = 0, 155, 0
grey = 150, 150, 150
white = 255, 255, 255


class AvoidanceTwoStrategy(Strategy):
    def do(self, robot, sensor_data):
        robot.set_speed(robot.max_speed/2)

        #self.sub_strategy.do(robot, sensor_data)
        angle_change = -PI/16 if sensor_data["dist_left"] > sensor_data["dist_right"] else PI/16

        robot.change_angle(angle_change)


class BraitenbergStrategy(Strategy):
    def __init__(self, robot_type=1):
        self.robot_type = robot_type

    def do(self, robot, sensor_data):
        print(robot, sensor_data)

        left_engine = min(sensor_data["left"] / 5000, 1)
        right_engine = min(sensor_data["right"] / 5000, 1)
        max_engine = max(left_engine, right_engine)

        wanted_speed = left_engine + right_engine
        if max_engine != 0:
            if self.robot_type == 1:
                angle_change = (left_engine/max_engine)*PI - (right_engine/max_engine)*PI
            else:
                angle_change = (right_engine/max_engine)*PI - (left_engine/max_engine)*PI
        else:
            angle_change = 0

        robot.change_angle(angle_change)
        robot.set_speed(wanted_speed)


class AvoidanceStrategy(Strategy):
    def __init__(self):
        self.sub_strategy = BraitenbergStrategy(1)
        self.last_seen = collections.deque(maxlen=800)

    def do(self, robot, sensor_data):
        robot.set_speed(robot.max_speed/2)

        #self.sub_strategy.do(robot, sensor_data)
        angle_change = 0 if sensor_data["dist_front"] > 10 else PI/4

        seen = self.last_seen.count(int(sensor_data["dist_front"]))
        if seen > 15:
            angle_change = np.deg2rad(random.randint(0, 360))
        print("Seen: ", seen)
        #print(self.last_seen)

        self.last_seen.append(int(sensor_data["dist_front"]))

        robot.change_angle(angle_change)


# all angles are in radians !
def main():
    simulation, wall_radius = build_scenario()
    simulation = load("world.sim")
    simulation.things.append(robotC())
    # pygame stuff

    zoom = 2
    screen = pygame.display.set_mode(simulation.size*zoom)
    pause = True
    root = tk.Tk()
    root.withdraw()

    try:
        while True:
            if not pause:
                simulation.step()

            #print("Speed: ", robot.speed)
            #print("Angle: ", np.rad2deg(robot.angle))
            #print("Position:", robot.position.data)

            screen.fill(black)

            for thing in simulation.things:
                try:
                    type = thing.type
                except AttributeError:
                    continue

                if type == "robot":
                    pygame.draw.circle(screen, red, thing.position.data.astype(int)*zoom, thing.radius*zoom, 2*zoom)
                    pygame.draw.line(
                        screen,
                        green,
                        thing.position.data.astype(int) * zoom,
                        (thing.position.data + Vector2D(thing.radius, 0).get_rotated(thing.angle)).astype(int) * zoom
                    )

                    for sensor in thing.sensors:
                        pygame.draw.circle(screen, white, sensor.get_position().astype(int)*zoom, 1*zoom, 1)

                elif type == "field_source":
                    pygame.draw.circle(screen, green, thing.position.data.astype(int)*zoom, thing.radius*zoom, 2*zoom)

                elif type == "obstacle":
                    pygame.draw.circle(screen, grey, thing.position.data.astype(int) * zoom, thing.radius * zoom, 2 * zoom)

            pygame.display.flip()

            time.sleep(0.01)

            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()

                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    simulation.things.append(
                        Obstacle(radius=wall_radius, position=Vector2D(pos[0]/zoom, pos[1]/zoom)))

                if event.type == pygame.KEYUP and event.key == pygame.K_p:
                    pause = not pause

                if event.type == pygame.KEYUP and event.key == pygame.K_s:
                    # save simulation
                    save(simulation)

                if event.type == pygame.KEYUP and event.key == pygame.K_l:
                    # load simulation
                    simulation = load()

    except KeyboardInterrupt:
        print("Stopped")


def update_scenario():
    s = load()
    #  get objects only
    s.things = [i for i in s.things if isinstance(i, Obstacle)]
    s.things.append(robotC())
    save(s)


def robotC():
    # Build ROBOT C
    robotC = CircularRobot(AvoidanceStrategy(), radius=5, position=Vector2D(200, 100), name="C")
    robotC.attach_sensor(FieldIntensitySensor("left", field_type="light", offset=Vector2D(3, 3)))
    robotC.attach_sensor(FieldIntensitySensor("right", field_type="light", offset=Vector2D(3, -3)))
    robotC.attach_sensor(
        DistanceSensor("dist_front", offset=Vector2D(3, 0), angle_offset=0, field_of_view_angle=np.deg2rad(15)))

    return robotC

def build_scenario():
    simulation = Simulation(np.array([800, 500]))
    # Build ROBOT A
    robotA = CircularRobot(BraitenbergStrategy(robot_type=0), radius=5, position=Vector2D(120, 140), name="A")
    robotA.attach_sensor(FieldIntensitySensor("left", field_type="light", offset=Vector2D(3, 3)))
    robotA.attach_sensor(FieldIntensitySensor("right", field_type="light", offset=Vector2D(3, -3)))
    # Build ROBOT B
    robotB = CircularRobot(BraitenbergStrategy(robot_type=1), radius=5, position=Vector2D(240, 100), name="B")
    robotB.attach_sensor(FieldIntensitySensor("left", field_type="light", offset=Vector2D(3, 3)))
    robotB.attach_sensor(FieldIntensitySensor("right", field_type="light", offset=Vector2D(3, -3)))
    robotB.attach_sensor(
        DistanceSensor("dist_front", offset=Vector2D(3, 0), angle_offset=0, field_of_view_angle=np.deg2rad(25)))

    # Bulid ROBOT D
    robotD = CircularRobot(AvoidanceTwoStrategy(), radius=5, position=Vector2D(200, 100), name="D")
    robotD.attach_sensor(
        DistanceSensor("dist_left", offset=Vector2D(3, -3), angle_offset=-15, field_of_view_angle=np.deg2rad(15)))
    robotD.attach_sensor(
        DistanceSensor("dist_right", offset=Vector2D(3, 3), angle_offset=15, field_of_view_angle=np.deg2rad(15)))
    # Add robots to the simulation
    # simulation.things.append(robotA)
    # simulation.things.append(robotB)
    simulation.things.append(robotC())
    # simulation.things.append(robotD)
    # Add three light sources to the simulation
    simulation.things.append(FieldSource(5,
                                         "light",
                                         position=Vector2D(80, 80),
                                         function=lambda x: -0.05 * x * x + 1000 if -0.05 * x * x + 1000 > 0 else 0))
    simulation.things.append(FieldSource(2,
                                         "light",
                                         position=Vector2D(240, 230),
                                         function=lambda x: -5 * x + 1000 if -5 * x + 1000 > 0 else 0))
    simulation.things.append(FieldSource(2,
                                         "light",
                                         position=Vector2D(240, 20),
                                         function=lambda x: -5 * x + 3000 if -5 * x + 3000 > 0 else 0))
    # walls, yes circular walls...
    wall_radius = 10
    for x in range(0, simulation.size[0] + wall_radius, wall_radius):
        for y in (0, simulation.size[1]):
            simulation.things.append(
                Obstacle(radius=wall_radius, position=Vector2D(x, y)))  # top and bottom wall
    for y in range(0, simulation.size[1] + wall_radius, wall_radius):
        for x in (0, simulation.size[0]):
            simulation.things.append(
                Obstacle(radius=wall_radius, position=Vector2D(x, y)))  # left and right wall
    return simulation, wall_radius


def save(simulation):

    # save only obstacles
    simulation.things = [i for i in simulation.things if isinstance(i, Obstacle)]

    output_file_path = filedialog.asksaveasfilename()
    if output_file_path:
        with open(output_file_path, 'wb') as output_file:
            dill.dump(simulation, output_file)


def load(world=None):
    if world:
        input_file_path = world
    else:
        input_file_path = filedialog.askopenfilename()

    if input_file_path:
        with open(input_file_path, 'rb') as input_file:
            return dill.load(input_file)

if __name__ == '__main__':
    #update_scenario()
    main()