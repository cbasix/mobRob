import numpy as np
from math import pi as PI
import time
import pygame
import sys
import random

# all angles are in radians !

def normalize_angle(angle):
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle

class Vector2D(object):
    # get vector by length and angle
    @staticmethod
    def create_by(length, angle):
        vector = Vector2D(length, 0)
        vector.data = vector.get_rotated(angle)

        return vector

    def __init__(self, x, y, data=None):
        if data is not None:
            self.data = data
        else:
            self.data = np.array([x, y], dtype='float64')

    def get_euclidean_distance(self, vec):
        # calculate euclidean distance between objects
        return np.linalg.norm(self.data - vec)

    # get around vector base rotated copy
    def get_rotated(self, angle):
        # create rotation matrix for robot angle
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])

        return R.dot(self.data)

    def unit_vector(self):
        if self.data[0] == 0 and self.data[1] == 0:
            return None
        """ Returns the unit vector of the vector.  """
        return self.data / np.linalg.norm(self.data)

    def angle_between(self, v2):
        """
        compute angle (in degrees) for p0p1p2 corner
        Inputs:
            p0,p1,p2 - points in the form of [x,y]
        """

        v0 = v2.data
        v1 = self.data

        angle = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
        return angle

        # v1_u = self.unit_vector()
        # v2_u = v2.unit_vector()
        # if v2_u is None or v1_u is None:
        #    return 0
        # if np.isnan(v1_u).any() or np.isnan(v2_u).any():
        #    return 0  # return an angle of zero if at least one zerovector
        # return normalize_angle(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

    # get vector angle in radians
    def get_angle(self):
        return self.angle_between(Vector2D(1, 0))

    # get norm of the vector
    def get_length(self):
        return np.linalg.norm(self.data)

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


class FieldIntensitySensor(Sensor):
    def __init__(self, name, field_type, offset=Vector2D(0, 0)):
        super().__init__(name=name, offset=offset)
        self.field_type = field_type

    def measure(self, world):
        value = 0
        for field_source in world.things:
            try:
                source_field_type = field_source.field_type
            except AttributeError:
                # has no field type, is of no interest to us
                continue

            if source_field_type == self.field_type:
                distance = field_source.position.get_euclidean_distance(self.get_position())
                intensity = field_source.function(distance)
                value += intensity


        return value


class DirectedSensor(Sensor):
    def __init__(self, name, offset, angle_offset, field_of_view_angle):
        super().__init__(name=name, offset=offset)
        self.angle_offset = angle_offset
        self.field_of_view_angle = field_of_view_angle

    def is_in_field_of_view(self, thing):
        angle_added_by_radius = np.tan((thing.radius/2) / thing.position.get_euclidean_distance(self.get_position()))

        view_direction = normalize_angle(self.angle_offset + self.robot.angle)
        vec_to_thing = Vector2D(x=None, y=None, data=thing.position.data - self.get_position())
        angle_between = normalize_angle(view_direction - vec_to_thing.get_angle())
        angle_between = min(angle_between, 2*PI - angle_between)

        # todo handle negative angle betweens (over zero distance
        return abs(angle_between) <= self.field_of_view_angle + angle_added_by_radius

    def measure(self, simulation):
        pass



class DistanceSensor(DirectedSensor):
    def __init__(self, name, offset, angle_offset, field_of_view_angle):
        super().__init__(name=name, offset=offset, angle_offset=angle_offset, field_of_view_angle=field_of_view_angle)

    def measure(self, world):
        value = None
        for thing in world.things:
            try:
                if thing.type == "robot":
                    continue
                radius = thing.radius
            except AttributeError:
                # has no field type, is of no interest to us
                continue

            if self.is_in_field_of_view(thing):
                distance = thing.position.get_euclidean_distance(self.get_position()) - radius
                if value is None or distance < value:
                    value = distance

        return value


class CircularThing(object):
    def __init__(self, position=Vector2D(0, 0), radius=0, image=None):
        self.position = position
        self.radius = radius
        self.image = image

    def simulate(self, simulation):
        pass


class FieldSource(CircularThing):
    def __init__(self, radius, field_type, position=np.array([0, 0]), function=lambda x: -(x*x)*0.01 + 1000 if -(x*x)*0.01 + 1000 > 0 else 0):
        super().__init__(radius=radius, position=position)
        self.field_type = field_type
        self.function = function
        self.type = "field_source"


class CircularRobot(CircularThing):
    def __init__(self, strategy, position=Vector2D(0,0), angle=0, radius=5):
        super().__init__(position=position, radius = radius)
        self.angle = angle
        self.sensors = []
        self.strategy = strategy
        self.speed = 0
        self.max_speed = 3
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
        if random.randint(0,50) == 0:
            self.position.data += np.random.randint(-1, 1, (2,))
        if random.randint(0, 50) == 0:
            self.angle += np.deg2rad(random.randint(-1, 1))


class BraitenbergStrategy(Strategy):
    def __init__(self, robot_type = 1):
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


def main():

    simulation = Simulation(np.array([360, 240]))


    #robotA = CircularRobot(BraitenbergStrategy(robot_type=0), radius=5, position=Vector2D(60, 40))
    #robotA.attach_sensor(FieldIntensitySensor("left", field_type="light", offset=Vector2D(3, 3)))
    #robotA.attach_sensor(FieldIntensitySensor("right", field_type="light", offset=Vector2D(3, -3)))

    robotB = CircularRobot(BraitenbergStrategy(robot_type=1), radius=5, position=Vector2D(100, 100))
    robotB.attach_sensor(FieldIntensitySensor("left", field_type="light", offset=Vector2D(3, 3)))
    robotB.attach_sensor(FieldIntensitySensor("right", field_type="light", offset=Vector2D(3, -3)))
    robotB.attach_sensor(DistanceSensor("dist_front",  offset=Vector2D(3, 0), angle_offset=0, field_of_view_angle=np.deg2rad(25)))

    #simulation.things.append(robotA)
    simulation.things.append(robotB)

    simulation.things.append(FieldSource(5,
                                         "light",
                                         position=Vector2D(80, 80),
                                         function=lambda x: -5*x + 1000 if -10*x + 10000 > 0 else 0))
    simulation.things.append(FieldSource(2,
                                         "light",
                                         position=Vector2D(240, 230),
                                         function=lambda x: -5 * x + 1000 if -10 * x + 1000 > 0 else 0))
    simulation.things.append(FieldSource(2,
                                         "light",
                                         position=Vector2D(240, 20),
                                         function=lambda x: -5 * x + 3000 if -10 * x + 1000 > 0 else 0))

    # pygame stuff
    zoom = 2
    screen = pygame.display.set_mode(simulation.size*zoom)

    black = 0, 0, 0
    red = 255, 0, 0
    green = 0, 155, 0


    try:
        while True:
            simulation.step()

            #print("Speed: ", robot.speed)
            #print("Angle: ", np.rad2deg(robot.angle))
            #print("Position:", robot.position.data)

            screen.fill(black)

            for thing in simulation.things:
                if thing.type == "robot":
                    pygame.draw.circle(screen, red, thing.position.data.astype(int)*zoom, thing.radius*zoom, 2*zoom)
                    pygame.draw.line(
                        screen,
                        green,
                        thing.position.data.astype(int)*zoom,
                        (thing.position.data + Vector2D(thing.radius, 0).get_rotated(thing.angle)).astype(int)*zoom
                    )

                    for sensor in thing.sensors:
                        pygame.draw.circle(screen, green, sensor.get_position().astype(int)*zoom, 1*zoom, 1)


                elif thing.type == "field_source":
                    pygame.draw.circle(screen, green, thing.position.data.astype(int)*zoom, thing.radius*zoom, 2*zoom)

            pygame.display.flip()

            time.sleep(0.5)

            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()

    except KeyboardInterrupt:
        print("Stopped")



if __name__ == '__main__':
    main()

