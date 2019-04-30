from math import pi as PI

import numpy as np

from sim.core.base import Sensor
from sim.core.vector import normalize_angle, Vector2D


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
        value = 9999999
        for thing in world.things:
            if thing == self.robot:
                continue

            if self.is_in_field_of_view(thing):
                distance = thing.position.get_euclidean_distance(self.get_position()) - thing.radius
                if value is None or distance < value:
                    value = distance

        return value


