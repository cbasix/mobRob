import numpy as np

from sim.core.base import CircularThing
from sim.core.vector import Vector2D


class Obstacle(CircularThing):
    def __init__(self, position=Vector2D(0, 0), radius=0):
        super().__init__(position=position, radius=radius)
        self.type = "obstacle"


class FieldSource(CircularThing):
    def __init__(self, radius, field_type, position=np.array([0, 0]), function=lambda x: -(x*x)*0.01 + 1000 if -(x*x)*0.01 + 1000 > 0 else 0):
        super().__init__(radius=radius, position=position)
        self.field_type = field_type
        self.function = function
        self.type = "field_source"