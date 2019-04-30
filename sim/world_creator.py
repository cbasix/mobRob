import numpy as np

from sim.core.base import Simulation
from sim.core.obstacles import Obstacle, FieldSource
from sim.core.robots import CircularRobot
from sim.core.sensors import FieldIntensitySensor, DistanceSensor
from sim.core.vector import Vector2D
from sim.run_avoidance import AvoidanceStrategy, BraitenbergStrategy, AvoidanceTwoStrategy

#def update_scenario():
#    s = load()
#    #  get objects only
#    s.things = [i for i in s.things if isinstance(i, Obstacle)]
#    s.things.append(robotC())
#    save(s)


def main():
    build_empty_world()


# example for building an empty world surrounded by a wall
def build_empty_world():
    simulation = Simulation(np.array([800, 500]))
    add_walls(simulation)
    simulation.save()

# exampe for building a robot
def robotC():
    # Build ROBOT C
    robotC = CircularRobot(AvoidanceStrategy(), radius=5, position=Vector2D(200, 100), name="C")
    robotC.attach_sensor(FieldIntensitySensor("left", field_type="light", offset=Vector2D(3, 3)))
    robotC.attach_sensor(FieldIntensitySensor("right", field_type="light", offset=Vector2D(3, -3)))
    robotC.attach_sensor(
        DistanceSensor("dist_front", offset=Vector2D(3, 0), angle_offset=0, field_of_view_angle=np.deg2rad(15)))

    return robotC

# example for building a complex scenario
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

def add_walls(simulation):
    # walls, yes circular walls...
    wall_radius = 30
    for x in range(0, simulation.size[0] + wall_radius, wall_radius):
        for y in (0, simulation.size[1]):
            simulation.things.append(
                Obstacle(radius=wall_radius, position=Vector2D(x, y)))  # top and bottom wall
    for y in range(0, simulation.size[1] + wall_radius, wall_radius):
        for x in (0, simulation.size[0]):
            simulation.things.append(
                Obstacle(radius=wall_radius, position=Vector2D(x, y)))  # left and right wall


if __name__ == '__main__':
    main()