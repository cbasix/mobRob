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