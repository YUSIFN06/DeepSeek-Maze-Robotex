from motor_control import MotorControl
from sensors import Sensors

class Robot:
    def __init__(self, enable_pins, motor_pins, sensor_pins):
        self.motors = MotorControl(enable_pins, motor_pins)
        self.sensors = Sensors(sensor_pins)

    def execute_algorithm(self, solver):
        solver.solve(self)

    # def ...
    # .
    # .
    # .