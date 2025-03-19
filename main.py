from robot import Robot
from factory import MazeSolverFactory

enable_pins = [17, 16]
motor_pins = [19, 18, 5, 4]
sensor_pins = [26, 27, 32]

robot = Robot(enable_pins, motor_pins, sensor_pins)

selected_algorithm = "dfs"

solver = MazeSolverFactory.get_solver(selected_algorithm)
if solver:
    robot.execute_algorithm(solver)
else:
    print("Wrong Algorithm!")








































































# from time import sleep
#
# from MotorControl import MotorControl
#
# enable_pins = [17, 16]
# motor_pins = [19, 18, 5, 4]
#
# robot = MotorControl(enable_pins, motor_pins, speed=40000)
#
# robot.forward()
# sleep(2)
#
# robot.turnRight()
# sleep(1)
#
# robot.turnLeft()
# sleep(1)
#
# robot.turnBack()
# sleep(2)
#
# robot.stop()
