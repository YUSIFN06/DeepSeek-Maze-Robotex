from time import sleep

from MotorControl import MotorControl

enable_pins = [17, 16]
motor_pins = [19, 18, 5, 4]

robot = MotorControl(enable_pins, motor_pins, speed=40000)

robot.forward()
sleep(2)

robot.turnRight()
sleep(1)

robot.turnLeft()
sleep(1)

robot.turnBack()
sleep(2)

robot.stop()
