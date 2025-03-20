from MotorControl import MotorControl
import time


enable_pins = [17, 16]
motor_pins = [19, 18, 5, 4]
sensor_pins = [26, 27, 32]

robot = MotorControl(enable_pins, motor_pins, sensor_pins, speed=40000)

def right_hand_rule():
    while True:
        sensors = robot.read_sensors()

        if sensors["right"] == 1:
            robot.turnRight()
            time.sleep(0.5)
        elif sensors["front"] == 1:
            robot.turnLeft()
            time.sleep(0.5)
        else:
            robot.forward()
            time.sleep(0.5)

right_hand_rule()