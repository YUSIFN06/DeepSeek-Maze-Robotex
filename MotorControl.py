from machine import PWM, Pin, Timer
import machine, time


class MotorControl():

    def __init__(self, enable_pins, motor_pins, sensor_pins, speed):
        # Motor PWM pins
        self.right_motor_enable_pin = PWM(Pin(enable_pins[0]), freq=2000)
        self.left_motor_enable_pin = PWM(Pin(enable_pins[1]), freq=2000)

        # Motor direction pins
        self.left_motor_forward = Pin(motor_pins[0], Pin.OUT)
        self.left_motor_backward = Pin(motor_pins[1], Pin.OUT)
        self.right_motor_forward = Pin(motor_pins[2], Pin.OUT)
        self.right_motor_backward = Pin(motor_pins[3], Pin.OUT)

        # IR Sensors
        self.front_sensor = Pin(sensor_pins[0], Pin.IN)
        self.left_sensor = Pin(sensor_pins[1], Pin.IN)
        self.right_sensor = Pin(sensor_pins[2], Pin.IN)

        # Default speed
        self.speed = speed
        self.update_speed()

    def read_sensors(self):
        return {
            "front": self.front_sensor.value(),
            "left": self.left_sensor.value(),
            "right": self.right_sensor.value()
        }

    def update_speed(self, stop = False):
        if stop:
            self.right_motor_enable_pin.duty_u16(0)
            self.left_motor_enable_pin.duty_u16(0)
        else:
            self.right_motor_enable_pin.duty_u16(self.speed)
            self.left_motor_enable_pin.duty_u16(self.speed)

    def move(self, left_dir, right_dir, stop = False):
        self.update_speed(stop)
        self.left_motor_forward.value(left_dir[0])
        self.left_motor_backward.value(left_dir[1])
        self.right_motor_forward.value(right_dir[0])
        self.right_motor_backward.value(right_dir[1])

    def forward(self):
        self.move((1, 0), (1, 0))

    def stop(self):
        self.move((0, 0), (0, 0),True)

    def turnRight(self):
        self.move((0, 1), (1, 0))

    def turnLeft(self):
        self.move((1, 0), (0, 1))

    def turnBack(self):
        self.move((0, 1), (1, 0))

    def setSpeed(self, new_speed):
        self.speed = new_speed
        self.update_speed()