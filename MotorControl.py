from machine import PWM, Pin, Timer
import machine, time


class MotorControl():

    def __init__(self, enable_pins, motor_pins, speed):
        self.right_motor_enable_pin = PWM(Pin(enable_pins[0]), freq=2000)
        self.left_motor_enable_pin = PWM(Pin(enable_pins[1]), freq=2000)

        self.left_motor_forward = Pin(motor_pins[0], Pin.OUT)
        self.left_motor_backward = Pin(motor_pins[1], Pin.OUT)

        self.right_motor_forward = Pin(motor_pins[2], Pin.OUT)
        self.right_motor_backward = Pin(motor_pins[3], Pin.OUT)

        self.speed = speed
        self.update_speed()

    def update_speed(self):
        self.right_motor_enable_pin.duty_u16(self.speed)
        self.left_motor_enable_pin.duty_u16(self.speed)

    def forward(self):
        self.update_speed()
        self.left_motor_forward.value(1)
        self.left_motor_backward.value(0)
        self.right_motor_forward.value(1)
        self.right_motor_backward.value(0)

    def stop(self):
        self.left_motor_forward.value(0)
        self.left_motor_backward.value(0)
        self.right_motor_forward.value(0)
        self.right_motor_backward.value(0)
        self.right_motor_enable_pin.duty_u16(0)
        self.left_motor_enable_pin.duty_u16(0)

    def turnRight(self):
        self.update_speed()
        self.left_motor_forward.value(0)
        self.left_motor_backward.value(1)
        self.right_motor_forward.value(1)
        self.right_motor_backward.value(0)

    def turnLeft(self):
        self.update_speed()
        self.left_motor_forward.value(1)
        self.left_motor_backward.value(0)
        self.right_motor_forward.value(0)
        self.right_motor_backward.value(1)

    def turnBack(self):
        self.update_speed()
        self.left_motor_forward.value(0)
        self.left_motor_backward.value(1)
        self.right_motor_forward.value(1)
        self.right_motor_backward.value(0)

    def setSpeed(self, new_speed):
        self.speed = new_speed
        self.update_speed()