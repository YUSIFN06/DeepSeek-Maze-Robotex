from machine import Pin

class Sensors:
    def __init__(self, sensor_pins):
        self.front = Pin(sensor_pins[0], Pin.IN)
        self.left = Pin(sensor_pins[1], Pin.IN)
        self.right = Pin(sensor_pins[2], Pin.IN)

    def read_sensors(self):
        return {
            "front": self.front.value(),
            "left": self.left.value(),
            "right": self.right.value()
        }
