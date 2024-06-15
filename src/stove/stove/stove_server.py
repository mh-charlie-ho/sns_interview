import rclpy
from rclpy.node import Node

from sns_msgs.srv import StoveAction
from std_msgs.msg import Header
from sensor_msgs.msg import Temperature

import random
import time


class StoveServer(Node):
    def __init__(self):
        super().__init__("stove_server")
        self.srv = self.create_service(StoveAction, "stove_service", self.process)
        self.publisher = self.create_publisher(Temperature, "heat_topic", 10)

        self.actionList = ["OFF", "Min", "Med", "Max"]
        self.actionName = "OFF"

        self.temperature = 0.0

        self.header = Header()
        self.header.frame_id = "heat_sensor"

        self.timer = self.create_timer(0.1, self.timer_callback)

    def SetTemperature(self, action):
        if action == "OFF":
            self.temperature += -2.0
        elif action == "Min":
            self.temperature += 1.0
        elif action == "Med":
            self.temperature += 2.0
        elif action == "Max":
            self.temperature += 3.0

    def limitTemperatureBound(self):
        if self.temperature >= 200.0:
            self.temperature = 200.0
        elif self.temperature <= 0.0:
            self.temperature = 0.0
        else:
            pass

    def process(self, request, response):

        self.actionName = request.action_name
        if not self.actionName in self.actionList:
            response.success = False
            return response

        response.success = True
        return response

    def timer_callback(self):
        self.header.stamp = self.get_clock().now().to_msg()

        msg = Temperature()
        msg.header = self.header
        msg.temperature = self.temperature
        self.publisher.publish(msg)
        self.SetTemperature(self.actionName)
        self.limitTemperatureBound()


def main():
    rclpy.init()

    print("starting stove server...")

    stove_server = StoveServer()
    rclpy.spin(stove_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
