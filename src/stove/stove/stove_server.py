import rclpy
from rclpy.node import Node

from sns_msgs.srv import StoveAction
from std_msgs.msg import Header
from sensor_msgs.msg import Temperature


class StoveServer(Node):
    def __init__(self):
        super().__init__("stove_server")
        self.srv = self.create_service(StoveAction, "stove_service", self.process)
        self.publisher = self.create_publisher(Temperature, "heat_topic", 10)
        self.create_timer(0.1, self.timer_callback)

        self.actionList = {"OFF": -2.0, "Min": 1.0, "Med": 2.0, "Max": 3.0}
        self.actionName = "OFF"

        self.temperature = 0.0

    def process(self, request, response):
        response.success = request.action_name in self.actionList
        if response.success:
            self.actionName = request.action_name
        return response

    def setTemperature(self, action, temperature):
        temperature = max(0, min(200, temperature + (self.actionList[action])))
        return float(temperature)

    def timer_callback(self):
        self.temperature = self.setTemperature(self.actionName, self.temperature)

        msg = Temperature()
        msg.header = Header()
        msg.temperature = self.temperature
        self.publisher.publish(msg)


def main():
    rclpy.init()
    print("starting stove server...")

    stove_server = StoveServer()
    rclpy.spin(stove_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
