import rclpy
from rclpy.node import Node
from sns_msg.srv import RobotAction

import random
import time

ROBOTXUPPER = 100.0
ROBOTXLOWER = -100.0

ROBOTYUPPER = 100.0
ROBOTYLOWER = -100.0


class RobotServer(Node):
    def __init__(self):
        super().__init__("robot_server")
        self.srv = self.create_service(RobotAction, "robot_service", self.process)

    def waitUntil(self, threshold):
        start = time.time()
        while time.time() - start < threshold:
            time.sleep(0.1)
        return

    def process(self, request, response):

        coordX = request.x
        coordY = request.y

        if not (
            ROBOTXLOWER <= coordX <= ROBOTXUPPER
            and ROBOTYLOWER <= coordY <= ROBOTYUPPER
        ):
            response.success = False
            return response

        self.waitUntil(random.uniform(1, 5))
        response.success = True
        return response


def main():
    rclpy.init()

    print("starting...")

    robot_server = RobotServer()
    rclpy.spin(robot_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
