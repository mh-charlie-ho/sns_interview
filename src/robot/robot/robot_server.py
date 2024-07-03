#!/usr/bin/python3
from math import sqrt
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sns_msgs.msg import RobotState
from sns_msgs.srv import RobotAction
from sensor_msgs.msg import JointState

import random
import time

ROBOTXUPPER = 10.0
ROBOTXLOWER = -10.0

ROBOTYUPPER = 10.0
ROBOTYLOWER = -10.0

METER2DEGREE = 1000  # 1mm = 1 degree


class RobotServer(Node):

    def __init__(self):
        super().__init__("robot_server")
        g1 = MutuallyExclusiveCallbackGroup()
        g2 = MutuallyExclusiveCallbackGroup()
        g3 = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(RobotAction,
                                       "robot_service",
                                       self.setBotPos,
                                       callback_group=g1)
        self.botState = self.create_subscription(JointState,
                                                 "joint_state",
                                                 self.getBotPos,
                                                 10,
                                                 callback_group=g2)
        self.botCmdPub = self.create_publisher(JointState,
                                               "joint_cmd",
                                               10,
                                               callback_group=g3)
        self.create_timer(0.5, self.updateBotJoint, callback_group=g3)

        self.tRobotPos = [0, 0]
        self.RobotPos = [0, 0]

    def isInInterval(self, target, min, max):
        if min <= target <= max:
            return True
        return False

    def setBotPos(self, request, response):

        if not (self.isInInterval(request.x, ROBOTXLOWER, ROBOTXUPPER)
                and self.isInInterval(request.y, ROBOTYLOWER, ROBOTYUPPER)):
            response.success = False
            return response

        self.tRobotPos[0] = request.x * METER2DEGREE
        self.tRobotPos[1] = request.y * METER2DEGREE

        while not ((abs(self.RobotPos[0] - self.tRobotPos[0]) < 100) and
                   (abs(self.RobotPos[1] - self.tRobotPos[1]) < 100)):
            time.sleep(0.1)  # block until success

        response.success = True
        return response

    def getBotPos(self, msg):
        self.RobotPos[0] = msg.position[0]
        self.RobotPos[1] = msg.position[1]

    def updateBotJoint(self):
        jointCmd = JointState()
        jointCmd.position.append(self.tRobotPos[0])
        jointCmd.position.append(self.tRobotPos[1])
        self.botCmdPub.publish(jointCmd)
        time.sleep(0.1)


def main():
    rclpy.init()

    print("starting robot server...")

    robot_server = RobotServer()
    executor = MultiThreadedExecutor(3)
    executor.add_node(robot_server)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
