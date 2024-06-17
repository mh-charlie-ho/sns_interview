import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sns_msgs.msg import RobotState
from sns_msgs.srv import RobotAction
from sensor_msgs.msg import JointState

from .monitor_plot import MonitorPlot
import threading

import random
import time

ROBOTXUPPER = 100.0
ROBOTXLOWER = -100.0

ROBOTYUPPER = 100.0
ROBOTYLOWER = -100.0

METER2DEGREE = 1000  # 1mm = 1 degree

# global variable for update plot
pos1 = 0
pos2 = 0


class ArmRobot(Node):

    def __init__(self):
        super().__init__("arm_robot")
        g1 = MutuallyExclusiveCallbackGroup()
        g2 = MutuallyExclusiveCallbackGroup()
        g3 = MutuallyExclusiveCallbackGroup()
        g4 = MutuallyExclusiveCallbackGroup()

        self.botCmdPub = self.create_subscription(JointState,
                                                  "joint_cmd",
                                                  self.setBotPos,
                                                  10,
                                                  callback_group=g1)
        self.botState = self.create_publisher(JointState,
                                              "joint_state",
                                              10,
                                              callback_group=g2)
        self.create_timer(0.5, self.pubRobotState, callback_group=g2)

        self.create_timer(0.5, self.updateX, callback_group=g3)
        self.create_timer(0.5, self.updateY, callback_group=g4)

        self.tRobotPos = [0, 0]  # rad
        self.RobotPos = [0, 0]  # rad

    def setBotPos(self, msg):
        self.tRobotPos[0] = msg.position[0]
        self.tRobotPos[1] = msg.position[1]

    def pubRobotState(self):
        global pos1, pos2
        pos1 = self.RobotPos[0]
        pos2 = self.RobotPos[1]
        jointCmd = JointState()
        jointCmd.position.append(pos1)
        jointCmd.position.append(pos2)
        self.botState.publish(jointCmd)
        time.sleep(0.1)

    def updateMotor(self, state, target):
        r = target
        y = state

        p = 0.1
        diff = r - y

        u = p * diff
        y = y + u
        return y

    def updateX(self):
        self.RobotPos[0] = self.updateMotor(self.RobotPos[0],
                                            self.tRobotPos[0])
        time.sleep(0.1)

    def updateY(self):
        self.RobotPos[1] = self.updateMotor(self.RobotPos[1],
                                            self.tRobotPos[1])
        time.sleep(0.1)


def node():
    rclpy.init()

    print("starting robot...")

    arm_robot = ArmRobot()
    executor = MultiThreadedExecutor(5)
    executor.add_node(arm_robot)
    executor.spin()

    rclpy.shutdown()


def updatePlot():
    global pos1, pos2
    while True:
        ani.sendDate(pos1, pos2)
        time.sleep(0.1)


def main():
    global ani
    threading.Thread(target=node).start()
    ani = MonitorPlot()
    threading.Thread(target=updatePlot).start()
    ani.animate()


if __name__ == "__main__":
    main()
