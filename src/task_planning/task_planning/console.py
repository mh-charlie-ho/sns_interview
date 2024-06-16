import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sns_msgs.srv import ConveyorAction, RobotAction, StoveAction
from sensor_msgs.msg import Temperature, JointState

import time

temperatureData = 0


class Console(Node):

    def __init__(self):
        super().__init__("console")

        g1 = MutuallyExclusiveCallbackGroup()
        g2 = MutuallyExclusiveCallbackGroup()
        g3 = MutuallyExclusiveCallbackGroup()
        g4 = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("ingredient_id", 1)
        self.declare_parameter("robot_position", [0.0, 0.0])
        self.declare_parameter("stove_level", "Max")
        self.declare_parameter("keep_temperature_sec", 10.0)

        self.updateParameters()  # initialization
        self.create_timer(1, self.updateParameters, callback_group=g1)

        self.temperature = 0.0
        self.subscription = self.create_subscription(Temperature,
                                                     "heat_topic",
                                                     self.updateTemperature,
                                                     10,
                                                     callback_group=g2)

        self.concli, self.conreq = self.serverHandshake(
            "conveyor_service", ConveyorAction, g3)
        self.robcli, self.robreq = self.serverHandshake(
            "robot_service", RobotAction, g3)
        self.stocli, self.storeq = self.serverHandshake(
            "stove_service", StoveAction, g3)

        self.create_timer(1, self.doTask, callback_group=g4)

    def updateParameters(self):
        self.ingredientID = int(
            self.get_parameter(
                "ingredient_id").get_parameter_value().integer_value)
        self.robotPos = (self.get_parameter(
            "robot_position").get_parameter_value().double_array_value)
        self.stoveLv = str(
            self.get_parameter(
                "stove_level").get_parameter_value().string_value)
        self.tempSec = float(
            self.get_parameter(
                "keep_temperature_sec").get_parameter_value().double_value)

    def updateTemperature(self, msg):
        self.temperature = msg.temperature

    def serverHandshake(self, serviceName, serviceType, callbackGroup):
        cli = self.create_client(serviceType,
                                 serviceName,
                                 callback_group=callbackGroup)
        while not cli.wait_for_service(timeout_sec=1.0):
            print(f"{serviceName} not available, waiting again...")
        return cli, serviceType.Request()

    def sendRequest(self, cli, req):
        future = cli.call_async(req)
        while not future.done():
            time.sleep(1)
        return future.result()

    def sendToConveyor(self, ingredient_id):
        self.conreq.ingredient_id = ingredient_id
        return self.sendRequest(self.concli, self.conreq)

    def sendToRobot(self, x, y):
        self.robreq.x = float(x)
        self.robreq.y = float(y)
        return self.sendRequest(self.robcli, self.robreq)

    def sendToStove(self, action_name):
        self.storeq.action_name = action_name
        return self.sendRequest(self.stocli, self.storeq)

    def getMsg(self, state, msgForTrue):
        if state:
            print(msgForTrue)

    def doTask(self):
        while True:
            ans = input("Ready?... [y/n]: ").strip().lower()
            if ans == "y":
                break
            elif ans == "n":
                return
            else:
                print("Please enter 'y' or 'n'.")

        print("\nprocessing =================")
        result = self.sendToConveyor(self.ingredientID)
        self.getMsg(result.success, result.state_msg)
        time.sleep(1)

        print(
            f"Robot's position command is ({self.robotPos[0]}, {self.robotPos[1]})."
        )
        result = self.sendToRobot(float(self.robotPos[0]),
                                  float(self.robotPos[1]))
        self.getMsg(result.success, "Robot has arrived")

        print("Turn on the stove.")
        result = self.sendToStove(self.stoveLv)
        self.getMsg(result.success, "Stove is turned on")

        while self.temperature < 200.0:
            time.sleep(0.1)
        time.sleep(self.tempSec)

        print(f"Robot is going home.")
        result = self.sendToRobot(0, 0)
        self.getMsg(result.success, "The robot has arrived home.")
        
        result = self.sendToStove("OFF")
        self.getMsg(result.success, "Stove Off")
        time.sleep(1)
        print("============================\n")


def main():
    rclpy.init()

    print("preparing...")

    console = Console()
    executor = MultiThreadedExecutor(5)
    executor.add_node(console)
    executor.spin()


if __name__ == "__main__":
    main()
