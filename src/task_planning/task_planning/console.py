from numpy import double
from sympy import pde_separate_add
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sns_msg.srv import ConveyorAction, RobotAction, StoveAction
from sensor_msgs.msg import Temperature

import time
import threading


class CustomError(BaseException):

    def __init__(self, message):
        self.message = message


temperatureData = 0


class SensorInfo(Node):

    def __init__(self):
        super().__init__("sensor_info")

        self.subscription = self.create_subscription(Temperature, "heat_topic",
                                                     self.updateTemperature,
                                                     10)
        self.subscription

    def updateTemperature(self, msg):
        global temperatureData
        temperatureData = msg.temperature


class Console(Node):

    def __init__(self):
        super().__init__("console")

        self.concli, self.conreq = self.serverHandshake(
            "conveyor_service", ConveyorAction)
        self.robcli, self.robreq = self.serverHandshake(
            "robot_service", RobotAction)
        self.stocli, self.storeq = self.serverHandshake(
            "stove_service", StoveAction)

        self.declare_parameter("ingredient_id", 1)
        self.declare_parameter("robot_position", [0.0, 0.0])
        self.declare_parameter("stove_level", "Max")
        self.declare_parameter("keep_temperature_sec", 10)

    def serverHandshake(self, serviceName, serviceType):
        cli = self.create_client(serviceType, serviceName)
        while not cli.wait_for_service(timeout_sec=1.0):
            print(serviceName + " not available, waiting again...")
        req = serviceType.Request()
        return cli, req

    def sendRequest(self, cli, req):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
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

    def getMsg(self, state, msgForTrue, msgForFalse):
        if state:
            print(msgForTrue)
        else:
            raise CustomError(msgForFalse)

    def doTask(self):
        global temperatureData
        while True:
            ans = input("ready?...[y/n]")
            if ans == "y":
                pass
            elif ans == "n":
                continue
            else:
                print("please press y or n.")
                continue

            ingredientID = int(
                self.get_parameter(
                    "ingredient_id").get_parameter_value().integer_value)
            robotPos = (self.get_parameter(
                "robot_position").get_parameter_value().double_array_value)
            stoveLv = str(
                self.get_parameter(
                    "stove_level").get_parameter_value().string_value)
            tempSec = float(
                self.get_parameter(
                    "keep_temperature_sec").get_parameter_value().double_value)

            print("\nprocessing =================")
            result = self.sendToConveyor(ingredientID)
            self.getMsg(result.success, result.state_msg, result.state_msg)
            time.sleep(1)

            result = self.sendToRobot(float(robotPos[0]), float(robotPos[1]))
            self.getMsg(result.success, "Robot OK", "Robot not arrived")
            time.sleep(1)

            result = self.sendToStove(stoveLv)
            self.getMsg(result.success, "Stove OK", "Stove fails")
            time.sleep(1)

            while temperatureData <= 200:
                time.sleep(0.1)

                if temperatureData == 200:
                    time.sleep(tempSec)
                    break

            result = self.sendToRobot(0, 0)
            self.getMsg(result.success, "Robot home position",
                        "Robot not arrived")
            time.sleep(1)

            result = self.sendToStove("OFF")
            self.getMsg(result.success, "Stove Off", "Stove fails")
            time.sleep(1)
            print("============================\n")


def main():
    rclpy.init()

    print("preparing...")
    console = Console()
    sensor_info = SensorInfo()

    executor = SingleThreadedExecutor()
    executor.add_node(sensor_info)
    executor.add_node(console)
    threading.Thread(target=console.doTask).start()
    executor.spin()


if __name__ == "__main__":
    main()
