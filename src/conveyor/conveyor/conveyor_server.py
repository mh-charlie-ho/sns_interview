#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sns_msgs.srv import ConveyorAction


class ConveyorServer(Node):
    def __init__(self):
        super().__init__("conveyor_server")
        self.srv = self.create_service(ConveyorAction, "conveyor_service", self.process)
        self.ingredientDict = {
            1: "garlic",
            2: "sausage",
            3: "rice"
        }

    def getIngredient(self, id):
        return self.ingredientDict.get(id)

    def process(self, request, response):
        ingredient = self.getIngredient(request.ingredient_id)
        
        if ingredient:
            response.success = True
            response.state_msg = "delivering... " + ingredient
        else:
            response.success = False
            response.state_msg = "There is not the ingredient."
        
        return response


def main():
    rclpy.init()
    print("starting conveyor server")

    conveyor_server = ConveyorServer()
    rclpy.spin(conveyor_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
