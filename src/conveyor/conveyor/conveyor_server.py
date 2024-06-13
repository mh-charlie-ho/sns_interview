import rclpy
from rclpy.node import Node
from sns_msg.srv import ConveyorAction

import random


class ConveyorServer(Node):
    def __init__(self):
        super().__init__("conveyor_server")
        self.srv = self.create_service(ConveyorAction, "conveyor_service", self.process)

        self.ingredientDict = {1: "garlic", 2: "sausage", 3: "rice"}
        self.stateMsg = {0: "There is not the ingredient.", 1: "delivering..."}

    def getIngredient(self, id):
        if id in self.ingredientDict:
            return self.ingredientDict[id]
        else:
            return None

    def checkState(self, ingredient):
        """
        check if the food is delivered.
        """
        result = round(random.uniform(0, 1))
        if result == 0:
            return False
        else:
            return True

    def process(self, request, response):
        ingredient = self.getIngredient(request.ingredient_id)

        noIngredient = ingredient is None
        stateNoGood = self.checkState(ingredient) is False

        if noIngredient or stateNoGood:
            response.state_msg = self.stateMsg[0]
            response.success = False

            return response
        else:
            response.state_msg = self.stateMsg[1] + ingredient
            response.success = True

            return response


def main():
    rclpy.init()

    print("starting...")

    conveyor_server = ConveyorServer()
    rclpy.spin(conveyor_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
