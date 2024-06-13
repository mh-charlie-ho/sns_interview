import rclpy
from rclpy.node import Node
from sns_msg.srv import ConveyorAction


class ConveyorClient(Node):
    def __init__(self):
        super().__init__("conveyor_client")
        self.cli = self.create_client(ConveyorAction, "conveyor_service")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")
        self.req = ConveyorAction.Request()

    def send_request(self, ingredient_id):
        self.req.ingredient_id = ingredient_id

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    conveyor_client = ConveyorClient()

    while rclpy.ok():
        id = int(input("input the ingredient ID: "))
        response = conveyor_client.send_request(id)
        print(response.state_msg)
        print(response.success)
        print("==============================")

    conveyor_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
