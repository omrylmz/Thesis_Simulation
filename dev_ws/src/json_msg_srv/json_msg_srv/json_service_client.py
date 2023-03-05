import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from json_interfaces.srv import JsonService

        
class JsonServiceClient(Node):
    def __init__(self, node_name):
        id = str(time.time()).replace(".", "")
        super().__init__("JsonServiceClient_" + id)
        self.request = JsonService.Request()
        self.cli = self.create_client(JsonService, node_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def sync_json_call(self, req_json: str) -> str:
        self.request.req = req_json
        resp_future = self.cli.call_async(self.request)
        while rclpy.ok():
            rclpy.spin_once(self)
            if resp_future.done():
                print("json_service_client is called.")
                return resp_future.result().resp
        return ""


def main(args=None):
    rclpy.init(args=args)
    pp_client = JsonServiceClient("path_planner")

    req_json = json.dumps({
        "start": [0.0, 0.0],
        "end": [5.0, 5.0],
        "static_environment": True
    })

    resp_json = pp_client.sync_json_call(req_json)
    print(resp_json)
    
    pp_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    