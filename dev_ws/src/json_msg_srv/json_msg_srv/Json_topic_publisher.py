import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from json_interfaces.msg import JsonMessage

        
class JsonTopicPublisher(Node):
    def __init__(self, node_name):
        id = str(time.time()).replace(".", "")
        super().__init__("JsonTopicPublisher_" + id)
        self.message = JsonMessage
        self.publisher = self.create_publisher(JsonMessage, node_name, 10)
        
    def publish_json(self, data: str) -> None:
        self.message.data = data
        self.publisher.publish(self.message)
        return
    