from std_srvs.srv import Trigger
from rclpy.node import Node
import os
from pathlib import Path
import yaml

TOPIC = "param_dump"
PARAM_LOCATION = "param_yaml_full_path"
# ros2 service call /stream/param_dump std_srvs/srv/Trigger "{}"

class ParamDumpManager():
    def __init__(self, node):
        self.node: Node
        self.node = node
        if self.node.has_parameter(PARAM_LOCATION):
            self.location = self.node.get_parameter(PARAM_LOCATION).value
            if self.location:
                self.dump_srv = self.node.create_service(Trigger, 
                                self.node.get_name() + "/" + TOPIC, 
                                self.dump_handler)
                if not Path(self.location).exists():
                    raise Exception(f"param yaml path {self.location} not valid")


    def dump_handler(self, request: Trigger.Request, response: Trigger.Response):


        data = {k: v.value for k, v in self.node._parameters.items()}
        yaml_output = {"/**/" + self.node.get_name(): {"ros__parameters": data}}
        self.node.get_logger().warning(f"Save parameter to location: {self.location}")
        with open(self.location, "w", encoding="utf-8") as f:
            yaml.dump(yaml_output, stream=f, default_flow_style=False)

        response.success = True
        response.message = f"save parameters to {self.location} "
        return response