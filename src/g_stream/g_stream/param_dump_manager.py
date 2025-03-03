from std_srvs.srv import Trigger
from rclpy.node import Node
import os
from pathlib import Path
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter

TOPIC = "param_dump"
PARAM_LOCATION = "param_yaml_full_path"
# ros2 service call /stream/param_dump std_srvs/srv/Trigger "{}"


def traverse_yaml(data, parent_key=""):
    if isinstance(data, dict):
        for key, value in data.items():
            new_key = f"{parent_key}.{key}" if parent_key else key
            traverse_yaml(value, new_key)
    elif isinstance(data, list):
        for index, item in enumerate(data):
            new_key = f"{parent_key}"
            traverse_yaml(item, new_key)
    else:
        print(f"{parent_key}: {data}")

# Run recursive traversal

def param_loader(node: Node, path: str):
    if not Path(path).exists():
        return False
    
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    traverse_yaml(data)

        # node.declare_parameter(param, value)

def param_save(data):
    data["high"]["fps"] = 10

    # Save the updated YAML back to the file
    with open("/tmp/data.yaml", "w") as file:
        yaml.dump(data, file, default_flow_style=False)

if __name__ == "__main__":
    yaml_data = """
    preset: "low"
    width: 640
    height: 480
    high:
        fps: 5
        bitrate: 200
        iframe_interval: 5
        vbv: 200
    low:
        fps: 5
        bitrate: 200
        iframe_interval: 5
        vbv: 200
        """

# Load YAML data
    data = yaml.safe_load(yaml_data)
    traverse_yaml(data)
    param_save(data)


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