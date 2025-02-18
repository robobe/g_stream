from std_srvs.srv import Trigger
from rclpy.node import Node
import os
from pathlib import Path
import yaml

TOPIC = "save_parameters"

# ros2 service call /stream/save_parameters std_srvs/srv/Trigger "{}"

class ParamDumpManager():
    def __init__(self, node):
        self.node: Node
        self.node = node

        self.dump_srv = self.node.create_service(Trigger, 
                            self.node.get_name() + "/" + TOPIC, 
                            self.dump_handler)
        
    def dump_handler(self, request: Trigger.Request, response: Trigger.Response):
        env_name = self.node.get_name().upper() + "_CONFIG_LOCATION"
        config_path = os.environ.get(env_name)
        if not config_path:
            # TODO: get runtime production path
            config_path = "/tmp"
        data = {k: v.value for k, v in self.node._parameters.items()}
        yaml_output = {"/**/" + self.node.get_name(): {"ros__parameters": data}}
        base_path = Path(config_path).joinpath("config")
        base_path.mkdir(parents=True, exist_ok=True)
        full_path = (
            Path(config_path)
            .joinpath("config")
            .joinpath(self.node.get_name() + ".yaml")
            .as_posix()
        )


        with open(full_path, "w", encoding="utf-8") as f:
            yaml.dump(yaml_output, stream=f, default_flow_style=False)

        response.success = True
        return response