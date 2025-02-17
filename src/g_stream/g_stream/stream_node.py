#!/usr/bin/env python3

# very importent
# https://github.com/ros2/rclpy/issues/1149


import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rcl_interfaces.msg import (
    IntegerRange,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

from typing import List
# import stream_node_gst as gst_handler
from enum import Enum, IntEnum

import minimal_pipe

# region consts
DEFAULT_FPS = 10
# endregion

# region parameters
PARAM_BITRATE = "bitrate"
PARAM_FPS = "fps"

PARAM_PRESET = "preset"
PARAM_PRESET_LOW = "preset_low"
PARAM_PRESET_MEDIUM = "preset_medium"
PARAM_PRESET_HIGH = "preset_high"
# end region

NAME = "stream_node"

# region topics name
CAMERA_TOPIC = "/camera/image_raw"
START_STOP_SRV = "start_stop"
SRV_PRESET = "set_preset"
# end region

# region helper class
class Presets(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

# end region



class StreamHandlerNode(Node):
    def __init__(self):
        super().__init__(NAME)

        # self.execute_gst()
        self.callback_group = ReentrantCallbackGroup()
        self.cv_br = CvBridge()
        self.gst = None
        self._init_parameters()
        self._init_services()
        # self.timer = self.create_timer(1.0, self.__timer_handler)
        # preset = self.get_parameter(PARAM_PRESET).value
        self.add_on_set_parameters_callback(self.parameters_handler)
        self.play()

    def __timer_handler(self):
        preset1 = self.get_parameter("low.fps").value
        preset2 = self.get_parameter("medium.fps").value
        preset3 = self.get_parameter("high.fps").value

        self.get_logger().info(f"current preset {preset1}")
        self.get_logger().info(f"current preset {preset2}")
        self.get_logger().info(f"current preset {preset3}")

    # region init
    def _init_services(self):
        self.start_stop_srv = self.create_service(SetBool, 
                            self.build_topic_name(START_STOP_SRV), 
                            self.start_stop_handler)
        self.start_stop_srv = self.create_service(Trigger, 
                            self.build_topic_name(SRV_PRESET), 
                            self.set_preset_handler)
    # endregion
    # region parameters

    def parameters_handler(self, params: List[Parameter]):
        self.get_logger().info(f"---------------------------bb")
        success = True
        param_result = SetParametersResult()
        for param in params:
            try:
                if param.name == PARAM_PRESET:
                    # throw exception on not valid value #TODO: think again for exception for login issue
                    preset = Presets(param.value)
                    
                

                success = True

            except Exception as err:
                self.get_logger().error(str(err))
                self.get_logger().error("Failed to update parameter")
                success = False

        param_result.successful = success
        return param_result

    def _init_parameters(self):
        bitrate_descriptor = ParameterDescriptor(
            description="stream bitrate",
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(from_value=100, to_value=700)],
        )
        # self.declare_parameter(PARAM_BITRATE, value=300, descriptor=bitrate_descriptor)

        #default_preset
        # preset_descriptor = ParameterDescriptor(type=Presets)
        self.declare_parameter(PARAM_PRESET, value=Presets.LOW.value)
        self.declare_parameter(PARAM_FPS,value=DEFAULT_FPS)
        for group in ["low", "medium", "high"]:
            for item in ["fps", "bitrate"]:

                self.declare_parameter(f"{group}.{item}")
    # endregion

    # region subscribers
    def _init_subscribers(self):

        self.img_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_handler,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.g,
        )
    # endregion

    # region handlers
    # region services
    
    def set_preset_handler(self, request: SetBool.Request, response: SetBool.Response):
        """
        ros2 param get /stream_node preset
        ros2 param set /stream_node preset high
        ros2 param set /stream_node preset medium
        ros2 param set /stream_node preset low
        ros2 service call /stream_node/set_preset std_srvs/srv/Trigger "{}"
        """
        self.get_logger().warning(f"--Restart pipe--")
        self.stop()
        time.sleep(1)
        self.play()
        
        response.success = True
        return response
    
    def start_stop_handler(self, request: SetBool.Request, response: SetBool.Response):
        """
        ros2 service call /stream_node/start_stop std_srvs/srv/SetBool "{data: true}"
        ros2 service call /stream_node/start_stop std_srvs/srv/SetBool "{data: false}"
        """
        if request.data:
            self.get_logger().info("Start stream request")
            self.play()
            response.message = "Start stream"
        else:
            self.get_logger().info("Stop stream request")
            self.stop()
            response.message = "Stop stream"

        
        response.success = True
        return response
    # endregion
    # region subscribers
    def image_handler(self, msg: Image):
        frame = self.cv_br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # gst_handler.push_image(frame)
    #endregion

    #endregion
    def build_topic_name(self, base_name):
        node_name = self.get_name()
        return f"{node_name}/{base_name}"
    
    def build_pipe(self):
        preset = self.get_parameter(PARAM_PRESET).value
        fps = self.get_parameter(f"{preset}.fps").value
        pipeline_desc = f"videotestsrc ! video/x-raw,width=640,height=480 ! videoconvert ! videorate ! video/x-raw,framerate={fps}/1 ! fpsdisplaysink"
        return pipeline_desc
    
    def play(self):
        pipe = self.build_pipe()
        self.get_logger().info(pipe)
        self.gst = minimal_pipe.GstPipelineThread(pipe)
        self.gst.start()

    def stop(self):
        self.gst.stop()

def main(args=None):
    rclpy.init(args=args)
    node = StreamHandlerNode()
    executer = MultiThreadedExecutor()
    executer.add_node(node)
    executer.spin()

    node.destroy_node()
    executer.shutdown()
    rclpy.shutdown()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()