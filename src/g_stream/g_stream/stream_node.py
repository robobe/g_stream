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
DEFAULT_MTU = 1400
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5000
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_NOT_SET = 0
DEFAULT_BITRATE = 500
DEFAULT_IFRAMEINTERVAL = 30
# endregion

# region parameters


PARAM_MTU = "mtu"
PARAM_ENCODER_TYPE = "encoder_type"
PARAM_PRESET = "preset"
PARAM_PRESET_LOW = "preset_low"
PARAM_PRESET_MEDIUM = "preset_medium"
PARAM_PRESET_HIGH = "preset_high"
PARAM_HOST = "host"
PARAM_PORT = "port"
PARAM_VBV = "vbv"
PARAM_ARCH = "arch"
PARAM_TEST_ENABLE = "test_enable"
PARAM_RECEIVER_PIPE = "receiver_pipe"
PARAM_WIDTH = "width"
PARAM_HEIGHT = "height"
# end region

NAME = "stream_node"

# region topics name
CAMERA_TOPIC = "/camera/image_raw"
START_STOP_SRV = "start_stop"
SRV_PRESET = "set_preset"
# end region

# region helper class
class EncoderArch(Enum):
    CPU = "cpu"
    NVIDIA = "nvidia"

class Presets(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"



class PresetsItems(Enum):
    BITRATE = "bitrate"
    FPS = "fps"
    IFRAME = "iframeinterval"

# end region

class EncoderType(Enum):
    H264 = "h264"
    H265 = "h265"

class StreamHandlerNode(Node):
    def __init__(self):
        super().__init__(NAME)

        # self.execute_gst()
        self.callback_group = ReentrantCallbackGroup()
        self.cv_br = CvBridge()
        self.gst = None
        self._init_parameters()
        self._init_services()
        self._init_subscribers()
        # self.timer = self.create_timer(1.0, self.__timer_handler)
        # preset = self.get_parameter(PARAM_PRESET).value
        self.add_on_set_parameters_callback(self.parameters_handler)
        self.play()



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
        success = True
        param_result = SetParametersResult()
        for param in params:
            try:
                if param.name == PARAM_PRESET:
                    # throw exception on not valid value #TODO: think again for exception for login issue
                    _ = Presets(param.value)

                if param.name == PARAM_ENCODER_TYPE:
                    # throw exception on not valid value #TODO: think again for exception for login issue
                    _ = EncoderType(param.value)

                if param.name == PARAM_ARCH:
                    # throw exception on not valid value #TODO: think again for exception for login issue
                    _ = EncoderArch(param.value)

                self.get_logger().warning(f"Try to update {param.name} ")
                success = True

            except Exception as err:
                self.get_logger().error(str(err))
                self.get_logger().error("Failed to update parameter")
                success = False

        param_result.successful = success
        return param_result

    def get_preset_default(self, preset_name, item):
        self.get_logger().warning(f"Set default value for preset: {preset_name} - {item}")
        preset_item = PresetsItems(item)
        if preset_item == PresetsItems.FPS:
            return DEFAULT_FPS
        elif preset_item == PresetsItems.BITRATE:
            return DEFAULT_BITRATE
        elif preset_item == PresetsItems.IFRAME:
            return DEFAULT_IFRAMEINTERVAL
        else:
            self.get_logger().error("Wrong item {item} ")
        
    def _init_parameters(self):
        self.declare_parameter(PARAM_TEST_ENABLE, value=True)
        self.declare_parameter(PARAM_VBV, value=DEFAULT_NOT_SET)
        self.declare_parameter(PARAM_PRESET, value=Presets.LOW.value)
        self.declare_parameter(PARAM_MTU, value=DEFAULT_MTU)
        self.declare_parameter(PARAM_ENCODER_TYPE, value=EncoderType.H264.value)
        self.declare_parameter(PARAM_HOST, value=DEFAULT_HOST)
        self.declare_parameter(PARAM_PORT, value=DEFAULT_PORT)
        self.declare_parameter(PARAM_ARCH, value=EncoderArch.CPU.value)
        self.declare_parameter(PARAM_WIDTH, value=DEFAULT_WIDTH)
        self.declare_parameter(PARAM_HEIGHT, value=DEFAULT_HEIGHT)
        

        # decalre preset params for each 
        for group in Presets:
            for item in PresetsItems:
                self.declare_parameter(f"{group.value}.{item.value}", value=self.get_preset_default(group.value, item.value))

    # endregion

    # region subscribers
    def _init_subscribers(self):

        self.img_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_handler,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
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
        self.gst.push_image(frame)
    #endregion

    #endregion
    def build_topic_name(self, base_name):
        node_name = self.get_name()
        return f"{node_name}/{base_name}"
    
    def build_pipe(self):
        preset = self.get_parameter(PARAM_PRESET).value
        
        fps = self.get_parameter(f"{preset}.{PresetsItems.FPS.value}").value
        bitrate = self.get_parameter(f"{preset}.{PresetsItems.BITRATE.value}").value
        iframeinterval = self.get_parameter(f"{preset}.{PresetsItems.IFRAME.value}").value

        encoder_type = EncoderType(self.get_parameter(PARAM_ENCODER_TYPE).value)
        encoder_arch = EncoderArch(self.get_parameter(PARAM_ARCH).value)
        vbv = self.get_parameter(PARAM_VBV).value
        mtu = self.get_parameter(PARAM_MTU).value
        port = self.get_parameter(PARAM_PORT).value
        host = self.get_parameter(PARAM_HOST).value
        test_enable = self.get_parameter(PARAM_TEST_ENABLE).value
        width  = self.get_parameter(PARAM_WIDTH).value
        height  = self.get_parameter(PARAM_HEIGHT).value

        pipe_header = f"appsrc name=app_src  is-live=true format=GST_FORMAT_TIME ! video/x-raw,width={width},height={height},format=BGR,framerate=30/1 "
        if test_enable:
            pipe_header = f"videotestsrc  name=app_src is-live=true ! video/x-raw, width={width}, height={height}, framerate=30/1, format=I420 "
        
        if encoder_arch==EncoderArch.CPU:
            if encoder_type == EncoderType.H264:
                self.get_logger().warning("encoder not support vbv, iframeinterval argument")

                #region pipe cpu h264
                pipeline_desc = f"""{pipe_header}
                ! videoconvert \
                ! queue max-size-buffers=1 leaky=downstream \
                ! videorate \
                ! video/x-raw,framerate={fps}/1 \
                ! x264enc \
                    tune=zerolatency \
                    speed-preset=ultrafast \
                    bitrate={bitrate} \
                ! rtph264pay config-interval=1 mtu={mtu} \
                ! udpsink host={host} port={port} sync=true"""

                receiver_pipe=f"""
                gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H264, payload=96 ! rtpjitterbuffer latency=10 ! rtph264depay ! avdec_h264 ! videoconvert ! fpsdisplaysink sync=true
                """
                #endregion

            if encoder_type == EncoderType.H265:
                self.get_logger().warning("encoder not support vbv argument")
                # region pipe cpu h265
                pipeline_desc = f"""{pipe_header}
                ! videoconvert \
                ! queue max-size-buffers=1 leaky=downstream \
                ! videorate \
                ! video/x-raw,framerate={fps}/1 \
                ! x265enc \
                    tune=zerolatency \
                    speed-preset=ultrafast \
                    bitrate={bitrate} \
                ! rtph265pay config-interval=1 mtu={mtu} \
                ! udpsink host={host} port={port} sync=true"""

                receiver_pipe=f"""
                gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H265, payload=96 ! rtpjitterbuffer latency=10 ! rtph265depay ! avdec_h265 ! videoconvert ! fpsdisplaysink sync=true
                """
                # endregion

        if encoder_arch == EncoderArch.NVIDIA:
            if encoder_type == EncoderType.H264:
                nvidia_bitrate = bitrate*1000
                if vbv == DEFAULT_NOT_SET:
                    vbv = nvidia_bitrate
                #region pipe nvidia h264
                pipeline_desc = f"""{pipe_header}
                ! videoconvert \
                ! queue max-size-buffers=1 leaky=downstream \
                ! videorate \
                ! video/x-raw,framerate={fps}/1 \
                ! nvvidconv \
                ! nvv4l2h264enc bitrate={nvidia_bitrate} \
                    maxperf-enable=true \
                    preset-level=UltraFastPreset \
                    control-rate=constant_bitrate \
                    vbv-size={vbv} \
                    iframeinterval={iframeinterval} \
                ! h264parse \
                ! rtph264pay config-interval=1 mtu={mtu} \
                ! udpsink host={host} port={port} sync=true
                """

                receiver_pipe=f"""
                gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H264, payload=96 ! rtpjitterbuffer latency=10 ! rtph264depay ! avdec_h264 ! videoconvert ! fpsdisplaysink sync=true
                """
                #endregion
            if encoder_type == EncoderType.H265:
                nvidia_bitrate = bitrate*1000
                if vbv == DEFAULT_NOT_SET:
                    vbv = nvidia_bitrate
                #region pipe nvidia h265
                pipeline_desc = f"""{pipe_header}
                ! videoconvert \
                ! queue max-size-buffers=1 leaky=downstream \
                ! videorate \
                ! video/x-raw,framerate={fps}/1 \
                ! nvvidconv \
                ! nvv4l2h265enc bitrate={nvidia_bitrate} \
                    maxperf-enable=true \
                    preset-level=UltraFastPreset \
                    control-rate=constant_bitrate \
                    vbv-size={vbv} \
                    iframeinterval={iframeinterval} \
                ! h265parse \
                ! rtph265pay config-interval=1 mtu={mtu} \
                ! udpsink host={host} port={port} sync=true
                """

                receiver_pipe=f"""
                gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H265, payload=96 ! rtpjitterbuffer latency=10 ! rtph265depay ! avdec_h265 ! videoconvert ! fpsdisplaysink sync=true
                """
                #endregion

        
        if self.has_parameter(PARAM_RECEIVER_PIPE):
            self.undeclare_parameter(PARAM_RECEIVER_PIPE)
        desc = ParameterDescriptor(read_only=True)
        self.declare_parameter(PARAM_RECEIVER_PIPE, value=receiver_pipe)#, descriptor=desc)
        
        if minimal_pipe.SRC_ELEMENT not in pipeline_desc:
            self.get_logger().error("bad pipe ------------------")
            self.get_logger().error(pipeline_desc)

            raise Exception("source element {SRC_ELEMENT} not found in pipe")

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