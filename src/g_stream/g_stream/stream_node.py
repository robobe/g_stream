#!/usr/bin/env python3

# very importent
# https://github.com/ros2/rclpy/issues/1149

import sys
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
from diagnostic_updater import (
    DiagnosticStatus,
    DiagnosticStatusWrapper,
    DiagnosticTask,
    Updater,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
import os
from typing import List
# import stream_node_gst as gst_handler
from enum import Enum, IntEnum

import minimal_pipe
from bfid import draw_binary_on_image
# from param_dump_manager import ParamDumpManager, PARAM_LOCATION
from g_stream_interface.srv import Preset
from parameters_manager_ex import ParameterManagerEx

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
PARAM_HOST = "ip_address"
PARAM_PORT = "port"
PARAM_HARDWARE = "hardware"
PARAM_TEST_ENABLE = "test_enable"
PARAM_RECEIVER_PIPE = "receiver_pipe"
PARAM_WIDTH = "width"
PARAM_HEIGHT = "height"
PARAM_ON_IMAGE_TIME_STAMP = "on_image_time_stamp"
PARAM_STATUS = "status"
PARAM_LOCATION = "param_yaml_full_path"
# end region

NAME = "stream"

# region topics name
CAMERA_TOPIC = "/image_raw"
START_STOP_SRV = "start_stop"
SRV_PRESET = "set_preset"
# end region

# region helper class
class HeartbeatStatus(DiagnosticTask):
    def __init__(self, name, pid):
        super().__init__(name)
        self.pid = pid

    def run(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.OK, "Running")

        stat.add("pid", str(self.pid))
        return stat
    
class EncoderHardware(Enum):
    PC = "pc"
    NVIDIA = "nvidia"

class Presets(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"



class PresetsItems(Enum):
    BITRATE = "bitrate"
    FPS = "fps"
    IFRAME = "iframe_interval"
    VBV = "vbv"

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
        self._init_diagnostic()
        self.param_dump_manager = ParameterManagerEx(self, self.get_fully_qualified_name())
        self._on_image_time_stamp = self.get_parameter(PARAM_ON_IMAGE_TIME_STAMP).value
        # self.timer = self.create_timer(1.0, self.__timer_handler)
        # preset = self.get_parameter(PARAM_PRESET).value
        self.add_on_set_parameters_callback(self.parameters_handler)
        self.play()



    # region init
   
    
    def _init_diagnostic(self):
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID(self.get_name())
        self.hb_diagnostic = HeartbeatStatus("HB", os.getpid())
        self.diagnostic_updater.add(self.hb_diagnostic)

    def _init_services(self):
        self.start_stop_srv = self.create_service(SetBool, 
                            self.build_topic_name(START_STOP_SRV), 
                            self.start_stop_handler)
        self.preset_srv = self.create_service(Preset, 
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

                if param.name == PARAM_HARDWARE:
                    # throw exception on not valid value #TODO: think again for exception for login issue
                    _ = EncoderHardware(param.value)

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
        elif preset_item == PresetsItems.VBV:
            # same as bitrate
            return DEFAULT_BITRATE
        else:
            self.get_logger().error("Wrong item {item} ")
        
    def _init_parameters(self):
        self.declare_parameter(PARAM_TEST_ENABLE, value=False)
        self.declare_parameter(PARAM_PRESET, value=Presets.LOW.value)
        self.declare_parameter(PARAM_MTU, value=DEFAULT_MTU)
        self.declare_parameter(PARAM_ENCODER_TYPE, value=EncoderType.H264.value)
        self.declare_parameter(PARAM_HOST, value=DEFAULT_HOST)
        self.declare_parameter(PARAM_PORT, value=DEFAULT_PORT)
        self.declare_parameter(PARAM_HARDWARE, value=EncoderHardware.PC.value)
        self.declare_parameter(PARAM_WIDTH, value=DEFAULT_WIDTH)
        self.declare_parameter(PARAM_HEIGHT, value=DEFAULT_HEIGHT)
        self.declare_parameter(PARAM_LOCATION, value="")
        self.declare_parameter(PARAM_RECEIVER_PIPE, value="")
        self.declare_parameter(PARAM_ON_IMAGE_TIME_STAMP, value=True)
        self.declare_parameter(PARAM_STATUS, "")
        # declare preset params for each 
        for group in Presets:
            for item in PresetsItems:
                self.declare_parameter(f"{group.value}.{item.value}", value=self.get_preset_default(group.value, item.value))

    # endregion

    # region subscribers
    def _init_subscribers(self):

        self.img_sub = self.create_subscription(
            Image,
            "/camera/image_raw", #self.build_topic_name(CAMERA_TOPIC),
            self.image_handler,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
    # endregion

    # region handlers
    # region services
    
    def set_preset_handler(self, request: Preset.Request, response: Preset.Response):
        """
        ros2 service call /stream/set_preset g_stream_interface/srv/Preset " {preset: 'high'} "
        ros2 service call /stream/set_preset g_stream_interface/srv/Preset " {preset: 'low'} "
        """
        self.set_parameters([
            Parameter(PARAM_PRESET, value=request.preset)
        ])
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
        if self._on_image_time_stamp:
            frame = draw_binary_on_image(frame, msg.header.stamp.sec, msg.header.stamp.nanosec, bit_size=3)
        try:
            self.gst.push_image(frame)
        except Exception as err:
            self.get_logger().error(str(err))
            self.get_logger().warning("Fail to push image")
    #endregion

    #endregion
    

    def build_topic_name(self, base_name):
        node_name = self.get_name()
        return f"{node_name}/{base_name}".replace("//","/")
    
    def build_pipe(self):
        preset = self.get_parameter(PARAM_PRESET).value
        
        fps = self.get_parameter(f"{preset}.{PresetsItems.FPS.value}").value
        bitrate = self.get_parameter(f"{preset}.{PresetsItems.BITRATE.value}").value
        iframeinterval = self.get_parameter(f"{preset}.{PresetsItems.IFRAME.value}").value
        vbv = self.get_parameter(f"{preset}.{PresetsItems.VBV.value}").value

        encoder_type = EncoderType(self.get_parameter(PARAM_ENCODER_TYPE).value)
        encoder_hw = EncoderHardware(self.get_parameter(PARAM_HARDWARE).value)
        mtu = self.get_parameter(PARAM_MTU).value
        port = self.get_parameter(PARAM_PORT).value
        host = self.get_parameter(PARAM_HOST).value
        test_enable = bool(self.get_parameter(PARAM_TEST_ENABLE).value)
        width  = self.get_parameter(PARAM_WIDTH).value
        height  = self.get_parameter(PARAM_HEIGHT).value
        

        pipe_header = f"appsrc name=app_src  is-live=true format=GST_FORMAT_TIME ! video/x-raw,width={width},height={height},format=BGR,framerate=30/1 "
        if test_enable:
            self.get_logger().warning("Using gstreamer testsource")
            pipe_header = f"videotestsrc  name=app_src is-live=true ! video/x-raw, width={width}, height={height}, framerate=30/1, format=I420  "
        
        if encoder_hw==EncoderHardware.PC:
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
                    key_int_max={fps} \
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
                    key_int_max={fps} \
                ! rtph265pay config-interval=1 mtu={mtu} \
                ! udpsink host={host} port={port} sync=true"""

                receiver_pipe=f"""
                gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H265, payload=96 ! rtpjitterbuffer latency=10 ! rtph265depay ! avdec_h265 ! videoconvert ! fpsdisplaysink sync=true
                """
                # endregion

        if encoder_hw == EncoderHardware.NVIDIA:
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
                    insert-sps-pps=true \
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
                    insert-sps-pps=true \
                ! h265parse \
                ! rtph265pay config-interval=1 mtu={mtu} \
                ! udpsink host={host} port={port} sync=true
                """

                receiver_pipe=f"""
                gst-launch-1.0 udpsrc port={port} ! application/x-rtp, encoding-name=H265, payload=96 ! rtpjitterbuffer latency=10 ! rtph265depay ! avdec_h265 ! videoconvert ! fpsdisplaysink sync=true
                """
                #endregion

        
        # if self.has_parameter(PARAM_RECEIVER_PIPE):
        #     self.undeclare_parameter(PARAM_RECEIVER_PIPE)
        # desc = ParameterDescriptor(read_only=True)
        #, descriptor=desc)
        self.set_parameters([
            Parameter(name=PARAM_RECEIVER_PIPE, value=receiver_pipe)
        ])
        if minimal_pipe.SRC_ELEMENT not in pipeline_desc:
            self.get_logger().error("bad pipe ------------------")
            self.get_logger().error(pipeline_desc)

            raise Exception("source element {SRC_ELEMENT} not found in pipe")

        return pipeline_desc
    
    def state_changed_handler(self, state):
        self.get_logger().info("--------- state change")
        print(state.value_name)
        self.set_parameters([Parameter(PARAM_STATUS, Parameter.Type.STRING, state.value_name)])

    def play(self):
        pipe = self.build_pipe()
        self.get_logger().info(pipe)
        self.gst = minimal_pipe.GstPipelineThread(pipe)
        self.gst.on_state_changed += self.state_changed_handler
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