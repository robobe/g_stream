#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from qt_gui.plugin import Plugin
from std_srvs.srv import SetBool, Trigger
from g_stream_interface.srv import Preset
from .rqt_demo import DemoWidget
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from functools import partial
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QApplication

PARAM_RECEIVER_PIPE = "receiver_pipe"
PARAM_PRESET = "preset"


class Worker(QObject):
    receiver_pipe = pyqtSignal(str)
    preset = pyqtSignal(str)

class DemoPlugin(Plugin):
    def __init__(self, context):
        super(DemoPlugin, self).__init__(context)
        self.worker = Worker()
        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('RQTDemo')
        self._widget = DemoWidget(self._node, self)
        self.node = context.node if context.node else rclpy.create_node("my_rqt_plugin")

        # Create ROS 2 service client
        self.start_stop_client = self.node.create_client(SetBool, "/stream/start_stop")
        # self.param_set = self.node.create_client(SetParameters, '/stream_node/set_parameters')
        self.preset_set = self.node.create_client(Preset, '/stream/set_preset')
        while not self.preset_set.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Waiting for parameter service...')

        self._widget.cmd_low_preset.clicked.connect(partial(self.call_service, "low"))
        self._widget.cmd_medium_preset.clicked.connect(partial(self.call_service, "medium"))
        self._widget.cmd_high_preset.clicked.connect(partial(self.call_service, "high"))
        self._widget.cmd_start_pipe.clicked.connect(partial(self.start_stop_service, True))
        self._widget.cmd_stop_pipe.clicked.connect(partial(self.start_stop_service, False))
        self._widget.cmdCopy.clicked.connect(self.copy_to_clipbard)
        self.worker.receiver_pipe.connect(self.update_receiver_pipe)
        self.worker.preset.connect(self.update_select_preset)

        context.add_widget(self._widget)
        self.get_parameters_service()

    def copy_to_clipbard(self):
        data = self._widget.txtReceivePipe.toPlainText()
        clip = QApplication.clipboard()
        clip.setText(data)

    def update_receiver_pipe(self, txt: str):
        # import threading
        # self.node.get_logger().info(f'try to update: {threading.current_thread().getName()}')
        self._widget.txtReceivePipe.setPlainText(txt.strip())

    def update_select_preset(self, preset: str):
        """
        update preset button from parameter
        """
        self._widget.cmd_low_preset.setStyleSheet("")
        self._widget.cmd_medium_preset.setStyleSheet("")
        self._widget.cmd_high_preset.setStyleSheet("")

        if preset == "low":
            self._widget.cmd_low_preset.setStyleSheet("background-color: green;")
        elif preset == "medium":
            self._widget.cmd_medium_preset.setStyleSheet("background-color: green;")
        else:
            self._widget.cmd_high_preset.setStyleSheet("background-color: green;")

    def get_parameters_service(self):
        """
        Get receiver pipe and preset parameter using service
        """
        self.node.get_logger().info("Call get parameters service")
        get_parameters_client = self.node.create_client(GetParameters, '/stream/get_parameters')
        while not get_parameters_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Waiting for get parameters service...')

        request = GetParameters.Request()
        request.names = [PARAM_RECEIVER_PIPE, PARAM_PRESET]

        future = get_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            receiver_pipe = future.result().values[0].string_value
            self.worker.receiver_pipe.emit(receiver_pipe)
            self.node.get_logger().info(f"Get parameters success: {receiver_pipe}")

            preset = future.result().values[1].string_value
            self.worker.preset.emit(preset)
            
        else:
            self.node.get_logger().error('Failed to get parameters')


    def start_stop_service(self, state):
        request = SetBool.Request()
        request.data = state
        future = self.start_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"start stop sucess")
        else:
            self.node.get_logger().error('Failed start/stop pipe')

    def call_service(self, preset):
        self.node.get_logger().info("Call service")
        request = Preset.Request()
        self.node.get_logger().info(f"Try set preset {preset}")
        # Create parameter object
        request.preset = preset

        future = self.preset_set.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"Parameter update success")
            self.get_parameters_service()
        else:
            self.node.get_logger().error('Failed to update parameter')

    # def call_service(self, preset):
    #     self.node.get_logger().info("Call service")
    #     request = SetParameters.Request()
    #     self.node.get_logger().info(f"Try set preset {preset}")
    #     # Create parameter object
    #     param = Parameter()
    #     param.name = "preset"
    #     param.value = ParameterValue()
    #     param.value.string_value = preset
    #     param.value.type = 4

    #     request.parameters.append(param)

    #     future = self.preset_set.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)

    #     if future.result() is not None:
    #         self.node.get_logger().info(f"Parameter update success")
    #         self.force_preset()
    #     else:
    #         self.node.get_logger().error('Failed to update parameter')


"""
ros2 service call /stream_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'preset', value: {string_value: 'xxx'}}]}"

ros2 service call /stream_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['preset']}"
ros2 service call /stream_node/set_parameters_atomically rcl_interfaces/srv/SetParametersAtomically "{parameters: [{name: 'preset', value: {type: 4, string_value: 'low'}}]}"
ros2 service call /stream_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'preset', value: {type: 4, string_value: 'low'}}]}"

"""