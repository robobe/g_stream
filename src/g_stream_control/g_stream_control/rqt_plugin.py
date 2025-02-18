#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from qt_gui.plugin import Plugin
from std_srvs.srv import SetBool, Trigger
from .rqt_demo import DemoWidget
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class DemoPlugin(Plugin):
    def __init__(self, context):
        super(DemoPlugin, self).__init__(context)

        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('RQTDemo')
        self._widget = DemoWidget(self._node, self)
        self.node = context.node if context.node else rclpy.create_node("my_rqt_plugin")

        # Create ROS 2 service client
        self.client = self.node.create_client(SetBool, "my_service")
        self.param_set = self.node.create_client(SetParameters, '/stream_node/set_parameters')
        
        while not self.param_set.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Waiting for parameter service...')

        self._widget.pushButton.clicked.connect(self.call_service)
        
        context.add_widget(self._widget)

    

    def call_service(self):
        self.node.get_logger().info("Call service")
        request = SetParameters.Request()

        # Create parameter object
        param = Parameter()
        param.name = "preset"
        param.value = ParameterValue()
        param.value.string_value = "high"
        param.value.type = 4

        request.parameters.append(param)

        future = self.param_set.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info(f"Parameter update success")
        else:
            self.node.get_logger().error('Failed to update parameter')


"""
ros2 service call /stream_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'preset', value: {string_value: 'xxx'}}]}"

ros2 service call /stream_node/get_parameters rcl_interfaces/srv/GetParameters "{names: ['preset']}"
ros2 service call /stream_node/set_parameters_atomically rcl_interfaces/srv/SetParametersAtomically "{parameters: [{name: 'preset', value: {type: 4, string_value: 'low'}}]}"
ros2 service call /stream_node/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'preset', value: {type: 4, string_value: 'low'}}]}"

"""