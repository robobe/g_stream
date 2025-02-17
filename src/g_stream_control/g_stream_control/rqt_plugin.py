#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from qt_gui.plugin import Plugin
from std_srvs.srv import SetBool, Trigger
from .rqt_demo import DemoWidget


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

        self._widget.pushButton.clicked.connect(self.call_service)
        
        context.add_widget(self._widget)

    def call_service(self):
        self.node.get_logger().info("Call service")
