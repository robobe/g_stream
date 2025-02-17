import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class DemoWidget(QWidget):
    def __init__(self, node, plugin):
        super(DemoWidget, self).__init__()

        self._node = node
        self._plugin = plugin

        _, package_path = get_resource('packages', 'g_stream_control')
        ui_file = os.path.join(package_path, 'share', 'g_stream_control', 'resource', 'Demo.ui')
        loadUi(ui_file, self)
        self._node.get_logger().info("loaded")