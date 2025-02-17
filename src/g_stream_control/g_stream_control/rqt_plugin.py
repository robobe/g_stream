#!/usr/bin/env python3


from qt_gui.plugin import Plugin

class DemoPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)

        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('RQTDemo')