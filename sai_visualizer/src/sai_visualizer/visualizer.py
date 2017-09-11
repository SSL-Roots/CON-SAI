import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
try:
    from python_qt_binding.QtGui import QWidget
except ImportError as e:
    from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer

from .paint_widget import PaintWidget

class Visualizer(Plugin):

    def __init__(self, context):
        super(Visualizer, self).__init__(context)
        self.setObjectName("Visualizer")

        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path("sai_visualizer"), \
                "resource", "visualizer_widget.ui")
        loadUi(ui_file, self._widget, {"PaintWidget": PaintWidget})
        self._widget.setObjectName("VisualizerUi")

        self._widget.setWindowTitle("SAI Visualizer")

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._redraw_timer = QTimer()
        self._redraw_timer.setInterval(16)
        self._redraw_timer.timeout.connect(self._widget.update)
        self._redraw_timer.start()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
