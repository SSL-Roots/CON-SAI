import os
import rospy
import rospkg

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtCore import Qt

from consai_msgs.msg import nodeData, nodeDataArray

from xdot.xdot_qt import DotWidget
import pygraphviz as pygr

class BTV(Plugin):

    def __init__(self, context):
        super(BTV, self).__init__(context)

        self._dotcode_sub = None

        # Give QObjects reasonable names
        self.setObjectName("BTV")

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('behavior_tree_visualizer'), 'resource', 'rqt_dot.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('BTVPluginUi')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('  (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        palette = QPalette()
        palette.setColor(QPalette.Background, Qt.white)
        self._widget.setPalette(palette)

        self._widget.stop_button.setCheckable(True)
        self._widget.stop_button.clicked[bool].connect(self._handle_stop_clicked)
        self._widget.all_nodes_button.setCheckable(True)
        self._widget.all_nodes_button.clicked[bool].connect(self._handle_all_nodes_clicked)
        self._widget.show_line_edit.returnPressed.connect(self._handle_show_line_returnPressed)
        self._widget.show_button.clicked.connect(self._handle_show_clicked)
        self._widget.centering_button.clicked.connect(self._handle_centering_clicked)

        self.draw_all_nodes = False
        self.centering = False
        self.show_node_name = ""

        self.set_subscriber()

    def _handle_stop_clicked(self, checked):
        if checked:
            if self._dotcode_sub:
                self._dotcode_sub.unregister()
                self._dotcode_sub = None
        else:
            self.set_subscriber()

    def _handle_only_active_nodes_clicked(self, checked):
        self.draw_only_active_nodes = checked

    def _handle_centering_clicked(self):
        self.centering = True

    def _handle_all_nodes_clicked(self,checked):
        self.draw_all_nodes = checked

    def _handle_show_line_returnPressed(self):
        self.set_show_node_name()

    def _handle_show_clicked(self):
        self.set_show_node_name()

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

    def set_subscriber(self):
        self._dotcode_sub = rospy.Subscriber(
                "node_data_array",
                nodeDataArray,
                self.node_data_call_back)

    def set_show_node_name(self):
        self.show_node_name = self._widget.show_line_edit.text()

    def node_data_call_back(self,nodeDataArray):
        graph = pygr.AGraph()
        show_name = self.show_node_name
        for node in nodeDataArray.nodes:
            my_color,my_shape = self.read_attributes(node)
            my_label = node.myName + "\n" + "("+node.myType+")"

            if node.myStatus != 3 \
                    or self.draw_all_nodes == True \
                    or node.myName == show_name \
                    or node.parentName == show_name:

                # avoid an overwrinting of inactive node on an active node
                if graph.has_node(node.myName) == False \
                        or node.myStatus != 3:
                    graph.add_node(node.myName,
                        style="filled",
                        fillcolor=my_color,
                        shape=my_shape,
                        label=my_label)

                graph.add_edge(node.myName,node.parentName)

        dot_string = graph.to_string()

        # use same sentences to correct centering
        if self.centering == True:
            self._widget.xdot_widget.set_dotcode(dot_string, center=True)
            self.centering = False
        else:
            self._widget.xdot_widget.set_dotcode(dot_string, center=False)

    def read_attributes(self,node):
        color_list = ["red","cyan","green","white"]
        shapde_dict = {
                "Task":"box",
                "Selector":"ellipse",
                "Sequence":"diamond",
                "RandomSelector":"ellipse",
                "RandomSequence":"diamond",
                "Iterator":"triangle",
                "RandomIterator":"triangle",
                "ParallelOne":"invtrapezium",
                "ParallelAll":"invtrapezium",
                "ParallelSelector":"doublecircle",
                "Loop":"box",
                "LoopInf":"Msquare",
                "Limit":"box",
                "IgnoreFailure":"invhouse",
                "AlwaysFail":"invhouse",
                "AlwaysSucceed":"invhouse",
                "Invert":"invhouse",
                "UntilFail":"invhouse",
                "AutoRemoveSequence":"invhouse",
                "CallbackTask":"invhouse"
        }

        return color_list[node.myStatus], \
                shapde_dict.get(node.myType,"box")
