import os
import rospkg
import rospy
import rosservice

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QSpinBox, QMessageBox, QCheckBox
from python_qt_binding.QtCore import Qt, Slot

class SLSensorCalibrationPlugin(Plugin):

    def __init__(self, context):
        super(SLSensorCalibrationPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SLSensorCalibrationPlugin')
        rp = rospkg.RosPack()

        # Create QWidget
        self.widget_ = QWidget()

        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('sl_sensor_calibration_gui'), 'resource', 'Calibration.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.widget_)

        # Give QObjects reasonable names
        self.widget_.setObjectName('CalibrationUi')

        # Show widget_.windowTitle on left-top of each plugin (when 
        # it's set in widget_). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self.widget_.setWindowTitle(self.widget_.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Add widget to the user interface
        context.add_widget(self.widget_)
        
        # Connect buttons to events
        self.widget_.grabNextSequence.clicked.connect(self.grab_next_sequence)
        self.widget_.erasePreviousSequence.clicked.connect(self.erase_previous_sequence)

        # Get required service names
        self.grab_next_sequence_service_name = rospy.get_param("grab_sequence_service_name", default="/grab_calibration_sequence")
        self.erase_previous_sequence_service_name = rospy.get_param("erase_sequence_service_name", default="/erase_calibration_sequence")

        if self.grab_next_sequence_service_name[0] != "/":
          self.grab_next_sequence_service_name = "/" + self.grab_next_sequence_service_name

        if self.erase_previous_sequence_service_name[0] != "/":
          self.erase_previous_sequence_service_name = "/" + self.erase_previous_sequence_service_name

    @Slot()
    def erase_previous_sequence(self):
      rospy.logwarn(self.erase_previous_sequence_service_name)
      self.send_trigger_request(self.erase_previous_sequence_service_name)

    @Slot() 
    def grab_next_sequence(self):
      rospy.logwarn(self.grab_next_sequence_service_name)
      self.send_trigger_request(self.grab_next_sequence_service_name)

    def send_trigger_request(self, service_name):
      try:
        srv_class = rosservice.get_service_class_by_name(service_name)
        srv_client = rospy.ServiceProxy(service_name, srv_class)
        request = srv_class._request_class()
        success = srv_client(request)
      except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
          rospy.logwarn(
              "[SLSensorCalibrationPlugin] Could not get class of service %s: %s" %
              (service_name, e))
          QMessageBox.warning(self.widget_, "Warning",
                              "Could not send service request to CalibrationSequenceAcquisitionNodelet, make sure it is running.")
                              
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
    
