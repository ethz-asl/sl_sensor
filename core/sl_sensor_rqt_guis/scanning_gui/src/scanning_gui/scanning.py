import os
import rospkg
import rospy
import rosservice

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QSpinBox, QMessageBox
from python_qt_binding.QtCore import Qt, Slot

class ScanningPlugin(Plugin):

    def __init__(self, context):
        super(ScanningPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ScanningPlugin')
        rp = rospkg.RosPack()

        # Create QWidget
        self.widget_ = QWidget()

        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('scanning_gui'), 'resource', 'Scanning.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self.widget_)

        # Give QObjects reasonable names
        self.widget_.setObjectName('ScanningUi')

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
        self.widget_.startScanHWTrigger.clicked.connect(self.start_scan_hw_trigger)
        self.widget_.startScanSWTrigger.clicked.connect(self.start_scan_sw_trigger)
        self.widget_.stopScan.clicked.connect(self.stop_scan_sw_trigger)
        

    @Slot()
    def start_scan_hw_trigger(self):
      self.send_command_image_synchroniser_command("start", True, self.widget_.numberScans.value(), self.widget_.delayMs.value())

    @Slot()
    def start_scan_sw_trigger(self):
      self.send_command_image_synchroniser_command("start", False, self.widget_.numberScans.value(), self.widget_.delayMs.value())

    @Slot()
    def stop_scan_sw_trigger(self):
      self.send_command_image_synchroniser_command("stop", False, 0, 0)

    def send_command_image_synchroniser_command(self, command, is_hw_trigger, number_scans, delay_ms):
      service = "/command_image_synchroniser"
      try:
        srv_class = rosservice.get_service_class_by_name(service)
        srv_client = rospy.ServiceProxy(service, srv_class)
        request = srv_class._request_class()

        request.command = command
        request.pattern_name = "" # Empty pattern name so image synchroniser will use the default one it has been set to
        request.is_hardware_trigger = is_hw_trigger
        request.number_scans = number_scans
        request.delay_ms = delay_ms

        success = srv_client(request)
        if not success:
            QMessageBox.warning(self.widget_, "Warning",
                                "Sending of service request ro image synchroniser failed")
      except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
          rospy.logwarn(
              "[ScanningGui] Could not get class of service %s: %s" %
              (service, e))
          QMessageBox.warning(self.widget_, "Warning",
                              "Could not send service request to image synchroniser, make sure it is running.")
                              
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
    