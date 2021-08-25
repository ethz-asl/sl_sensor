import os
import rospkg
import rospy
import rosservice

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QSpinBox, QMessageBox, QCheckBox
from python_qt_binding.QtCore import Qt, Slot

class SLSensorScanningPlugin(Plugin):

    def __init__(self, context):
        super(SLSensorScanningPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SLSensorScanningPlugin')
        rp = rospkg.RosPack()

        # Create QWidget
        self.widget_ = QWidget()

        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('sl_sensor_scanning_gui'), 'resource', 'Scanning.ui')

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

        # Attempt to initialise state of checkboxes to be the same as initial state of the loggers, if not set to false
        point_cloud_logger_initially_enabled = False
        try:
            point_cloud_logger_initially_enabled = rospy.get_param("/point_cloud_logger/initially_enabled")
        except KeyError as e:
            rospy.logerr("Parameter %s is not available. Default values of False will be used." % e)
        self.widget_.savePointClouds.setChecked(point_cloud_logger_initially_enabled)
        
        image_logger_initially_enabled = False
        try:
            self.simulation_ = rospy.get_param("/image_logger/initially_enabled")
        except KeyError as e:
            rospy.logerr("Parameter %s is not available. Default values will be used." % e)
        self.widget_.saveImages.setChecked(image_logger_initially_enabled)

        # Populate projector pattern combo box. 
        # If not specified, we add "Default" which sets the pattern field in the /command_image_synchroniser service call to be empty 
        # Empty pattern name means that if the fixed_pattern_name rosparam is specified during the initialisation of the image_synchroniser,
        # it will play that particular pattern. If not, then no action will be performed by the image syncrhoniser
        projector_patterns_concat = ""
        try:
            projector_patterns_concat = rospy.get_param("/sl_sensor_scanning_gui/projector_patterns")
        except KeyError as e:
            rospy.logerr("Parameter %s is not available. Default values will be used." % e)

        projector_patterns = projector_patterns_concat.split(" ")

        if not projector_patterns:
          self.widget_.projectorPatternCombo.addItem("Default")
        else:
          for pattern_name in projector_patterns:
            self.widget_.projectorPatternCombo.addItem(pattern_name)

        # Get projector and image synchroniser service names
        self.image_synchroniser_service_name = rospy.get_param("image_synchroniser_service_name", default="/command_image_synchroniser")

        if self.image_synchroniser_service_name[0] != "/":
          self.image_synchroniser_service_name = "/" + self.image_synchroniser_service_name

        # Connect tick boxes to events
        self.widget_.savePointClouds.toggled.connect(self.update_point_cloud_logger_state)
        self.widget_.saveImages.toggled.connect(self.update_image_logger_state)
        
    @Slot() 
    def update_point_cloud_logger_state(self):
      service_name = "/enable_log_pc"
      self.send_enable_logger_request(service_name, self.widget_.savePointClouds.isChecked())

    @Slot()
    def update_image_logger_state(self):
      service_name = "/enable_log_image"
      self.send_enable_logger_request(service_name, self.widget_.saveImages.isChecked())

    @Slot()
    def start_scan_hw_trigger(self):
      self.send_command_image_synchroniser_request("start", True, self.widget_.numberScans.value(), self.widget_.delayMs.value())

    @Slot()
    def start_scan_sw_trigger(self):
      self.send_command_image_synchroniser_request("start", False, self.widget_.numberScans.value(), self.widget_.delayMs.value())

    @Slot()
    def stop_scan_sw_trigger(self):
      self.send_command_image_synchroniser_request("stop", False, 0, 0)

    def get_projector_pattern_name_from_combo_box(self):
      combo_box_text = self.widget_.projectorPatternCombo.currentText()
      return "" if ((not combo_box_text) or combo_box_text == "Default") else combo_box_text

    def send_enable_logger_request(self, service_name, enable):
      try:
        srv_class = rosservice.get_service_class_by_name(service_name)
        srv_client = rospy.ServiceProxy(service_name, srv_class)
        request = srv_class._request_class()

        request.enable = enable

        success = srv_client(request)
        if not success:
            QMessageBox.warning(self.widget_, "Warning",
                                "Sending of service request to logger failed")
      except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
          rospy.logwarn(
              "[SLSensorScanningPlugin] Could not get class of service %s: %s" %
              (service_name, e))
          QMessageBox.warning(self.widget_, "Warning",
                              "Could not send service request to logger, make sure it is running.")

    def send_command_image_synchroniser_request(self, command, is_hw_trigger, number_scans, delay_ms):
      service_name = self.image_synchroniser_service_name
      try:
        srv_class = rosservice.get_service_class_by_name(service_name)
        srv_client = rospy.ServiceProxy(service_name, srv_class)
        request = srv_class._request_class()

        request.command = command
        request.pattern_name = self.get_projector_pattern_name_from_combo_box()
        request.is_hardware_trigger = is_hw_trigger
        request.number_scans = number_scans
        request.delay_ms = delay_ms

        success = srv_client(request)
        if not success:
            QMessageBox.warning(self.widget_, "Warning",
                                "Sending of service request to image synchroniser failed")
      except (rosservice.ROSServiceException, rosservice.ROSServiceIOException) as e:
          rospy.logwarn(
              "[ScanningGui] Could not get class of service %s: %s" %
              (service_name, e))
          QMessageBox.warning(self.widget_, "Warning",
                              "Could not send service request to image synchroniser, make sure it is running.")
                              
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
    