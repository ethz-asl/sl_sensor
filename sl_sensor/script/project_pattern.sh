#!/bin/bash

rosservice call /command_image_synchroniser "{command: 'start', pattern_name: '', is_hardware_trigger: true, number_scans: 0, delay_ms: 500}" 
exit 0
