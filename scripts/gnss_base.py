#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray


class GnssDriverBase():
    def __init__(self) -> None:
        rospy.init_node("gnss_driver", anonymous=True)

        # Get parameters
        try:
            self.timeout = float(rospy.get_param("~timeout", 5.0))
            self.baudrate = int(rospy.get_param("~baudrate", 9600))
            self.port = str(rospy.get_param("~port", "/dev/ttyUSB0"))
            self.frame_id = str(rospy.get_param("~frame_id", "base_link"))
            self.sweep_rate = float(rospy.get_param("~sweep_rate", 10))
            self.status_name = str(rospy.get_param("~diagnostic_status_name", "GNSS"))
            
        except ValueError as e:
            rospy.logerr(f"[gnss-driver] ValueError exception: {e}")
            rospy.signal_shutdown("ValueError exception")

        # Publisher
        self.pub_gnss_fix = rospy.Publisher("gnss_fix", NavSatFix, queue_size=10)
        self.pub_diagnostics = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=10)
