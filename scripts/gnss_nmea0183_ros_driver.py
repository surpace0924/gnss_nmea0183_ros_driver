#!/usr/bin/python3
# -*- coding: utf-8 -*-

from gnss_base import GnssDriverBase

import rospy
from sensor_msgs.msg import NavSatStatus, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import pynmea2
import serial


class GnssDriver(GnssDriverBase):
    def __init__(self) -> None:
        super().__init__()

        # To check if the GNSS data is received
        self.serial_connected_time = None
        self.gnss_data_received_time = None

        # Open serial port
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        if not self.ser.is_open:
            self.ser.open()
        rospy.loginfo(f"[gnss-driver] Serial port opened: {self.port} at {self.baudrate} baud")

        # Timer callback
        rospy.Timer(rospy.Duration(1.0 / self.sweep_rate), self.timer_callback)

    def __del__(self):
        """
        Destructor
        Close the serial port
        """
        if self.ser.is_open:
            self.ser.close()

    def timer_callback(self, event):
        # Read a line from the serial port
        line = None
        try:
            if self.ser.in_waiting:
                self.serial_connected_time = rospy.Time.now()
                line = self.ser.readline().decode("utf-8").strip()
        except serial.SerialException as e:
            rospy.logerr(f"[gnss-driver] Serial exception: {e}")
        except rospy.ROSInterruptException:
            return

        # Parse the NMEA message
        pynmea2_msg = None
        try:
            if line is not None:
                pynmea2_msg = pynmea2.parse(line)
        except pynmea2.ParseError as e:
            rospy.logerr(f"[gnss-driver] ParseError exception: {e}")

        # Publish the NavSatFix message
        if pynmea2_msg is not None:
            if pynmea2_msg.sentence_type == "GGA" and pynmea2_msg.gps_qual > 0:
                self.gnss_data_received_time = rospy.Time.now()

                # Generate the NavSatFix message
                gnss_fix_msg = NavSatFix()
                gnss_fix_msg.header.stamp = rospy.Time.now()
                gnss_fix_msg.header.frame_id = self.frame_id
                gnss_fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
                gnss_fix_msg.status.service = NavSatStatus.SERVICE_GPS
                gnss_fix_msg.latitude = pynmea2_msg.latitude
                gnss_fix_msg.longitude = pynmea2_msg.longitude
                gnss_fix_msg.altitude = pynmea2_msg.altitude
                gnss_fix_msg.position_covariance = [0.0] * 9
                gnss_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                # Publish the message
                self.pub_gnss_fix.publish(gnss_fix_msg)

        # Generate the DiagnosticArray message
        diagnostic_array_msg = DiagnosticArray()
        diagnostic_array_msg.header.stamp = rospy.Time.now()
        diagnostic_array_msg.header.frame_id = self.frame_id
        status = DiagnosticStatus()
        status.name = self.status_name
        if self.gnss_data_received_time is not None:
            if (rospy.Time.now() - self.gnss_data_received_time).to_sec() < self.timeout:
                status.level = DiagnosticStatus.OK
                status.message = 'Connected'
        elif self.serial_connected_time is not None:
            if (rospy.Time.now() - self.serial_connected_time).to_sec() < self.timeout:
                status.level = DiagnosticStatus.WARN
                status.message = 'No GNSS data'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Disconnected'
        diagnostic_array_msg.status.append(status)
        self.pub_diagnostics.publish(diagnostic_array_msg)


if __name__ == "__main__":
    try:
        gnss_driver = GnssDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
