#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatStatus, NavSatFix

import pynmea2
import serial


class GnssDriver():
    def __init__(self) -> None:
        rospy.init_node("gnss_driver", anonymous=True)

        # Get parameters
        self.baudrate = rospy.get_param("~baudrate", 9600)
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.sweep_rate = rospy.get_param("~sweep_rate", 10)

        # Set the sweep rate
        rate = rospy.Rate(self.sweep_rate)

        # Open serial port
        ser = serial.Serial(self.port, self.baudrate, timeout=1)
        if not ser.is_open:
            ser.open()
        rospy.loginfo(f"[gnss-driver] Serial port opened: {self.port} at {self.baudrate} baud")

        # Publisher
        pub_gnss_fix = rospy.Publisher("gnss_fix", NavSatFix, queue_size=10)

        # Main loop
        while not rospy.is_shutdown():
            # Read a line from the serial port
            line = None
            try:
                if ser.in_waiting:
                    line = ser.readline().decode("utf-8").strip()
                rate.sleep()
            except serial.SerialException as e:
                rospy.logerr(f"[gnss-driver] Serial exception: {e}")
            except rospy.ROSInterruptException:
                break

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
                    pub_gnss_fix.publish(gnss_fix_msg)

        # If the node is killed, close the serial port
        ser.close()


if __name__ == "__main__":
    try:
        gnss_driver = GnssDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
