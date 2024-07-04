#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy

import pynmea2
import serial


class GnssDriver():
    def __init__(self) -> None:
        rospy.init_node('serial_receiver', anonymous=True)
        rate = rospy.Rate(10)  # 10Hz

        # sudo chmod a+rw /dev/ttyUSB0
        port = '/dev/ttyUSB0'
        baudrate = 9600
        ser = serial.Serial(port, baudrate, timeout=1)

        if not ser.is_open:
            ser.open()

        rospy.loginfo(f"Serial port opened: {port} at {baudrate} baud")

        while not rospy.is_shutdown():
            line = None
            try:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8').strip()
                rate.sleep()
            except serial.SerialException as e:
                rospy.logerr(f"Serial exception: {e}")
            except rospy.ROSInterruptException:
                break

            msg = None
            try:
                if line is not None:
                    msg = pynmea2.parse(line)
            except pynmea2.ParseError as e:
                rospy.logerr(f"ParseError exception: {e}")
            if msg is not None:
                if msg.sentence_type == 'GGA':
                    rospy.loginfo(int(msg.num_sats))
                    rospy.loginfo(float(msg.latitude))
                    rospy.loginfo(float(msg.altitude))
                    rospy.loginfo(int(msg.gps_qual))
                    rospy.loginfo(float(msg.longitude))
                    rospy.loginfo("")

        ser.close()
        rospy.loginfo("Serial port closed")


if __name__ == '__main__':
    try:
        gnss_driver = GnssDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
