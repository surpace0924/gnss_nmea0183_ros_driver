#!/usr/bin/python3
# -*- coding: utf-8 -*-


from gnss_base import GnssDriverBase

import rospy
from sensor_msgs.msg import NavSatStatus, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class GnssDriverDummy(GnssDriverBase):
    def __init__(self) -> None:
        super().__init__()

        # Get Dummy values
        try:
            self.dummy_latitude = float(rospy.get_param("~dummy_latitude", 35.689))
            self.dummy_longitude = float(rospy.get_param("~dummy_longitude", 139.692))
            self.dummy_altitude = float(rospy.get_param("~dummy_altitude", 40))
            self.publish_rate = float(rospy.get_param("~publish_rate", 1))
        except ValueError as e:
            rospy.logerr(f"[gnss-driver] ValueError exception: {e}")
            rospy.signal_shutdown("ValueError exception")

        # Timer callback
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

    def timer_callback(self, event):
        """
        Timer callback
        Publish the dummy NavSatFix message
        """

        # Generate the NavSatFix message
        gnss_fix_msg = NavSatFix()
        gnss_fix_msg.header.stamp = rospy.Time.now()
        gnss_fix_msg.header.frame_id = self.frame_id
        gnss_fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        gnss_fix_msg.status.service = NavSatStatus.SERVICE_GPS
        gnss_fix_msg.latitude = self.dummy_latitude
        gnss_fix_msg.longitude = self.dummy_longitude
        gnss_fix_msg.altitude = self.dummy_altitude
        gnss_fix_msg.position_covariance = [0.0] * 9
        gnss_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # Publish the message
        self.pub_gnss_fix.publish(gnss_fix_msg)

        # Generate the DiagnosticArray message
        diagnostic_array_msg = DiagnosticArray()
        diagnostic_array_msg.header.stamp = rospy.Time.now()
        diagnostic_array_msg.header.frame_id = self.frame_id
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = self.status_name
        status.message = 'Connected'
        diagnostic_array_msg.status.append(status)
        self.pub_diagnostics.publish(diagnostic_array_msg)


if __name__ == "__main__":
    try:
        gnss_driver = GnssDriverDummy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
