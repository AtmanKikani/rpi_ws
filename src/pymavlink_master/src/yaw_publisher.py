#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32


def main():
    rospy.init_node("arduino_yaw_publisher", anonymous=True)
    pub = rospy.Publisher("mira/heading", Int32, queue_size=10)
    rate = rospy.Rate(10)

    serial_port = rospy.get_param("~serial_port", "/dev/Mega")
    serial_baud = rospy.get_param("~serial_baud", 115200)
    ser = serial.Serial(serial_port, serial_baud)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode("utf-8").strip()
                yaw_value = int(float(line))
                pub.publish(yaw_value)
            except ValueError as e:
                rospy.logwarn(f"ValueError: {e} with line '{line}'")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
