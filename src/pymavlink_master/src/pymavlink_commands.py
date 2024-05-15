#!/usr/bin/python3
from optparse import OptionParser

import rospy
from custom_msgs.msg import commands, telemetry
from pymavlink import mavutil


class Basic:
    def __init__(self, port, mode):
        self.mode = mode
        self.pixhawk_port = port
        self.arm_state = False
        self.master = mavutil.mavlink_connection(self.pixhawk_port, baud=115200)
        self.channel_ary = [1500] * 8
        self.telem_msg = telemetry()

        self.master.wait_heartbeat()

        self.thruster_subs = rospy.Subscriber(
            "/master/commands", commands, self.__callback__, queue_size=1
        )
        self.telemetry_pub = rospy.Publisher(
            "/master/telemetry", telemetry, queue_size=1
        )

    def __callback__(self, msg):
        self.channel_ary[0] = msg.pitch
        self.channel_ary[1] = msg.roll
        self.channel_ary[2] = msg.thrust
        self.channel_ary[3] = msg.yaw
        self.channel_ary[4] = msg.forward
        self.channel_ary[5] = msg.lateral
        self.channel_ary[6] = msg.servo1
        self.channel_ary[7] = msg.servo2

        if msg.arm and not self.arm_state:
            self.arm()
            self.arm_state = True
        elif not msg.arm and self.arm_state:
            self.disarm()
            self.arm_state = False

        if self.mode != msg.mode:
            if not self.arm_state:
                self.mode = msg.mode
                self.mode_switch()
            else:
                rospy.logwarn("Disarm AUV to change modes")

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        rospy.loginfo("Arm command sent to Pixhawk")

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        rospy.loginfo("Disarm command sent to Pixhawk")

    def mode_switch(self):
        mode_mapping = self.master.mode_mapping()
        if self.mode not in mode_mapping:
            rospy.logerr(f"Unknown mode: {self.mode}. Try: {list(mode_mapping.keys())}")
            exit(1)
        mode_id = mode_mapping[self.mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        rospy.loginfo(f"Mode switched to: {self.mode}")

    def set_rc_channel_pwm(self, id, pwm):
        if 1 <= id <= 8:
            rc_channel_values = [65535] * 8
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values,
            )
        else:
            rospy.logwarn("Channel does not exist.")

    def actuate(self):
        for i in range(1, 9):
            self.set_rc_channel_pwm(i, int(self.channel_ary[i - 1]))

    def request_message_interval(self, message_id: int, frequency_hz: float):
        interval = 1e6 / frequency_hz
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval,
            0,
            0,
            0,
            0,
            0,
        )
        response = self.master.recv_match(type="COMMAND_ACK", blocking=True)
        if (
            response
            and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        ):
            rospy.loginfo("Command Accepted")
        else:
            rospy.logerr("Command Failed")

    def telem_publish_func(self, imu_msg, attitude_msg, vfr_hud_msg, depth_msg):
        self.telem_msg.timestamp = imu_msg.time_boot_ms
        self.telem_msg.internal_pressure = vfr_hud_msg.alt
        self.telem_msg.external_pressure = depth_msg.press_abs
        self.telem_msg.heading = vfr_hud_msg.heading
        self.telem_msg.imu_gyro_x = imu_msg.xgyro
        self.telem_msg.imu_gyro_y = imu_msg.ygyro
        self.telem_msg.imu_gyro_z = imu_msg.zgyro
        self.telem_msg.imu_gyro_compass_x = imu_msg.xmag
        self.telem_msg.imu_gyro_compass_y = imu_msg.ymag
        self.telem_msg.q1 = attitude_msg.q1
        self.telem_msg.q2 = attitude_msg.q2
        self.telem_msg.q3 = attitude_msg.q3
        self.telem_msg.q4 = attitude_msg.q4
        self.telem_msg.rollspeed = attitude_msg.rollspeed
        self.telem_msg.pitchspeed = attitude_msg.pitchspeed
        self.telem_msg.yawspeed = attitude_msg.yawspeed
        self.telemetry_pub.publish(self.telem_msg)


if __name__ == "__main__":
    rospy.init_node("pymav_master", anonymous=True)

    parser = OptionParser(description="description for prog")
    parser.add_option(
        "-p",
        "--port",
        dest="port_addr",
        default="/dev/Pixhawk",
        help="Pass Pixhawk Port Address",
        metavar="VAR",
    )
    parser.add_option(
        "-m",
        "--mode",
        dest="auv_mode",
        default="STABILIZE",
        help="Pass Pixhawk Mode",
        metavar="VAR",
    )
    (options, args) = parser.parse_args()
    obj = Basic(options.port_addr, options.auv_mode)

    obj.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2000)
    rospy.spin()
