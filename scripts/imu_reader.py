#!/usr/bin/env python3

import rospy
import serial
import json
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_RATE = 100


def talker():
    """
    Main function to read IMU data and publish as a ROS Topic
    """
    rospy.init_node("imu_reader", anonymous=True)

    port = rospy.get_param("~imu_data", DEFAULT_PORT)
    baudrate = rospy.get_param("~baudrate", DEFAULT_BAUDRATE)
    node_rate = rospy.get_param("~rate", DEFAULT_RATE)

    pub = rospy.Publisher("imu_data", Imu, queue_size=10)
    rate = rospy.Rate(node_rate)

    ser = serial.Serial(port, baudrate, timeout=1)

    with ser:
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode("utf-8").rstrip()
                except UnicodeDecodeError:
                    rospy.logerr("Could not decode line - possible data corruption")
                    continue

                try:
                    data = json.loads(line)

                    if "orientation" in data and "angular_velocity" in data and "linear_acceleration" in data:
                        imu_msg = Imu()

                        imu_msg.orientation = Quaternion(x=data["orientation"]["x"],
                                                         y=data["orientation"]["y"],
                                                         z=data["orientation"]["z"],
                                                         w=data["orientation"]["w"])
                        imu_msg.angular_velocity = Vector3(x=data["angular_velocity"]["x"],
                                                           y=data["angular_velocity"]["y"],
                                                           z=data["angular_velocity"]["z"])
                        imu_msg.linear_acceleration = Vector3(x=data["linear_acceleration"]["x"],
                                                              y=data["linear_acceleration"]["y"],
                                                              z=data["linear_acceleration"]["z"])
                        
                        pub.publish(imu_msg)
                    
                    else:
                        rospy.logerr("JSON data is missing required fields")
                
                except json.JSONDecodeError:
                    rospy.logerr("Could not parse JSON")
            
            if rospy.is_shutdown():
                break

            rate.sleep()



if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("IMU reader node interrupted. Shutting down...")
