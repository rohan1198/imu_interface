#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <serial/serial.h>
#include <jsoncpp/json/json.h>


int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "imu_reader");
  ros::NodeHandle nh("~");

  // Retrieve parameters for serial port, baud rate, and node rate
  std::string port;
  int baudrate, rate;
  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  nh.param("baudrate", baudrate, 115200);
  nh.param("rate", rate, 100);

  // Set up the publisher
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
  
  serial::Serial ser;

  try {
    // Connect to the given serial port and ser up the baud rate
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if (!ser.isOpen()) {
    ROS_ERROR_STREAM("Serial port not open");
    return -1;
  }

  // Confirm connection to port
  ROS_INFO_STREAM("Serial port initialized at " << ser.getPort());

  // Main loop
  ros::Rate loop_rate(rate);

  while (ros::ok()) {
    // Check if data is available to read
    if (ser.available()) {
      std::string line = ser.readline(65536, "\n");
      Json::Value root;
      Json::Reader reader;
      
      bool parsingSuccessful = reader.parse(line, root);
      if (!parsingSuccessful) {
        ROS_ERROR("Failed to parse JSON from serial data");
        continue;
      }

      // Make sure all the data is present in the JSON string
      if (!root["orientation"].isNull() &&
          !root["angular_velocity"].isNull() &&
          !root["linear_acceleration"].isNull()) {

            sensor_msgs::Imu imu_msg;

            // Populate data
            imu_msg.orientation.x = root["orientation"]["x"].asDouble();
            imu_msg.orientation.y = root["orientation"]["y"].asDouble();
            imu_msg.orientation.z = root["orientation"]["z"].asDouble();
            imu_msg.orientation.w = root["orientation"]["w"].asDouble();

            imu_msg.angular_velocity.x = root["angular_velocity"]["x"].asDouble();
            imu_msg.angular_velocity.y = root["angular_velocity"]["y"].asDouble();
            imu_msg.angular_velocity.z = root["angular_velocity"]["z"].asDouble();

            imu_msg.linear_acceleration.x = root["linear_acceleration"]["x"].asDouble();
            imu_msg.linear_acceleration.y = root["linear_acceleration"]["y"].asDouble();
            imu_msg.linear_acceleration.z = root["linear_acceleration"]["z"].asDouble();

            imu_pub.publish(imu_msg);
      }
      else {
        ROS_ERROR("JSON object missing required fields");
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}