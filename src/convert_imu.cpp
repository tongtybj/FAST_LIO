#include <ros/ros.h>
#include <spinal/Imu.h>
#include <sensor_msgs/Imu.h>

void imuCallback(const spinal::Imu::ConstPtr& msg, ros::Publisher& pub) {
    // Create a sensor_msgs::Imu message
    sensor_msgs::Imu imu_msg;

    // Copy the timestamp
    imu_msg.header.stamp = msg->stamp;

    // Set the frame ID to "livox_frame"
    imu_msg.header.frame_id = "livox_frame";

    // Copy acceleration data
    imu_msg.linear_acceleration.x = msg->acc_data[0];
    imu_msg.linear_acceleration.y = msg->acc_data[1];
    imu_msg.linear_acceleration.z = msg->acc_data[2]/9.8;

    // Copy angular velocity data
    imu_msg.angular_velocity.x = msg->gyro_data[0];
    imu_msg.angular_velocity.y = msg->gyro_data[1];
    imu_msg.angular_velocity.z = msg->gyro_data[2];

    // Magnetic data and angles are ignored as sensor_msgs::Imu does not include these fields

    // Initialize orientation (modify if transformation is required)
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    // Publish the message
    pub.publish(imu_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_converter");
    ros::NodeHandle nh;

    // Create a publisher for the converted IMU data
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("livox/imu", 10);

    // Create a subscriber to the input IMU data
    ros::Subscriber imu_sub = nh.subscribe<spinal::Imu>(
        "imu", 10, boost::bind(imuCallback, _1, boost::ref(imu_pub))
    );

    ros::spin();
    return 0;
}
