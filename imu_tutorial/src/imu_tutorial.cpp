#include <ros/ros.h>
#include "string.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

sensor_msgs::Imu msg_imu;

void imuimu_cb(sensor_msgs::Imu msg){

    msg_imu.header = msg.header;
    msg_imu.angular_velocity.x = msg.angular_velocity.x;
    msg_imu.angular_velocity.y = msg.angular_velocity.y;
    msg_imu.angular_velocity.z = msg.angular_velocity.z;
    msg_imu.linear_acceleration.x = msg.linear_acceleration.x;
    msg_imu.linear_acceleration.y = msg.linear_acceleration.y;
    msg_imu.linear_acceleration.z = msg.linear_acceleration.z;

}

void imupose_cb(geometry_msgs::PoseStamped msg){

    msg_imu.orientation.w = msg.pose.orientation.w;
    msg_imu.orientation.x = msg.pose.orientation.x;
    msg_imu.orientation.y = msg.pose.orientation.y;
    msg_imu.orientation.z = msg.pose.orientation.z;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "imu_tutorial_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_imuimu = nh.subscribe("/imu/imu", 1, imuimu_cb);
    ros::Subscriber sub_imupose = nh.subscribe("/imu/pose", 1, imupose_cb);
    ros::Publisher pub_imu_tutorial = nh.advertise<sensor_msgs::Imu>("/imuTopic",1);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        pub_imu_tutorial.publish(msg_imu);
        loop_rate.sleep();
    }
    return 0;
}
