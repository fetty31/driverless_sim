#include <ros/ros.h>

#include <dv_common/CarCommands.h>
#include <dv_common/CarState.h>
#include <dv_common/Float32Stamped.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <boost/bind.hpp>

ros::Publisher cmd_pub, pose_pub, twist_pub;

void state_callback(const dv_common::CarState::ConstPtr &data){

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.pose.position.x = data->x;
    pose_msg.pose.pose.position.y = data->y;
    pose_msg.pose.pose.position.z = 0.0;

    pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,data->yaw);

    pose_pub.publish(pose_msg);


    geometry_msgs::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = pose_msg.header.stamp;
    twist_msg.header.frame_id = pose_msg.header.frame_id;

    twist_msg.twist.twist.linear.x = data->vx;
    twist_msg.twist.twist.linear.y = data->vy;

    twist_msg.twist.twist.angular.z = data->r;

    twist_pub.publish(twist_msg);

}

void cmd_callback(const dv_common::Float32Stamped::ConstPtr &steer, const dv_common::Float32Stamped::ConstPtr &motor){
    dv_common::CarCommands msg;
    msg.header.stamp = ros::Time::now();
    msg.delta = steer->data;
    msg.throttle = motor->data;

    cmd_pub.publish(msg);
}

int main(int argc, char **argv){

    // Initialize ROS node
    ros::init(argc, argv, "dv_bypass");

    // Declare private NodeHandle obj
    ros::NodeHandle nh("~");

    // Read params
    std::string pose_topic, twist_topic, steer_topic, motor_topic, dv_cmd_topic, dv_state_topic;
    nh.param<std::string>("/RaceCar/Topics/Output/KinState",    dv_state_topic,     "/driverless/state");
    nh.param<std::string>("Topics/Output/Pose",                 pose_topic,         "/robot_pose_ekf/odom_combined");
    nh.param<std::string>("Topics/Output/Twist",                twist_topic,         "/robot_pose_ekf/odom_combined");

    nh.param<std::string>("/RaceCar/Topics/Input/Commands", dv_cmd_topic,       "/driverless/commands");
    nh.param<std::string>("Topics/Input/Steering",          steer_topic,        "/cmd/steer");
    nh.param<std::string>("Topics/Input/Motor",             motor_topic,        "/cmd/motor");

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe(dv_state_topic, 100, &state_callback);

    // Synchronized subscribers
    message_filters::Subscriber<dv_common::Float32Stamped> steer_sub(nh, steer_topic, 10);
    message_filters::Subscriber<dv_common::Float32Stamped> motor_sub(nh, motor_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<dv_common::Float32Stamped, dv_common::Float32Stamped> CmdSync;

    message_filters::Synchronizer<CmdSync> *sync;
    sync = new message_filters::Synchronizer<CmdSync>(CmdSync(10), steer_sub, motor_sub);
    sync->registerCallback(boost::bind(cmd_callback, _1, _2));

    // Publishers
    cmd_pub     = nh.advertise<dv_common::CarCommands>(dv_cmd_topic, 1);
    pose_pub    = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 10);
    twist_pub   = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(twist_topic, 10);

    // Start spinning :)
    ros::spin();

}