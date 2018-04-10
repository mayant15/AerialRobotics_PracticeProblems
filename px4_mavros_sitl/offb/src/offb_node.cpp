/**
 * @file offb_node.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

#include <cmath>

#define THRESHOLD 0.1
#define STEP 0.1

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

float distance(const geometry_msgs::PoseStamped::ConstPtr& curr_pose,
        const geometry_msgs::PoseStamped::ConstPtr& reqd_pose){
    float x1 = curr_pose->pose.position.x;
    float y1 = curr_pose->pose.position.y;
    float x2 = reqd_pose->pose.position.x;
    float y2 = reqd_pose->pose.position.y;

    return sqrt((x1 - x2)*(x1 - x2) + (y1 -  y2)*(y1 - y2));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");


    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);


    // wait for FCU connection
    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::PoseStamped reqd_pose;
    reqd_pose.pose.position.x = 0;
    reqd_pose.pose.position.y = 0;
    reqd_pose.pose.position.z = 2;


    // send a few setpoints before starting
    for (int i = 100; i > 0 && ros::ok(); i--) {
        local_pos_pub.publish(reqd_pose);
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    ros::Time last_request = ros::Time::now();


    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(0.5))) {
            if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard Enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Check distance from current position to required position
        // Update required pose if near
        if (current_state.mode == "OFFBOARD" && current_state.armed &&
                distance(&current_pose, &reqd_pose) < THRESHOLD) {
            reqd_pose.pose.position.x += STEP;
            reqd_pose.pose.position.y += sin(reqd_pose.pose.position.x);
            ROS_INFO("POSE UPDATED");
        }


        local_pos_pub.publish(reqd_pose);
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
