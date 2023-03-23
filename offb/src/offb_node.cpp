/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// #include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/ActuatorMotors.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    // ros::Publisher act_motors_pub = nh.advertise<mavros_msgs::ActuatorControl>
    //         ("/mavros/actuator_control", 10);
    ros::Publisher act_motors_pub = nh.advertise<mavros_msgs::ActuatorMotors>
            ("mavros/actuator_motors/actuator_motors_sub", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 10;
    // pose.pose.position.y = 30;
    // pose.pose.position.z = 100;

    // // send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    mavros_msgs::ActuatorMotors motors_val;
    motors_val.header.stamp = ros::Time::now();
    motors_val.control[0] = 0.6;
    motors_val.control[1] = 0.6;
    motors_val.control[2] = 0.6;
    motors_val.control[3] = 0.6;
    // motors_val.control[4] = 0.0;
    // motors_val.control[5] = 0.0;
    // motors_val.control[6] = 0.0;
    // motors_val.control[7] = 0.0;
    // motors_val.control[8] = 0.0;
    // motors_val.control[9] = 0.0;
    // motors_val.control[10] =0.0;
    // motors_val.control[11] =0.0;

    for(int i = 100; ros::ok() && i > 0; --i){
        ROS_INFO("### IN THE FOR LOOP ###");
        act_motors_pub.publish(motors_val);
        ros::spinOnce();
        rate.sleep();
    }


    // mavros_msgs::ActuatorControl motors_val;
    // motors_val.header.stamp = ros::Time::now();
    // motors_val.controls[0] = 1;
    // motors_val.controls[1] = 1;
    // motors_val.controls[2] = 1;
    // motors_val.controls[3] = 1;
    // motors_val.controls[4] = 1;
    // motors_val.controls[5] = 1;
    // motors_val.controls[6] = 1;
    // motors_val.controls[7] = 1;

    // for(int i = 100; ros::ok() && i > 0; --i){
    //     ROS_INFO("### IN THE FOR LOOP ###");
    //     act_motors_pub.publish(motors_val);
    //     ros::spinOnce();
    //     rate.sleep();
    // }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("### IN OFFBOARD ###");
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                ROS_INFO("### IN ARMING ###");
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub.publish(pose);
        // ROS_INFO(current_state);
        act_motors_pub.publish(motors_val);

        //ROS_INFO("### PUBLISHED ONCE ###");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
