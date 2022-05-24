/**
 * Based on PX4 offbroad control example described here: https://docs.px4.io/v1.12/en/ros/mavros_offboard.html  
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
mavros_msgs::SetMode offb_set_mode;

void state_cb(const mavros_msgs::State::ConstPtr & msg)
{
    current_state = *msg;
}

void changePosition(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    ROS_INFO("Received landing position");
    pose = *msg;
    offb_set_mode.request.custom_mode = "AUTO.LAND";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initiate_drone");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mission_controller/target_position", 10, changePosition);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    bool landingInitiated = false;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;
    

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && offb_set_mode.request.custom_mode != "AUTO.LAND"  &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if(offb_set_mode.request.custom_mode == "AUTO.LAND" && landingInitiated == false)
        {
            if(ros::Time::now() > pose.header.stamp ) // Time to land
            {
                set_mode_client.call(offb_set_mode);
                ROS_INFO("Landing at target position");
                last_request = ros::Time::now();
                landingInitiated = true;
            }
        }
        else 
        {
            if( !current_state.armed && offb_set_mode.request.custom_mode != "AUTO.LAND" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

