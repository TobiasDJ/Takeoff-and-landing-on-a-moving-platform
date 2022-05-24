/**
*    Aligns the path of the moving platform with the drone to initiate landing. 
*/

#include <ros/ros.h>
#include "path_estimator/Estimated_path.h"
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class MissionControllerNode
{
public:
	MissionControllerNode()
	{
		ros::NodeHandle nh;
		subs_ = nh.subscribe("/path_estimator/pad_path", 1, &MissionControllerNode::pathEstimatedCallback, this);
		pubs_ = nh.advertise<geometry_msgs::PoseStamped>("/mission_controller/target_position", 10);
	}

	void pathEstimatedCallback(const path_estimator::Estimated_path & msg)
	{
		ROS_INFO("Received pad path with position: [x,y,z] [%lf,%lf,%lf] and velocity: [vx,vy,vz] [%lf,%lf,%lf] at time: [%lf]", msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.timeStamp.toSec());
        
        double timeNeeded = determineTimeNeededToLand(msg.vx, msg.vy);

        futureLandingPadPositionX = msg.x + (msg.vx * timeNeeded);
        futureLandingPadPositionY = msg.y + (msg.vy * timeNeeded);
        futureLandingPadPositionZ = msg.z + (msg.vz * timeNeeded);

        ROS_INFO("In %lf secs the pad is at position [x,y,z] [%lf,%lf,%lf]", timeNeeded, futureLandingPadPositionX, futureLandingPadPositionY, futureLandingPadPositionZ);

        distance = sqrt(pow(futureLandingPadPositionX - msg.droneX, 2) +  pow(futureLandingPadPositionY - msg.droneY, 2) + pow(futureLandingPadPositionZ - msg.droneZ, 2));
        
        ROS_INFO("Distance is: [x,y,z] [d] [%lf,%lf,%lf] [%lf] at time: [%lf]", futureLandingPadPositionX - msg.droneX , futureLandingPadPositionY - msg.droneY, futureLandingPadPositionZ - msg.droneZ, distance, msg.timeStamp.toSec());

        double landingTime = calculateLandingTime(distance);

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = futureLandingPadPositionX;
        pose.pose.position.y = futureLandingPadPositionY;
        pose.pose.position.z = 0.3; //futureLandingPadPositionZ for now Z is ignored
        pose.header.stamp = ros::Time().now() + ros::Duration(landingTime);
        ROS_INFO("Published new position and time to be there");
        pubs_.publish(pose);    
    }

    double calculateLandingTime(double distance)
    {   
       double timeNeededToLand = 0.75; 
       return (distance / 2) + timeNeededToLand;
    }

    // Determine time intervals needed and what time to calculate distance 
    double determineTimeNeededToLand(double velocityX, double velocityY)
    {
        double absoluteSumVelocity = abs(velocityX) + abs(velocityY);
        double timeNeededToLand;

        if(absoluteSumVelocity > 0.1 && absoluteSumVelocity < 1.1 )
        {
            timeNeededToLand = 7.5;
        }
        else if(absoluteSumVelocity > 1.1 && absoluteSumVelocity < 2.1 )
        {
            timeNeededToLand = 8.5;
        }
        else if(absoluteSumVelocity > 2.1 && absoluteSumVelocity < 3.1 )
        {
            timeNeededToLand = 10;
        }
        else // Default
        {
            timeNeededToLand = 8.5;
        }

        return timeNeededToLand;
    }

private:
    ros::Subscriber subs_;
	ros::Publisher pubs_;
	path_estimator::Estimated_path path_msg_;

    // Landing pad position
	double futureLandingPadPositionX, futureLandingPadPositionY, futureLandingPadPositionZ;
    // Distance 
    double distance;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "mission_controller_node");

    MissionControllerNode sync;
    ros::spin();
}