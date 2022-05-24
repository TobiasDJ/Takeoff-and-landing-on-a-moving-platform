
/**
*	ROS package responsible for estimating velocity based on detected landing pad positions. The velocity is measured using a Kalman filter
*	to provide reasonable estimations accounting for process noise and fluctuations in measurement accuracy
*/

#include <ros/ros.h>
#include "landing_pad_detector/Detected_position.h"
#include <geometry_msgs/PoseStamped.h>
#include "kalman_filter.hpp"
#include "path_estimator/Estimated_path.h"
#include <cmath>
#include <gazebo_msgs/ModelStates.h>

class PathEstimatorNode
{
public:
	PathEstimatorNode()
	{	
		ros::NodeHandle nh;
		landing_pad_subs = nh.subscribe("/landing_pad_detector/pad_position", 1, &PathEstimatorNode::objectsDetectedCallback, this);
		drone_local_position_subs = nh.subscribe("/mavros/local_position/pose", 1, &PathEstimatorNode::droneLocalPositionCallback, this);
		landing_gazebo_subs = nh.subscribe("/gazebo/model_states/", 1, &PathEstimatorNode::modelStateCallback, this);
		pubs_ = nh.advertise<path_estimator::Estimated_path>("/path_estimator/pad_path", 100);

		// Input: x -- Initial_prediction, r -- Measurement uncertainty, q -- Process noise covariance, p -- estimated_error_coveriance 
		kalmanFilterX = KalmanFilter(0.5, 33, 0.15, 1089); 
		kalmanFilterY = KalmanFilter(0.5, 33, 0.15, 1089);
		kalmanFilterZ = KalmanFilter(0.001, 33, 0.15, 1089);
		firstDetection = true;
		measurementsNeeded = 1;
		counter = 0;
		calibrationX = 0.5;
		calibrationY = 0.2;
	}

	void objectsDetectedCallback(const landing_pad_detector::Detected_position & msg)
	{
        //ROS_INFO("[%d] Received landing pad position [x,y,z]: [%lf,%lf,%lf] at time: [%lf] -- true position is: [%lf,%lf,%lf]", counter, lastDronePositionX - msg.x + calibrationX, lastDronePositionY - msg.y + calibrationY, msg.z - lastDronePositionZ, msg.timeStamp.toSec(), truePadPositionX, truePadPositionY, truePadPositionZ);
		calculateVelocity(msg.x , msg.y , msg.z, msg.timeStamp);
		//counter = counter + 1;
	}

	void droneLocalPositionCallback(const geometry_msgs::PoseStamped & msg)
	{
        //ROS_INFO("Received drone position [x,y,z]: [%f,%f,%f] at time: [%f]", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.stamp.toSec());
		lastDronePositionX = msg.pose.position.x;
		lastDronePositionY = msg.pose.position.y;
		lastDronePositionZ = msg.pose.position.z;
	}

	void modelStateCallback(const gazebo_msgs::ModelStates & msg)
	{
        //ROS_INFO(" Received model state for landing pad [x,y,z]: [%lf,%lf,%lf] ", msg.pose[1].position.x, msg.pose[1].position.y, msg.pose[1].position.z);
		truePadPositionX = msg.pose[1].position.x;
		truePadPositionY = msg.pose[1].position.y;
		truePadPositionZ = msg.pose[1].position.z;

	}



	void calculateVelocity(double detectedX, double detectedY, double detectedZ, ros::Time detectedTimestamp)
	{
		// Relative to drone position
		detectedX = lastDronePositionX - detectedX + calibrationX;
		detectedY = lastDronePositionY - detectedY + calibrationY;
		detectedZ = lastDronePositionZ - detectedZ;

		if(firstDetection)
		{
			lastLandingPadPositionX = detectedX;
			lastLandingPadPositionY = detectedY;
			lastLandingPadPositionZ = detectedZ;
			lastTimeStamp = detectedTimestamp;
			firstDetection = false;
		}
		else
		{
			deltaX = detectedX - lastLandingPadPositionX;
			deltaY = detectedY - lastLandingPadPositionY;
			deltaZ = detectedZ - lastLandingPadPositionZ;
			deltaT = detectedTimestamp.toSec() - lastTimeStamp.toSec();	
			ROS_INFO("Calculated velocity [vx,vy,vz]: [%lf,%lf,%lf]", deltaX/deltaT + 0.075, deltaY/deltaT, deltaZ/deltaT);
			
			lastLandingPadPositionX = detectedX;
			lastLandingPadPositionY = detectedY;
			lastLandingPadPositionZ = detectedZ;
			lastTimeStamp = detectedTimestamp;

			kalmanVelocity(deltaX/deltaT + 0.075, deltaY/deltaT, deltaZ/deltaT);

		}

	}

	void kalmanVelocity(double velocityX, double velocityY, double velocityZ)
	{
		kalmanFilteredX = kalmanFilterX.kalmanIteration(velocityX);
		kalmanFilteredY = kalmanFilterY.kalmanIteration(velocityY);
		kalmanFilteredZ = kalmanFilterZ.kalmanIteration(velocityZ);
		ROS_INFO("Calculated kalman velocity [vx]: [%lf] with k: [%lf], p: [%lf], q: [%lf], r: [%lf]", kalmanFilteredX, kalmanFilterX.getKalmanGain(), kalmanFilterX.getEstimationErrorCovariance(), kalmanFilterX.getProcessNoiseCoveriance(), kalmanFilterX.getMeasurementNoiseCoveriance());
		ROS_INFO("Calculated kalman velocity [vy]: [%lf] with k: [%lf], p: [%lf], q: [%lf], r: [%lf]", kalmanFilteredY, kalmanFilterY.getKalmanGain(), kalmanFilterY.getEstimationErrorCovariance(), kalmanFilterY.getProcessNoiseCoveriance(), kalmanFilterY.getMeasurementNoiseCoveriance());
		//ROS_INFO("Calculated kalman velocity [vz]: [%lf] with k: [%lf], p: [%lf], q: [%lf], r: [%lf]", kalmanFilteredZ, kalmanFilterZ.getKalmanGain(), kalmanFilterZ.getEstimationErrorCovariance(), kalmanFilterZ.getProcessNoiseCoveriance(), kalmanFilterZ.getMeasurementNoiseCoveriance());


		if(kalmanFilterX.getEstimationErrorCovariance() < 3) // if accepted level of certainty
		{
			if(measurementsNeeded == 4)
			{
				// Publish detected position and timestamp 
				planned_path_msg_.x = lastLandingPadPositionX;
				planned_path_msg_.y = lastLandingPadPositionY;
				planned_path_msg_.z = lastLandingPadPositionZ;
				planned_path_msg_.droneX = lastDronePositionX;
				planned_path_msg_.droneY = lastDronePositionY;
				planned_path_msg_.droneZ = lastDronePositionZ;
				planned_path_msg_.vx = kalmanFilteredX;
				planned_path_msg_.vy = kalmanFilteredY;
				planned_path_msg_.vz = kalmanFilteredZ;
				planned_path_msg_.timeStamp = lastTimeStamp;
				pubs_.publish(planned_path_msg_);
			}
			measurementsNeeded = measurementsNeeded + 1;
		}
	}



private:
	// Subscriber and publisher
    ros::Subscriber landing_pad_subs, drone_local_position_subs, landing_gazebo_subs;
	ros::Publisher pubs_;
	// Landing pad position
	double lastLandingPadPositionX, lastLandingPadPositionY, lastLandingPadPositionZ;
	double truePadPositionX, truePadPositionY, truePadPositionZ;
	ros::Time lastTimeStamp;
	// Drone position
	double lastDronePositionX, lastDronePositionY, lastDronePositionZ;
	// Difference in position and time
	double deltaX, deltaY, deltaZ, deltaT;
	// Dont trigger first read
	bool firstDetection;
	// Kalman
	KalmanFilter kalmanFilterX, kalmanFilterY, kalmanFilterZ;
	double kalmanFilteredX, kalmanFilteredY, kalmanFilteredZ;
	// Velocity and position message
	path_estimator::Estimated_path planned_path_msg_;
	// Counter
	int measurementsNeeded, counter;
	// Calibration
	double calibrationX, calibrationY;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "path_estimator_node");
    PathEstimatorNode sync;
    ros::spin();
}