/** 
*	ROS package responsible for detecting the moving landing platform and publishing its position and the time of detection. 
*	Based on https://github.com/introlab/find-object/blob/master/src/ros/tf_example_node.cpp -- Mathieu Labbe - IntRoLab - Universite de Sherbrooke
*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <QtCore/QString>
#include "landing_pad_detector/Detected_position.h"

class PositionPublisherNode
{
public:
	PositionPublisherNode() :
		objFramePrefix_("object")
	{
		ros::NodeHandle pnh("~");
		pnh.param("target_frame_id", targetFrameId_, targetFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

		ros::NodeHandle nh;
		subs_ = nh.subscribe("objectsStamped", 1, &PositionPublisherNode::objectsDetectedCallback, this);
		pubs_ = nh.advertise<landing_pad_detector::Detected_position>("/landing_pad_detector/pad_position", 100);
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{
		if(msg->objects.data.size())
		{
			std::string targetFrameId = targetFrameId_;
			if(targetFrameId.empty())
			{
				targetFrameId = msg->header.frame_id;
			}
			char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];

				QString multiSuffix;
				if(id == previousId)
				{
					multiSuffix = QString("_") + multiSubId++;
				}
				else
				{
					multiSubId = 'b';
				}
				previousId = id;

				// "object_1", "object_1_b", "object_1_c", "object_2"
				std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();

				tf::StampedTransform pose;
				try
				{
					// Get transformation from "object_#" frame to target frame
					// The timestamp matches the one sent over TF
					tfListener_.lookupTransform(targetFrameId, objectFrameId, msg->header.stamp, pose);
				}
				catch(tf::TransformException & ex)
				{
					ROS_WARN("%s",ex.what());
					continue;
				}


				// Here "pose" is the position of the object "id" in target frame.
				ROS_INFO("Spotted landing pad %s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						objectFrameId.c_str(), targetFrameId.c_str(),
						pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
						pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());

				// Publish detected position and timestamp 
				position_msg_.x = pose.getOrigin().x();
				position_msg_.y = pose.getOrigin().y();
				position_msg_.z = pose.getOrigin().z();
				position_msg_.timeStamp = ros::Time::now();
				pubs_.publish(position_msg_);
			}
		}
	}

private:
	std::string targetFrameId_;
	std::string objFramePrefix_;
    ros::Subscriber subs_;
    tf::TransformListener tfListener_;
	ros::Publisher pubs_;
	landing_pad_detector::Detected_position position_msg_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "position_publisher_node");

    PositionPublisherNode sync;
    ros::spin();
}