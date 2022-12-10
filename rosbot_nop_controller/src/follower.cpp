#include "follower.hpp"
#include "model.hpp"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include <cmath>
#include <iostream>

Model::State rosbot_state{};

void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  // std::cout<< "model callback"<< std::endl;
  const auto &q = msg->pose[1].orientation;

  double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  rosbot_state.x = msg->pose[1].position.x;
  rosbot_state.y = msg->pose[1].position.y;
  rosbot_state.yaw = yaw;
}

int main(int argc, char **argv) 
{
	std::cout<< " Follower start "<<std::endl;


	std::vector<geometry_msgs::PointStamped> refPoints;

	geometry_msgs::PointStamped point;
	point.point.x = 1;
	point.point.y = 0.;
	refPoints.push_back(point);
	point.point.x = 1.5;
	point.point.y = 0.5;
	refPoints.push_back(point);
	point.point.x = 2.0;
	point.point.y = 1;
	refPoints.push_back(point);
	point.point.x = -2.0;
	point.point.y = 1;
	refPoints.push_back(point);
	point.point.x = -2.0;
	point.point.y = -2.0;
	refPoints.push_back(point);

	std::cout<< refPoints.size() << std::endl;

	ros::init(argc, argv, "follower");
	ros::NodeHandle node;

	ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
	ros::Publisher target_pub = node.advertise<geometry_msgs::PointStamped>("/clicked_point", 5);
	
	ros::Rate rate(1. / 1.);
	size_t index = 0;

	std::cout<<"START GOAL:"<<refPoints[index].point.x << " " << refPoints[index].point.y << std::endl;
	while (ros::ok()) {
		ros::spinOnce();

		target_pub.publish(refPoints[index]);

		if (rosbot_state.distXY({refPoints[index].point.x, refPoints[index].point.y, 0.}) < 0.2) 
		{
			std::cout<<"REACHED GOAL:"<<refPoints[index].point.x <<" "<< refPoints[index].point.y<<std::endl;
			index++;
			std::cout<<"ROBOT STATE"<<rosbot_state.x << " " << rosbot_state.y << std::endl;
			std::cout<<"NEXT GOAL:"<<refPoints[index].point.x<<" "<<refPoints[index].point.y<< std::endl;

			if (index > refPoints.size() - 1)
				return 0;
		}


		rate.sleep();

	}

	return 0;

}