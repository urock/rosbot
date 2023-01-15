#include "follower.hpp"
#include "model.hpp"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <string>
#include "AStarPlanner.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

Model::State rosbot_state{};
geometry_msgs::PoseStamped curr_state;
geometry_msgs::PoseStamped GOAL;


void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  // std::cout<< "model callback"<< std::endl;
  const auto &q = msg->pose[1].orientation;

  double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  rosbot_state.x = msg->pose[1].position.x;
  rosbot_state.y = msg->pose[1].position.y;
  rosbot_state.yaw = yaw;

  curr_state.pose.position.x = msg->pose[1].position.x;
  curr_state.pose.position.y = msg->pose[1].position.y;
  curr_state.pose.orientation.x = q.x;
  curr_state.pose.orientation.y = q.y;
  curr_state.pose.orientation.z = q.z;
  curr_state.pose.orientation.w = q.w;
}

int main(int argc, char **argv) 
{
	std::cout<< " Follower start "<<std::endl;

	GOAL.pose.position.x = 0.5;
	GOAL.pose.position.y = 1.;

	// std::vector<geometry_msgs::PointStamped> refPoints;
	// geometry_msgs::PointStamped point;
	// point.point.x = 1;
	// point.point.y = 0.;
	// refPoints.push_back(point);
	// point.point.x = 1.5;
	// point.point.y = 0.5;
	// refPoints.push_back(point);
	// point.point.x = 2.0;
	// point.point.y = 1;
	// refPoints.push_back(point);
	// point.point.x = -2.0;
	// point.point.y = 1;
	// refPoints.push_back(point);
	// point.point.x = -2.0;
	// point.point.y = -2.0;
	// refPoints.push_back(point);
	// std::cout<< refPoints.size() << std::endl;

	// rosparam set /follower/costmap/global_frame "odom"
	ros::init(argc, argv, "follower");
	
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  costmap_2d::Costmap2DROS costmap("costmap", tfBuffer);

	ros::NodeHandle node;

	ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
	ros::Publisher target_pub = node.advertise<geometry_msgs::PointStamped>("/clicked_point", 5);

	ros::Rate rate(1. / 1.);
	size_t index = 0;
	
	astar_plugin::AStarPlanner planner = astar_plugin::AStarPlanner("AStar", &costmap);

	// std::cout<<"START GOAL:"<<refPoints[index].point.x << " " << refPoints[index].point.y << std::endl;

	std::vector<geometry_msgs::PoseStamped> path;
	GOAL.header.frame_id = "odom";
	planner.makePlan(curr_state, GOAL, path);

	for(const auto& item : path)
	{
		std::cout<<item.pose.position.x << " "<< item.pose.position.y << std::endl;
	}

	geometry_msgs::PointStamped msg;
	msg.point = path[index].pose.position;
	target_pub.publish(msg);
	
	while (ros::ok()) {
		ros::spinOnce();

		geometry_msgs::PointStamped msg;
		msg.point = path[index].pose.position;

		if (rosbot_state.distXY({path[index].pose.position.x, path[index].pose.position.y, 0.}) < 0.1) 
		{
			std::cout<<"REACHED GOAL:"<<path[index].pose.position.x <<" "<< path[index].pose.position.y<<std::endl;
			index++;
			std::cout<<"ROBOT STATE"<<rosbot_state.x << " " << rosbot_state.y << std::endl;
			std::cout<<"NEXT GOAL:"<<path[index].pose.position.x<<" "<<path[index].pose.position.y<< std::endl;

			if (index > path.size() - 1)
				return 0;
			
			target_pub.publish(msg);

		}
		
		target_pub.publish(msg);
		// target_pub.publish(msg);

		// target_pub.publish(refPoints[index]);
		// if (rosbot_state.distXY({refPoints[index].point.x, refPoints[index].point.y, 0.}) < 0.2) 
		// {
		// 	std::cout<<"REACHED GOAL:"<<refPoints[index].point.x <<" "<< refPoints[index].point.y<<std::endl;
		// 	index++;
		// 	std::cout<<"ROBOT STATE"<<rosbot_state.x << " " << rosbot_state.y << std::endl;
		// 	std::cout<<"NEXT GOAL:"<<refPoints[index].point.x<<" "<<refPoints[index].point.y<< std::endl;

		// 	if (index > refPoints.size() - 1)
		// 		return 0;
		// }


		rate.sleep();

	}

	return 0;

}