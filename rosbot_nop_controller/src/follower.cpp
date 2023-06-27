#include "follower.hpp"


geometry_msgs::PoseStamped GOAL;
geometry_msgs::PoseStamped curr_state;

void model_state_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  // std::cout<< "model callback"<< std::endl;
  const auto &q = msg->pose[1].orientation;

  double siny_cosp = 2. * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1. - 2. * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  curr_state.pose.position.x = msg->pose[1].position.x;
  curr_state.pose.position.y = msg->pose[1].position.y;
  curr_state.pose.orientation.x = q.x;
  curr_state.pose.orientation.y = q.y;
  curr_state.pose.orientation.z = q.z;
  curr_state.pose.orientation.w = q.w;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "parking_node");

	GOAL.header.frame_id = "odom";
	ros::param::set("/parking_node/costmap/resolution", 0.2);
	ros::param::set("/parking_node/costmap/global_frame", "odom");
	// ros::param::set("/parking_node/costmap/observation_sources", "camera/depth/image_raw");
  
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    costmap_2d::Costmap2DROS costmap("costmap", tfBuffer);

	ros::NodeHandle node;
	ros::Subscriber models_sub = node.subscribe("gazebo/model_states", 5, model_state_cb);
	ros::Publisher target_pub = node.advertise<geometry_msgs::PointStamped>("/clicked_point", 5);	
	
	
	costmap.start();
	costmap_2d::Costmap2D* map = costmap.getCostmap(); 
	

	// 
	// 
	double res = map->getResolution();

	for(double i = -0.45; i < 0.6; i=+res)
	{
		unsigned int mx, my;
		map->worldToMap (i, 0.2, mx, my);
		map->setCost (mx, my, 255);
	}
	// worldToMap (double wx, double wy, unsigned int &mx, unsigned int &my) const 
	// mapToWorld (unsigned int mx, unsigned int my, double &wx, double &wy)
	// setCost (unsigned int mx, unsigned int my, unsigned char cost)



	astar_plugin::AStarPlanner planner = astar_plugin::AStarPlanner("AStar", &costmap);
	

	ros::Rate rate(1. / 1.);
	while (ros::ok()) {
		ros::spinOnce();
		std::vector<geometry_msgs::PoseStamped> path;
		bool valid = planner.makePlan(curr_state, GOAL, path);
		
		if(!valid || path.empty() || path.size()==1) continue; // goal reached
		
		std::cout<<curr_state.pose.position.x<< " " <<curr_state.pose.position.y<<std::endl;
		
		std::cout<< "move to the goal" << std::endl;
		int index = 1;

		geometry_msgs::PointStamped PointMsg;
		PointMsg.point = path[index].pose.position;

		std::cout<<path[index].pose.position.x<<" "<<path[index].pose.position.y<<std::endl;
		
		target_pub.publish(PointMsg);
		rate.sleep();

	}

	return 0;

}