#include <ctime>
#include <random>

#include <IK.h>
#include <APF.h>
#include <msg_convert.h>
#include <ikfast_ur10.h>

#include <ros/duration.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

void VelAccCallback(const robot_state::RobotState joint_state, std::vector<std::string>  joint_names)
{
	for (size_t i = 0; i < joint_names.size(); i++)
			std::cout <<  	*(joint_state.getJointVelocities(joint_names[i]) )<< "  ";
		std::cout << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Vel_Acc_Listener");
	ros::NodeHandle node_handle;
	
	//Wait to create the move planning scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	sleep(0.1);

    robot_model_loader::RobotModelLoader Robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = Robot_model_loader.getModel();
	robot_model::RobotModelConstPtr kinematic_model_const = Robot_model_loader.getModel();
	planning_scene::PlanningScene planning_Scene(kinematic_model);

	static const std::string PLANNING_GROUP = "manipulator_i5";
	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

	std::vector<std::string> joint_names = group.getJointNames();	

	ros::Subscriber sub = node_handle.subscribe("", 1000)

}
	

