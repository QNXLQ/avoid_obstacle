#include <ctime>
#include <random>

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

#define Change 20.0

int main(int argc, char** argv)
{

	ros::init(argc, argv, "APF_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	//Wait to create the move planning scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	sleep(0.1);
	
	robot_model_loader::RobotModelLoader Robot_model_loader("robot_description");
	robot_model::RobotModelConstPtr kinematic_model_const = Robot_model_loader.getModel();
	//robot_model::RobotModelPtr kinematic_model = Robot_model_loader.getModel();
	//planning_scene::PlanningScene planning_Scene(kinematic_model);

	//Get actual posicion
	static const std::string PLANNING_GROUP = "manipulator_i5";
	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
	std::vector<double> start_joint_values = group.getCurrentJointValues();
	for (size_t nums = 0; nums < start_joint_values.size(); nums++)
		std::cout << start_joint_values[nums] << " ";
	std:cout << std::endl;
	
	std::vector<std::vector<double>> path;
	moveit::planning_interface::MoveGroupInterface::Plan planning;
	std::vector<std::string> joint_names = group.getJointNames();	
	robot_trajectory::RobotTrajectory path_trajectory(kinematic_model_const, "manipulator_i5");

	std::vector<double> goal_joint_values(6);
	//goal_joint_values = start_joint_values;
	//goal_joint_values[4] = start_joint_values[4] - Change * M_PI / 180;
	for (size_t ng = 0; ng < goal_joint_values.size(); ng++)
		goal_joint_values[ng] = start_joint_values[ng] - Change * M_PI / 180;
	group.setJointValueTarget(goal_joint_values);
	
	std::cout << "\033[41mGET A NICE TARGET!" << std::endl;
	for (size_t i = 0; i < goal_joint_values.size(); i++)
		std::cout << goal_joint_values[i] << " ";
	std::cout << "\033[0m" << std::endl;
	
	path.push_back(start_joint_values);
	std::vector<double> next_joint_state(6);
	double step_length = 0.1 * M_PI / 180;
	
	while(1)
	{
		for (size_t j_n = 0; j_n < path.back().size(); j_n++)
			next_joint_state[j_n] = path.back()[j_n] + sign(goal_joint_values[j_n] - path.back()[j_n]) * step_length;
		path.push_back(next_joint_state);
		
		if ((path.back()[0] - goal_joint_values[0] <= ERR) && (path.back()[1] - goal_joint_values[1] <= ERR) && (path.back()[2] - goal_joint_values[2] <= ERR) && (path.back()[3] - goal_joint_values[3] <= ERR) && (path.back()[4] - goal_joint_values[4] <= ERR) && (path.back()[5] - goal_joint_values[5] <= ERR))
		break;
	}
	
	std::cout << "\033[41mGET THE PATH!" << "\033[0m" << std::endl;
	
	moveit_msgs::RobotTrajectory trajectory;
	
	planning.trajectory_.joint_trajectory.joint_names = joint_names;
	planning.trajectory_.joint_trajectory.points.resize(path.size());
	trajectory_msgs::JointTrajectoryPoint tp;
	std::vector<double> velocity(joint_names.size(), step_length );
	std::vector<double> acceleration(joint_names.size(), step_length*100 );
	for (size_t i =0; i < path.size(); i++)
	{
		ros::Duration step((i+1)*0.01);
		tp.positions = path[i];
		tp.time_from_start = step;
		if (i == path.size() - 1)		//Design   velocity and acceleration for  smooth trajectory
		{
			tp.velocities = {0, 0, 0, 0, 0, 0};
			tp.accelerations = {-step_length*100, -step_length*100, -step_length*100, -step_length*100, -step_length*100, -step_length*100};
		}
		else
		{
			tp.velocities = velocity;
			tp.accelerations = acceleration;
		}
		planning.trajectory_.joint_trajectory.points[i] = tp;
	}
		
	group.execute(planning);
	group.move();
	
	sleep(0.2);
	group.setStartStateToCurrentState();
	
	std::vector<double> current_joint_values = group.getCurrentJointValues();
	std::cout << "\033[41mCurrent joint positions:\n";
	for (size_t i = 0; i < current_joint_values.size(); i++)
		std::cout << current_joint_values[i] << "  ";
	std:: cout << "\033[0m" << std::endl;
	
	return 0;
}
