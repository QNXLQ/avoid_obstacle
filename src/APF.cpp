#include <ctime>
#include <random>

#include <IK.h>
#include <APF.h>
#include <msg_convert.h>
#include <ikfast_ur10.h>

#include <ros/duration.h>

//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/opencv.hpp>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

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
	robot_model::RobotModelPtr kinematic_model = Robot_model_loader.getModel();
	robot_model::RobotModelConstPtr kinematic_model_const = Robot_model_loader.getModel();
	planning_scene::PlanningScene planning_Scene(kinematic_model);
	planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

		
	//Get All obstacles and set the scope of influence
	std::vector<Point_3D> obstacles;
	double resolution = 0.032;
	string filename =  "pcds1652087193344162";
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::io::loadPCDFile (("/home/nielei/pcds/" + filename + ".pcd").c_str(), cloud);
	pc2octomap(cloud, filename, resolution, obstacles);
	std::cout << "\033[41mFINISH OBSTACLE LOAD!" << "\033[0m" << std::endl;
	//Get actual posicion
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
	std::vector<double> start_joint_values = group.getCurrentJointValues();
	for (size_t nums = 0; nums < start_joint_values.size(); nums++)
		std::cout << start_joint_values[nums] << " ";
	std:cout << std::endl;
	/*
	//Set goal posicion
	IkSolutionList<IkReal> solutions;
	IkReal eerot[9], eetrans[3];
	random_pose_generator(eerot, eetrans);
	//ik-fast algorithm to calculate inverse kinematics	
	bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);
	if( !bSuccess ) 
	{
		fprintf(stderr,"Failed to get ik solution\n");
		return -1; //No IK solution
	}
	
	//std::vector<IkReal> solvalues(GetNumJoints());
	if (solutions.GetNumSolutions() > 16)
	{
		std::cout << "\033[31mERROR!BAD SOLUTION NUMS, CHECK THE IKFAST SOURCE!" << solutions.GetNumSolutions() << std::endl;
		std::cout << eerot[0] << " "<< eerot[1] << " " << eerot[2] << " " << eetrans[0] << " " << eerot[3] << " " << eerot[4] << " " << eerot[5] << " " << eetrans[1] << " " << eerot[6] << " " << eerot[7] << " " << eerot[8] << " " << eetrans[2] << "\033[0m" << std::endl;
		return -2;	//IK wrong
	}
	*/

	
	//std::vector<double> joint_values(6);
	std::vector<double> goal_joint_values(7,40);

	//find_avalible_solution(solutions, monitor_ptr_udef, start_joint_values, goal_joint_values);
	
	std::vector<std::vector<double>> path;
	moveit::planning_interface::MoveGroupInterface::Plan planning;
	std::vector<std::string> joint_names = group.getJointNames();
	robot_trajectory::RobotTrajectory path_trajectory(kinematic_model_const, "manipulator");
	//robot_state::RobotState path_state = planning_Scene.getCurrentState();
	
	
	if (goal_joint_values.back() >= 0)
	{
		goal_joint_values.erase(goal_joint_values.end()-1);
		
		for (size_t ng = 0; ng < goal_joint_values.size(); ng++)
			goal_joint_values[ng] = start_joint_values[ng] - 20 * M_PI / 180;
		
		//goal_joint_values = {190 * M_PI / 180, -90 * M_PI / 180, 122 * M_PI / 180, -35 * M_PI / 180, 99 * M_PI / 180, 193 * M_PI / 180};
		group.setJointValueTarget(goal_joint_values);
		std::cout << "\033[41mGET A NICE TARGET!" << std::endl;
		for (size_t i = 0; i < goal_joint_values.size(); i++)
			std::cout << goal_joint_values[i] << " ";
		std::cout << "\033[0m" << std::endl;
		
		APF_path_planning_RT(obstacles, resolution, start_joint_values, goal_joint_values, path, monitor_ptr_udef);
		
		std::cout << "\033[41mGET THE PATH!" << "\033[0m" << std::endl;
		
		planning.trajectory_.joint_trajectory.joint_names = joint_names;
		planning.trajectory_.joint_trajectory.points.resize(path.size());
		trajectory_msgs::JointTrajectoryPoint tp;
		for (size_t i =0; i < path.size(); i++)
		{
			ros::Duration step((i+1)*0.05);
			tp.positions = path[i];
			tp.time_from_start = step;
			planning.trajectory_.joint_trajectory.points[i] = tp;
		}
		//path_convert_to_moveit_msgs(path_trajectory, path_state, path, joint_names, planning.trajectory_);
			
		group.execute(planning);
		group.move();
		

		sleep(0.1);
		group.setStartStateToCurrentState();
		
		std::vector<double> current_joint_values = group.getCurrentJointValues();
		std::cout << "\033[41mCurrent joint positions:\n";
		for (size_t i = 0; i < current_joint_values.size(); i++)
			std::cout << current_joint_values[i] << "  ";
		std:: cout << "\033[0m" << std::endl;
		//std::cout << group.getCurrentPose(group.getEndEffectorLink()).pose << std::endl;

	}
	else
		std::cout << "\033[41mALL STATE IN COLLISION!" << "\033[0m" << std::endl;
	
	
	return 0;
}
