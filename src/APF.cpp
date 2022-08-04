#include <ctime>
#include <random>

#include <APF.h>
#include <box_generate.h>

#include <ros/duration.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

const double limit_joint_3 =  M_PI;
double resolution = 0.1;
void PahAlgorithm(moveit::planning_interface::MoveGroupInterface::Plan &planning, std::vector<double> start, std::vector<double> goal, std::vector<Point_3D> obstacles, 
										double step_time, double step_length, planning_scene::PlanningScene& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef );




int main(int argc, char** argv)
{
	ros::init(argc, argv, "APF_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(2);
	spinner.start();
	Boxes_Publisher(argc, argv);
	//Wait to create the move planning scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	sleep(0.1);
	
	robot_model_loader::RobotModelLoader Robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = Robot_model_loader.getModel();
	robot_model::RobotModelConstPtr kinematic_model_const = Robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);
	planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

		
	//Get All obstacles and set the scope of influence
	std::vector<Point_3D> obstacles;
		
	boost::shared_ptr<sensor_msgs::PointCloud2 const> pc2;
	pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pcl_output", ros::Duration(5));
	if (pc2 != NULL)
	{
		sensor_msgs::PointCloud out_pointcloud;
		sensor_msgs::convertPointCloud2ToPointCloud(*pc2, out_pointcloud);
		std::cout << out_pointcloud.points[0].x << "   " << out_pointcloud.points[0].y << "    " << out_pointcloud.points[0].z << std::endl;
		pc2octomap(out_pointcloud, resolution, obstacles);
		std::cout << "\033[41mFINISH OBSTACLE LOAD!" << "\033[0m" << std::endl;
	}
	else
		std::cout << "No existing pointcloud or octomap" << std::endl;

	//Get actual posicion
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
	std::vector<double> start_joint_values = group.getCurrentJointValues();
	for (size_t nums = 0; nums < start_joint_values.size(); nums++)
		std::cout << start_joint_values[nums] << " ";
	std:cout << std::endl;

	std::vector<double> goal_joint_values(6);
	
	//std::vector<std::vector<double>> path;
	moveit::planning_interface::MoveGroupInterface::Plan planning;
	std::vector<std::string> joint_names = group.getJointNames();
	monitor_ptr_udef->requestPlanningSceneState("get_planning_scene");
	//planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
	//robot_state::RobotState path_state = planning_Scene.getCurrentState();
	
		
	//for (size_t ng = 0; ng < goal_joint_values.size(); ng++)
		//goal_joint_values[ng] = start_joint_values[ng] - 20 * M_PI / 180;
	
	goal_joint_values = {190 * M_PI / 180, -90 * M_PI / 180, 122 * M_PI / 180, -35 * M_PI / 180, 99 * M_PI / 180, 193 * M_PI / 180};
	group.setJointValueTarget(goal_joint_values);
	std::cout << "\033[41mGET A NICE TARGET!" << std::endl;
	for (size_t i = 0; i < goal_joint_values.size(); i++)
		std::cout << goal_joint_values[i] << " ";
	std::cout << "\033[0m" << std::endl;
	
	//APF_path_planning_RT(obstacles, resolution * sqrt(2), start_joint_values, goal_joint_values, path, monitor_ptr_udef,  kinematic_model);
	
	std::vector<double> goal_quaternion(4, 0.0);
	Point_3D goal_coordinate = FK_xyz(goal_joint_values, goal_quaternion);
	double step_length = 0.1 * M_PI / 180;
	//path.push_back(start_joint_values);
	planning.trajectory_.joint_trajectory.joint_names = joint_names;
	trajectory_msgs::JointTrajectoryPoint tp;
	std::vector<double> next_joint_state(6);
	double U[2] = {0.0, 0.0};
	double step_time = 0.0;
	tp.positions = start_joint_values;
	ros::Duration first_step(0);
	tp.time_from_start = first_step;
	planning.trajectory_.joint_trajectory.points.push_back(tp);

	PahAlgorithm(planning, start_joint_values, goal_joint_values, obstacles, step_time, step_length, planning_scene, monitor_ptr_udef);
	
	for (size_t path_size = 0; path_size <next_joint_state.size(); path_size++)
			std::cout << planning.trajectory_.joint_trajectory.points.back().positions[path_size] <<  "  ";
		std::cout << std::endl;

	if (planning.trajectory_.joint_trajectory.points.size() == 1)
	{
		std::cout << "\033[41mFail!" << "\033[0m" << std::endl;
		return -1;
	}

	std::cout << "\033[41mGET THE PATH!" << "\033[0m" << std::endl;
	group.setJointValueTarget(planning.trajectory_.joint_trajectory.points.back().positions);
	group.execute(planning);
	group.move();
	

	sleep(0.1);
	group.setStartStateToCurrentState();
	
	std::vector<double> current_joint_values = group.getCurrentJointValues();
	std::cout << "\033[41mCurrent joint positions:\n";
	for (size_t i = 0; i < current_joint_values.size(); i++)
		std::cout << current_joint_values[i] << "  ";
	std:: cout << "\033[0m" << std::endl;

	return 0;
}



void PahAlgorithm(moveit::planning_interface::MoveGroupInterface::Plan &planning, std::vector<double> start, std::vector<double> goal, std::vector<Point_3D> obstacles,
										 double step_time, double step_length, planning_scene::PlanningScene& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef)
{
	std::vector<double> next_joint_state(6);
	double U[2] = {0.0, 0.0};
	std::vector<double> goal_quaternion(4, 0.0);
	Point_3D goal_coordinate = FK_xyz(goal, goal_quaternion);
	trajectory_msgs::JointTrajectoryPoint tp;
	while(1)
	{
		for (size_t j_n = 0; j_n <  planning.trajectory_.joint_trajectory.points.back().positions.size(); j_n++)
			next_joint_state[j_n] = planning.trajectory_.joint_trajectory.points.back().positions[j_n] + sign(goal[j_n] - planning.trajectory_.joint_trajectory.points.back().positions[j_n]) * step_length;
		
		std::vector<double> eff_quaternion(4, 0.0);
		Point_3D eff = FK_xyz(next_joint_state, eff_quaternion);
		
		APF_generate_RT(obstacles, resolution, eff, goal_coordinate, U);
		std::cout << goal_coordinate.getX() << "  " << goal_coordinate.getY() << "  " << goal_coordinate.getZ() << std::endl;

		planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
		ps->getCurrentStateNonConst().update();
		planning_scene::PlanningScenePtr scene = ps->diff();
		scene->decoupleParent();
		robot_state::RobotState state(scene->getRobotModel());
		state.setJointGroupPositions("manipulator", next_joint_state);
		scene->setCurrentState(state);
		robot_state::RobotState& current_state = scene->getCurrentStateNonConst();
		robot_state::RobotState copied_state = planning_scene.getCurrentState();
		copied_state.setJointGroupPositions("manipulator", next_joint_state);
		copied_state.update();
		bool state_valid = scene->isStateValid(current_state, "manipulator");
		

		if (U[1] == 0.0 && state_valid)
		{
			step_time += 0.005;
			ros::Duration step(step_time);
			tp.positions = next_joint_state;
			tp.time_from_start = step;
			planning.trajectory_.joint_trajectory.points.push_back(tp);
		}
		else
		{
			std::cout << "\033[41mMaybe Collision" << "\033[0m" << std::endl;
			
			double next_position[3] = {-(eff.getX() + sign(goal_coordinate.getX() - eff.getX())*0.001),  -(eff.getY() + sign(goal_coordinate.getY() - eff.getY())*0.001), eff.getZ() + sign(goal_coordinate.getZ() - eff.getZ())*0.001};

			double x, y, z, w;
			x = eff_quaternion[0];
			y = eff_quaternion[1];
			z = eff_quaternion[2];
			w = eff_quaternion[3];
			
			IkReal eerot[9]; IkReal eetrans[3]; 
			IkSolutionList<IkReal> solutions;
			eerot[0] = 1.0 - 2 * (y * y + z * z); 
			eerot[1] = 2 * (x * y - w * z); 
			eerot[2] = 2 * (w * y + x * z);
			eetrans[0] = next_position[0];
			eerot[3] = 2 * (x * y + w * z);
			eerot[4] = 1.0 - 2 * (x * x + z * z);
			eerot[5] = 2 * (z * y - w * x);
			eetrans[1] = next_position[1];
			eerot[6] = 2 * (x * z - w * y);
			eerot[7] = 2 * (z * y + x * w);
			eerot[8] = 1.0 - 2 * (x * x + y * y);
			eetrans[2] = next_position[2];
			
			bool bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);
			if( !bSuccess ) 
			{
				fprintf(stderr,"Failed to get ik solution\n");
				break; //No IK solution
			}
			
			//std::vector<IkReal> solvalues(GetNumJoints());
			if (solutions.GetNumSolutions() > 16)
			{
				std::cout << "\033[31mERROR!BAD SOLUTION NUMS, CHECK THE IKFAST SOURCE!" << solutions.GetNumSolutions() << std::endl;
				std::cout << eerot[0] << " "<< eerot[1] << " " << eerot[2] << " " << eetrans[0] << " " << eerot[3] << " " << eerot[4] << " " << eerot[5] << " " << eetrans[1] << " " << eerot[6] << " " << eerot[7] << " " << eerot[8] << " " << eetrans[2] << "\033[0m" << std::endl;
				break;	//IK wrong
			}
			
			double cost = 1e4;
			//find_avalible_solution(solutions, monitor_ptr_udef, kinematic_model, path.back(), next_joint_state);

			for(std::size_t k = 0; k < solutions.GetNumSolutions(); ++k) 
			{
				const IkSolutionBase<IkReal>& sol = solutions.GetSolution(k);
				std::vector<IkReal> vsolfree(sol.GetFree().size());
				std::vector<IkReal> solvalues(GetNumJoints());
				std::vector<double> joint_values(6);
				
				sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
				for( std::size_t j = 0; j < solvalues.size(); ++j)
					joint_values[j] = solvalues[j];		
				//check collision
				robot_state::RobotState state_(scene->getRobotModel());
				state_.setJointGroupPositions("manipulator", joint_values);
				scene->setCurrentState(state_);
				robot_state::RobotState& current_state_ = scene->getCurrentStateNonConst();
				robot_state::RobotState copied_state_ = planning_scene.getCurrentState();
				copied_state_.setJointGroupPositions("manipulator", joint_values);
				copied_state_.update();
				bool state_valid_ = scene->isStateValid(current_state_, "manipulator");

				double move_change_abs = 0.0;

				for (std::size_t j = 0; j < joint_values.size(); ++j)
				{
					if (j < joint_values.size()/2)
						move_change_abs += (10 * abs(joint_values[j] - planning.trajectory_.joint_trajectory.points.back().positions[j]));
					else
						move_change_abs += (1 * abs(joint_values[j] - planning.trajectory_.joint_trajectory.points.back().positions[j]));
				}

				if (state_valid_ && move_change_abs < cost)
				{
					for( std::size_t j = 0; j < joint_values.size(); ++j)
						next_joint_state[j] = joint_values[j];
					cost = move_change_abs;
				}
			}
			if (cost == 1e4)
			{
				std::cout << "Middle state collision!" << std::endl;
				break;
			}
			for (size_t i = 0; i < next_joint_state.size();i++)
				if (sign(next_joint_state[i]) != sign(planning.trajectory_.joint_trajectory.points.back().positions[i]))
					next_joint_state[i] += sign(planning.trajectory_.joint_trajectory.points.back().positions[i]) * 2 * M_PI;
			if (abs(next_joint_state[2]) >limit_joint_3 )
				next_joint_state[2] -= sign(next_joint_state[2]) * 2* M_PI;
			//此处引入递归？是否需要修改跳出条件？
			step_time += 10;
			ros::Duration step(step_time);
			tp.positions = next_joint_state;
			tp.time_from_start = step;
			planning.trajectory_.joint_trajectory.points.push_back(tp);
			break;
		}
		for (size_t path_size = 0; path_size <next_joint_state.size(); path_size++)
			std::cout << next_joint_state[path_size] <<  "  ";
		std::cout << std::endl;
		std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it = planning.trajectory_.joint_trajectory.points.end()-2;
		if ( (planning.trajectory_.joint_trajectory.points.back().positions[0] - goal[0] <= ERR) && 
				(planning.trajectory_.joint_trajectory.points.back().positions[1] - goal[1] <= ERR) &&
				(planning.trajectory_.joint_trajectory.points.back().positions[2] - goal[2] <= ERR) && 
				(planning.trajectory_.joint_trajectory.points.back().positions[3] - goal[3] <= ERR) && 
				(planning.trajectory_.joint_trajectory.points.back().positions[4] - goal[4] <= ERR) && 
				(planning.trajectory_.joint_trajectory.points.back().positions[5] - goal[5] <= ERR))
		{
			std::cout << "\033[41mFINISH APF+A*!" << "\033[0m" << std::endl;
			break;
		}
		else if (planning.trajectory_.joint_trajectory.points.back() == *it || 	planning.trajectory_.joint_trajectory.points.back() == *(it-1))
		{
			planning.trajectory_.joint_trajectory.points.pop_back();
			std::cout << "\033[41mFINISH APF+A*!" << "\033[0m" << std::endl;
			break;
		}
	}

}