#ifndef _APF_H_
#define _APF_H_

#define _USE_MATH_DEFINES
#include <iostream>
#include <stdlib.h>
#include <functional>
#include <math.h>
#include <string>
#include <vector>
#include <unordered_map>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <ikfast_ur10.h>

const double d1 =  0.1273;
const double a2 = -0.612;
const double a3 = -0.5723;
const double d4 =  0.163941;
const double d5 =  0.1157;
const double d6 =  0.0922;

const double joint_limit_min = -2 * M_PI;
const double joint_limit_max = 2 * M_PI;

#define ERR 1e-6

struct Point_3D
{
	double x, y, z;

	bool operator == (const Point_3D &p1) const{
		return this->x == p1.x && this->y == p1.y && this->z == p1.z;
	}
};

template<>
struct std::hash<Point_3D>
{
	size_t operator() (const Point_3D &p3d) const noexcept
	{
		return std::hash<double>()(p3d.x) ^ std::hash<double>()(p3d.y) ^ std::hash<double>()(p3d.z);
	}
};

double sign(double number)
{
	if (number > 0)
		return 1.0;
	else if (number < 0)
		return -1.0;
	else
		return 0.0;
}

void FK_xyz(std::vector<double> joint_state, Point_3D coordinate, std::vector<double> &quaternion) 
{
	std::vector<double>::iterator q = joint_state.begin();
	double s1 = sin(*q), c1 = cos(*q); q++;
	double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); 
	q++;
	double s3 = sin(*q), c3 = cos(*q); 
	q23 += *q; q234 += *q; q++;
	double s4 = sin(*q), c4 = cos(*q); 
	q234 += *q; q++;
	double s5 = sin(*q), c5 = cos(*q); 
	q++;
	double s6 = sin(*q), c6 = cos(*q); 
	double s23 = sin(q23), c23 = cos(q23);
	double s234 = sin(q234), c234 = cos(q234);

	coordinate.x = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1;
	coordinate.y = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1;
	coordinate.z = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4);

	double trace = c234*c1*s5 - c5*s1 - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6 + s234*c5*s6 - c234*c6;
	quaternion[3] = sqrt(trace + 1.0) / 2.0;
	quaternion[0] = ((-c234*s6 - s234*c5*c6) - (s6*(c1*s5 - c234*c5*s1) - s234*c6*s1)) / (4 * quaternion[3]);
	quaternion[1] = ((-s6*(s1*s5 + c234*c1*c5) - s234*c1*c6) - (-s234*s5)) / (4 * quaternion[3]);
	quaternion[2] = ((c1*c5 + c234*s1*s5) - ( c6*(s1*s5 + c234*c1*c5) - s234*c1*s6)) / (4 * quaternion[3]);
}

void random_pose_generator(IkReal* eerot, IkReal* eetrans)
{
	std::cout << "\033[41mSTART TO GENERATE RANDOM POSE!" << "\033[0m" << std::endl;
	//Generate a random target pose
	std::vector<double> pose(6, 0.0);
	std::random_device rand;	//random time seed
	std::default_random_engine e(rand());
	std::uniform_real_distribution<double> p(-0.75, 0.75);	//position x, y, z
	std::uniform_real_distribution<double> r(-1.8, 1.8);	//orientation vector of rotation rx, ry, rz
	for (int i = 0; i < 6;i++)
	{
		if (i<3)
			pose[i] = p(e);
		else
			pose[i] = r(e);
	}
	
	std::cout << "\033[41mRANDOM TARGET:" << std::endl;
	for (size_t i = 0; i < pose.size(); i++)
		std::cout << pose[i] << " ";
	std::cout << "\033[0m" << std::endl;
	
	cv::Mat rotv = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
	cv::Mat rotm;
	cv::Rodrigues(rotv,rotm);
	
	*eerot = rotm.at<double>(0,0); eerot++;
	*eerot = rotm.at<double>(0,1); eerot++;
	*eerot = rotm.at<double>(0,2); eerot++;
	*eetrans = pose[0]; eetrans++;
	*eerot = rotm.at<double>(1,0); eerot++;
	*eerot = rotm.at<double>(1,1); eerot++;
	*eerot = rotm.at<double>(1,2); eerot++;
	*eetrans = pose[1]; eetrans++;
	*eerot = rotm.at<double>(2,0); eerot++;
	*eerot = rotm.at<double>(2,1); eerot++;
	*eerot = rotm.at<double>(2,2); 
	*eetrans = pose[2];
}

bool check_collision(std::vector<double> &joint_values, planning_scene_monitor::PlanningSceneMonitorPtr &monitor_ptr_udef, robot_model::RobotModelPtr &kinematic_model)
{
	//check collsion
	planning_scene_monitor::LockedPlanningSceneRW ps(monitor_ptr_udef);
	ps->getCurrentStateNonConst().update();
	planning_scene::PlanningScenePtr scene = ps->diff();
	scene->decoupleParent();

	robot_state::RobotState state(scene->getRobotModel());
	state.setJointGroupPositions("manipulator", joint_values);
	scene->setCurrentState(state);

	planning_scene::PlanningScene planning_scene(kinematic_model);
	robot_state::RobotState& current_state = scene->getCurrentStateNonConst();
	robot_state::RobotState copied_state = planning_scene.getCurrentState();
	copied_state.setJointGroupPositions("manipulator", joint_values);
	copied_state.update();

	return scene->isStateValid(copied_state, "manipulator");
}

void find_avalible_solution(IkSolutionList<IkReal>& solutions, planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef, robot_model::RobotModelPtr kinematic_model, std::vector<double>& start_joint_values, std::vector<double>& goal_joint_values)
{
	/*
	Input:  solutions		---Solutions of ik
		monitor_ptr_udef	---for check_collision
		start_joint_values	---Start pose
	Output: goal_joint_values	---Avalible state
	*/
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
		bool state_valid = check_collision(joint_values, monitor_ptr_udef, kinematic_model);

		double move_change_abs = 0.0;

		for (std::size_t j = 0; j < joint_values.size(); ++j)
		{
			if (j < joint_values.size()/2)
				move_change_abs += (10 * abs(joint_values[j] - start_joint_values[j]));
			else
				move_change_abs += (1 * abs(joint_values[j] - start_joint_values[j]));
		}
		
		// order those joints target with a rule: no collision(!state_valid=0) and minium joints changes(move_change_abs is min)
		pair<bool, double> solvalues_i(!state_valid, move_change_abs);	
		std::priority_queue<std::pair<bool, double>, std::vector<pair<bool, double>>, std::greater<pair<bool, double>>> move_change_list;
		
		move_change_list.push(solvalues_i);

		if (move_change_list.top().first == 0 && (goal_joint_values.back() < 0 || goal_joint_values.back() > move_change_list.top().second))
		{
			goal_joint_values.back() = move_change_list.top().second;
			for( std::size_t j = 0; j < joint_values.size(); ++j)
				goal_joint_values[j] = joint_values[j];
		}	
	}
}

//Reduce points of the PointCloud by Octomap
void pc2octomap(sensor_msgs::PointCloud &cloud, double resolution, std::vector<Point_3D> &obstacles )
{
	/*
	Input:  cloud		---Point Cloud which need be solved
		output_filename	---the file for save those points after reduce points 
		resolution	---Resolution of the octomap
	Output: obstacles 	---All points after reduce
	*/
	//FILE *fp = fopen(("/home/nielei/maps/"+ output_filename + ".txt").c_str(),"w");
	octomap::OcTree tree(resolution);
	Point_3D m_p;
	//PointCloud convert to Octomap
	for (auto p:cloud.points)
		tree.updateNode( octomap::point3d(p.x, p.y, p.z), true);
	
	//Save all cube center of octomap if is necessary
	for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs();it!=end;++it)
	{
		m_p.x = it.getX() * 1000;
		m_p.y = it.getY() * 1000;
		m_p.z = it.getZ() * 1000;
		if (tree.isNodeOccupied(*it))
		{
			//fprintf(fp, "%f %f %f\n", m_p.x, m_p.y, m_p.z);	
			obstacles.push_back(m_p);
		}
	}
	//fclose(fp);
	
}


double APF_generate_RT(std::vector<Point_3D> &obstacles, double resolution, Point_3D actual, Point_3D goal)
{
	/*
	Input:  obstacles	--- Obstacles posicions, unit mm
		resolution	--- Scope of influence of every obstacle, should be equal to the resolution of octomap
		actual		--- Actual posicion, unit mm
		goal		--- Goal posicion, unit mm
	*/
	
	double C_ata = 10.0; //Attractiveness coefficient
	double C_rep = 20.0; //Repulsion coefficient
	
	double rep = 0;
	double rat = C_ata * (pow((actual.x - goal.x),2) + pow((actual.y - goal.y),2) + pow((actual.z - goal.z),2));
	
	for(size_t i = 0; i < obstacles.size(); i++)
	{
		double REP = (pow((obstacles[i].x - actual.x),2) + pow((obstacles[i].y - actual.y),2) + pow((obstacles[i].z - actual.z),2));
		if (REP >= resolution)
			continue;
		else
			rep += C_rep * (1/REP - 1/resolution) * 1/(pow(REP,2));
	}
	return rat+rep;
}


void APF_generate_RT(std::vector<Point_3D> &obstacles, double resolution, Point_3D actual, Point_3D goal, double* U)
{
	/*
	Input:  obstacles	--- Obstacles posicions, unit mm
		resolution	--- Scope of influence of every obstacle, should be equal to the resolution of octomap
		actual		--- Actual posicion, unit mm
		goal		--- Goal posicion, unit mm
	Output: U 		--- Energy of APF
	*/
	
	double C_ata = 10.0; //Attractiveness coefficient
	double C_rep = 20.0; //Repulsion coefficient
	
	double rep = 0;
	double rat = C_ata * sqrt(pow((actual.x - goal.x),2) + pow((actual.y - goal.y),2) + pow((actual.z - goal.z),2));
	
	for(size_t i = 0; i < obstacles.size(); i++)
	{
		double REP = sqrt(pow((obstacles[i].x - actual.x),2) + pow((obstacles[i].y - actual.y),2) + pow((obstacles[i].z - actual.z),2));
		if (REP > resolution)
			continue;
		else
			rep += C_rep * (1/REP - 1/resolution) * 1/(pow(REP,2));
	}
	*U = rat; U++;
	*U = rep;
}

void APF_generate_OL(Point_3D start, std::vector<Point_3D> &obstacles, double resolution, Point_3D goal, std::unordered_map<Point_3D,double> &APF_map)
{
	/*
	Input:  start		--- start posicion, unit mm
		obstacles	--- Obstacles posicions
		resolution	--- Scope of influence of every obstacle, should be equal to the resolution of octomap
		goal		--- Goal posicion, unit mm
	Output: APF_map		--- Artificial Potencial Field map
	*/
	double U[2] = {};
	for (int x = 0; x < abs(goal.x-start.x)*10; x++)
		for (int y = 0; y < abs(goal.y-start.y)*10; y++)
			for (int z = 0; z < abs(goal.z-start.z)*10; z++)
			{
				Point_3D point;
				point.x = x;
				point.y = y;
				point.z = z;
				APF_generate_RT(obstacles, resolution, point, goal, U);
				APF_map[point] = U[0] + U[1];
			}
}

void APF_path_planning_RT(std::vector<Point_3D> &obstacles, double resolution, std::vector<double> &joint_start, std::vector<double> joint_goal, std::vector<std::vector<double>> &path, planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef, robot_model::RobotModelPtr kinematic_model)
{
	/*
	Input:  obstacles	---Obstacles posicions in the map with unit mm
		resolution	---Scope of influence of every obstacle, mm
		joint_start	---Start joint state
		joint_goal	---Goal joint state
	Output:	path		---Path generated
	*/
	
	Point_3D goal_coordinate;
	std::vector<double> goal_quaternion(4, 0.0);
	FK_xyz(joint_goal, goal_coordinate, goal_quaternion);
	double step_length = 0.1 * M_PI / 180;
	path.push_back(joint_start);
	
	std::vector<double> next_joint_state(6);
	double U[2] = {0.0, 0.0};

	while(1)
	{
		for (size_t j_n = 0; j_n < joint_start.size(); j_n++)
			next_joint_state[j_n] = path.back()[j_n] + sign(joint_goal[j_n] - path.back()[j_n]) * step_length;
		
		Point_3D eff;
		std::vector<double> eff_quaternion(4, 0.0);
		FK_xyz(next_joint_state, eff, eff_quaternion);
		APF_generate_RT(obstacles, resolution*10, eff, goal_coordinate, U);
		if (U[1] == 0.0 && check_collision(next_joint_state, monitor_ptr_udef, kinematic_model))
			path.push_back(next_joint_state);
		else
		{
			std::cout << "\033[41mMaybe Collision" << "\033[0m" << std::endl;
			std::cout << path.size() << std::endl;
			double next_position[3] = {eff.x + sign(goal_coordinate.x - eff.x)*0.001,  eff.y + sign(goal_coordinate.y - eff.y)*0.001, eff.z + sign(goal_coordinate.z - eff.z)*0.001};
			
			double x, y, z, w;
			x = eff_quaternion[0];
			y = eff_quaternion[1];
			z = eff_quaternion[2];
			w = eff_quaternion[3];
			
			IkReal* eerot; IkReal*eetrans; 
			IkSolutionList<IkReal> solutions;
			*eerot = 1.0 - 2 * (y * y + z * z); eerot++;
			*eerot = 2 * (x * y - w * z); eerot++;
			*eerot = 2 * (w * y + x * z); eerot++;
			*eetrans = next_position[0]; eetrans++;
			*eerot = 2 * (x * y + w * z); eerot++;
			*eerot = 1.0 - 2 * (x * x + z * z); eerot++;
			*eerot = 2 * (z * y - w * x); eerot++;
			*eetrans = next_position[1]; eetrans++;
			*eerot = 2 * (x * z - w * y); eerot++;
			*eerot = 2 * (z * y + x * w); eerot++;
			*eerot = 1.0 - 2 * (x * x + y * y);
			*eetrans = next_position[2];
			
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
			
			
			find_avalible_solution(solutions, monitor_ptr_udef, kinematic_model, path.back(), next_joint_state);

			path.push_back(next_joint_state);
		}
		for (size_t path_size = 0; path_size < path.back().size(); path_size++)
			std::cout << path.back()[path_size] <<  "  ";
		std::cout << std::endl;
		std::vector<std::vector<double>>::iterator it = path.end()-2;
		if (((path.back()[0] - joint_goal[0] <= ERR) && (path.back()[1] - joint_goal[1] <= ERR) && (path.back()[2] - joint_goal[2] <= ERR) && (path.back()[3] - joint_goal[3] <= ERR) && (path.back()[4] - joint_goal[4] <= ERR) && (path.back()[5] - joint_goal[5] <= ERR)) || path.back() == *it || path.back() == *(it-1))
		{
			std::cout << "\033[41mFINISH APF+A*!" << "\033[0m" << std::endl;
			break;
		}
	}
}

void APF_path_planning_OL(std::unordered_map<Point_3D,double> APF_map, std::vector<double> joint_start, std::vector<double> joint_goal, std::vector<std::vector<double>> &path, planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef, robot_model::RobotModelPtr kinematic_model)
{
	/*
	Input:  APF_map		---Artificial Potencial Field map
		joint_start	---Start joint state
		joint_goal	---Goal joint state
	Output:	path		---Path generated
	*/
	
	Point_3D goal_coordinate;
	std::vector<double> goal_quaternion(4, 0.0);
	FK_xyz(joint_goal, goal_coordinate, goal_quaternion);
	double step_length = 2*M_PI/180;
	path.push_back(joint_start);
	std::vector<double> Energy;
	std::vector<std::vector<double>> joint_state_list;
	
	while(1)
	{
		for (double base = joint_start[0] - step_length; base <= joint_start[0] + step_length; base += step_length)
			for (double upper_arm = joint_start[1] - step_length; upper_arm <= joint_start[1] + step_length; upper_arm += step_length)
				for (double forearm = joint_start[2] - step_length; forearm <= joint_start[2] + step_length; forearm += step_length)
					for (double wrist1 = joint_start[3] - step_length; wrist1 <= joint_start[3] + step_length; wrist1 += step_length)
						for (double wrist2 = joint_start[4] - step_length; wrist2 <= joint_start[4] + step_length; wrist2 += step_length)
							for (double wrist3 = joint_start[5] - step_length; wrist3 <= joint_start[5] + step_length; wrist3 += step_length)
							{
								std::vector<double> joint_state = {base, upper_arm, forearm, wrist1, wrist2, wrist3};
								//check collision
								if (check_collision(joint_state, monitor_ptr_udef, kinematic_model))
									continue;
								else
								{
									Point_3D eff;
									std::vector<double> eff_quaternion(4, 0.0);
									FK_xyz(joint_state, eff, eff_quaternion);
									Energy.push_back(APF_map[eff]);
									joint_state_list.push_back(joint_state);
								}
							}
		int index = min_element(Energy.begin(),Energy.end()) - Energy.begin();
		path.push_back(joint_state_list[index]);
		joint_start = joint_state_list[index];
		
		Energy.clear();
		joint_state_list.clear();
		std::vector<std::vector<double>>::iterator it = path.end()-2;
		if ((path.back() == joint_goal) || (path.back() == *it))
			break;
	}
}

#endif
