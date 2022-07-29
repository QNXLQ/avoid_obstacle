#ifndef _IK_H_
#define _IK_H_

#include <APF.h>

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

void find_avalible_solution(IkSolutionList<IkReal>& solutions, planning_scene_monitor::PlanningSceneMonitorPtr monitor_ptr_udef, std::vector<double>& start_joint_values, std::vector<double>& goal_joint_values)
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
		bool state_valid = check_collision(joint_values, monitor_ptr_udef);

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


#endif
