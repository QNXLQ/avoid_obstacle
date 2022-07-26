#ifndef _MSG_CONVERT_H_
#define _MSG_CONVERT_H_

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>

#include<APF.h>

void path_convert_to_moveit_msgs(robot_trajectory::RobotTrajectory &path_trajectory, robot_state::RobotState &path_state, std::vector<std::vector<double>> path, std::vector<std::string> joint_names, moveit_msgs::RobotTrajectory& trajectory)
{
	/*
	Input:  path_trajectory	--- Intermediate variables of trajectory
		path_state	--- Intermediate variables of joint state
		path		--- Vector of path
		joint_names	--- All joint names
	Output: trajectory	--- trajectory with format moveit_msgs
	*/
		
	for (size_t path_index = 0; path_index < path.size(); path_index++)
	{
		path_trajectory.addSuffixWayPoint(path_state, 0.001);
		path_state.setJointGroupPositions("manipulator", path[path_index]);
		path_state.update();
	}
	
	path_trajectory.getRobotTrajectoryMsg(trajectory, joint_names);
}



#endif
