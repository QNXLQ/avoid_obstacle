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

#endif