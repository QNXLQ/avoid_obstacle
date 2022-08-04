#ifndef _UR10_FK_H
#define _UR10_FK_H

#include <math.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

//ur10株洲测试机标定数据
double d[6 + 1] = { 0, 0.1273, 0,0,0.163941, 0.1157, 0.0922};//第0个不用
double a[6] = { 0,-0.612, -0.5723, 0, 0, 0};
double alpha[6] = { 1.570796, 0, 0, 1.570796, -1.570796, 0 };
//double theta[6 + 1];//八组解，每组解六个角，第0个都不用

void FK(std::vector<double>& joint_point, std::vector<Eigen::Matrix4d>& matrix)
{
	Eigen::Matrix4d T[6 + 1];//为了和theta对应，0不用
	for (int i = 1; i <= 6; i++)
	{
		T[i](0, 0) = cos(joint_point[i]);
		T[i](0, 1) = -sin(joint_point[i]) * cos(alpha[i - 1]);
		T[i](0, 2) = sin(joint_point[i]) * sin(alpha[i - 1]);
		T[i](0, 3) = a[i - 1] * cos(joint_point[i]);
		T[i](1, 0) = sin(joint_point[i]);
		T[i](1, 1) = cos(joint_point[i]) * cos(alpha[i - 1]);
		T[i](1, 2) = -cos(joint_point[i]) * sin(alpha[i - 1]);
		T[i](1, 3) = a[i - 1] * sin(joint_point[i]);
		T[i](2, 0) = 0;
		T[i](2, 1) = sin(alpha[i - 1]);
		T[i](2, 2) = cos(alpha[i - 1]);
		T[i](2, 3) = d[i];
		T[i](3, 0) = 0;
		T[i](3, 1) = 0;
		T[i](3, 2) = 0;
		T[i](3, 3) = 1;
    }
    matrix.push_back(T[1]);
    matrix.push_back(matrix.back() * T[2]);
    matrix.push_back(matrix.back() * T[3]);
    matrix.push_back(matrix.back() * T[4]);
    matrix.push_back(matrix.back() * T[5]);
    matrix.push_back(matrix.back() * T[6]);
    //Eigen::Matrix4d T0 = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
}


#endif