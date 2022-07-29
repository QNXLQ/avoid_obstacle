#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <ros/ros.h>

void pclCallback(const sensor_msgs::PointCloud2 &msg)
{
	sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
	std::cout << out_pointcloud.points.size() << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	boost::shared_ptr<sensor_msgs::PointCloud2 const> pc2;
	pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pcl_output", ros::Duration(5));
	if (pc2 != NULL)
		pclCallback(*pc2);
	else
		std::cout << "No existing pointcloud or octomap" << std::endl;
	//ros::Subscriber sub = n.subscribe("/pcl_output", 10000000, pclCallback);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	return 0;
}
