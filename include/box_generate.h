#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <ros/ros.h>

#include <APF.h>

#include <fcl/octree.h>
#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>

octomap::OcTree* generateOcTree(double resolution, sensor_msgs::PointCloud cloud);

void generateBoxesFromOctomap(std::vector<moveit_msgs::CollisionObject>& boxes, fcl::OcTree& tree);

void Boxes_Publisher(int argc, char** argv)
{
	double resolution_boxes = 0.05;
	//ros::init (argc, argv, "boxes_generate");
   // ros::NodeHandle nh;
	moveit::planning_interface::PlanningSceneInterface current_scene;
	sleep(0.5);
    static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
	
	boost::shared_ptr<sensor_msgs::PointCloud2 const> pc2;
	pc2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/pcl_output", ros::Duration(5));
	sensor_msgs::PointCloud out_pointcloud;
	if (pc2 != NULL)
	{
		sensor_msgs::convertPointCloud2ToPointCloud(*pc2, out_pointcloud);
		std::cout << "\033[41mFINISH OBSTACLE LOAD!" << "\033[0m" << std::endl;
	}
	else
		std::cout << "No existing pointcloud or octomap" << std::endl;

	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(generateOcTree(resolution_boxes, out_pointcloud)));

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	generateBoxesFromOctomap(collision_objects, *tree);
	current_scene.addCollisionObjects(collision_objects);
}

octomap::OcTree* generateOcTree(double resolution, sensor_msgs::PointCloud cloud)
{
  octomap::OcTree* tree = new octomap::OcTree(resolution);

  // insert some measurements of occupied cells
	Point_3D m_p;
	//PointCloud convert to Octomap
	for (auto p:cloud.points)
		tree->updateNode( octomap::point3d(p.x, p.y, p.z), true);

  return tree;  
}

void generateBoxesFromOctomap(std::vector<moveit_msgs::CollisionObject>& boxes, fcl::OcTree& tree)
{
 	std::vector<std::array<fcl::FCL_REAL, 6> > boxes_ = tree.toBoxes();
	boxes.resize(boxes_.size());
	for(std::size_t i = 0; i < boxes_.size(); ++i)
	{
		boxes[i].header.frame_id = "base_link";
		boxes[i].header.stamp = ros::Time::now();
		boxes[i].id = "OBS" + std::to_string(i);

		shape_msgs::SolidPrimitive box_info;
		box_info.type = box_info.BOX;
		box_info.dimensions.resize(3);
		box_info.dimensions[box_info.BOX_X] = boxes_[i][3];
		box_info.dimensions[box_info.BOX_Y] = boxes_[i][3];
		box_info.dimensions[box_info.BOX_Z] = boxes_[i][3];

		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = boxes_[i][0];
		box_pose.position.y = boxes_[i][1];
		box_pose.position.z = boxes_[i][2];

		boxes[i].primitives.push_back(box_info);
		boxes[i].primitive_poses.push_back(box_pose);
		boxes[i].operation = boxes[i].ADD;
	}

	std::cout << "boxes size: " << boxes.size() << std::endl;
}
