#ifndef _UR10_CHECK_COLLISOIN_H
#define _UR10_CHECK_COLLISION_H

#define _USE_MATH_DEFINES
#include <functional>
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>

#include <CubeObstacle.h>
#include <ur10_FK.h>
#include <APF.h>

double distance_point_to_point(Point_3D p1, Point_3D p2)
{
    return sqrt(pow(p1.getX() - p2.getX(), 2) + pow(p1.getY() - p2.getY(), 2) + (p1.getZ() - p2.getZ(), 2));
}

double distance_point_to_cube_obstacle(Point_3D p, Cube_obstacle cube)
{
     double x_min = cube.getX() - cube.getSize()[1] / 2;
    double x_max = cube.getX() + cube.getSize()[1] / 2;
    double y_min = cube.getY() - cube.getSize()[0] / 2;
    double y_max = cube.getY() - cube.getSize()[0] / 2;
    double z_min = cube.getZ() - cube.getSize()[2] / 2;
    double z_max = cube.getZ() - cube.getSize()[2] / 2;

    Point_3D point_z1(x_min, y_min, p.getZ()), point_z2(x_min, y_max, p.getZ()), point_z3(x_max, y_min, p.getZ()), point_z4(x_max, y_max, p.getZ()),
                        point_x1(p.getX(), y_min, z_min), point_x2(p.getX(), y_min, z_max), point_x3(p.getX(), y_max, z_min), point_x4(p.getX(), y_max, z_max), 
                        point_y1(x_min, p.getY(), z_min), point_y2(x_min, p.getY(), z_max), point_y3(x_max, p.getY(), z_min), point_y4(x_max, p.getY(), z_max);

    Point_3D p1(x_min, y_min, z_min), p2(x_min, y_min,z_max), p3(x_min, y_max, z_min), p4(x_min, y_max, z_max), 
                        p5(x_max, y_min, z_min), p6(x_max, y_min, z_max), p7(x_max, y_max, z_min), p8(x_max, y_max, z_max);

    if (p.getX() >= x_min && p.getX() <= x_max && p.getY() >= y_min && p.getY() <= y_max && p.getZ() >= z_min && p.getZ() <= z_max)
        return 0.0;
    
    if (p.getX() >= x_min && p.getX() <= x_max && p.getY() >= y_min && p.getY() <= y_max && (p.getZ() <= z_min || p.getZ() >= z_max))
        return min(abs(p.getZ() - z_max), abs(p.getZ() - z_min));
    if (p.getX() >= x_min && p.getX() <= x_max && (p.getY() <= y_min || p.getY() >= y_max) && p.getZ() >= z_min && p.getZ() <= z_max)
        return min(abs(p.getY() - y_max), abs(p.getY() - y_min));
    if ((p.getX() <= x_min || p.getX() >= x_max) && p.getY() >= y_min && p.getY() <= y_max && p.getZ() >= z_min && p.getZ() <= z_max)
        return min(abs(p.getX() - x_max), abs(p.getX() - x_min));

    if ((p.getX() <= x_min || p.getX() >= x_max) && (p.getY() <= y_min || p.getY() >= y_max) && p.getZ() >= z_min && p.getZ() <= z_max)
    {
            double d1 = distance_point_to_point(point_z1, p);
            double d2 = distance_point_to_point(point_z2, p);
            double d3 = distance_point_to_point(point_z3, p);
            double d4 = distance_point_to_point(point_z4, p);
            return min(min(d1, d2), min(d3, d4));
    }
    if ((p.getX() <= x_min || p.getX() >= x_max) && p.getY() <= y_min && p.getY() <= y_max && (p.getZ() <= z_min || p.getZ() >= z_max))
    {
            double d1 = distance_point_to_point(point_y1, p);
            double d2 = distance_point_to_point(point_y2, p);
            double d3 = distance_point_to_point(point_y3, p);
            double d4 = distance_point_to_point(point_y4, p);
            return min(min(d1, d2), min(d3, d4));
    }
    if (p.getX() >= x_min && p.getX() <= x_max && (p.getY() <= y_min || p.getY() >= y_max) && (p.getZ() <= z_min || p.getZ() >= z_max))
    {
            double d1 = distance_point_to_point(point_x1, p);
            double d2 = distance_point_to_point(point_x2, p);
            double d3 = distance_point_to_point(point_x3, p);
            double d4 = distance_point_to_point(point_x4, p);
            return min(min(d1, d2), min(d3, d4));
    }

    if ((p.getX() <= x_min || p.getX() >= x_max) && (p.getY() <= y_min || p.getY() >= y_max) && (p.getZ() <= z_min || p.getZ() >= z_max))
    {
            double d1 = distance_point_to_point(p1, p);
            double d2 = distance_point_to_point(p2, p);
            double d3 = distance_point_to_point(p3, p);
            double d4 = distance_point_to_point(p4, p);
            double d5 = distance_point_to_point(p5, p);
            double d6 = distance_point_to_point(p6, p);
            double d7 = distance_point_to_point(p7, p);
            double d8 = distance_point_to_point(p8, p);
            return min(min(min(d1, d2), min(d3, d4)), min(min(d5, d6), min(d7, d8)));
    }
}

double distance_line_to_cube_obstacle(Point_3D p1, Point_3D p2, Cube_obstacle cube)
{
    int s = 20;
    std::vector<Point_3D> point_list;
    for (int i = 0; i < (s+1); i++)
    {
        Point_3D point;
        point.setX ( p1.getX() + i * (p2.getX() - p1.getX()) / s);
        point.setY ( p1.getY() + i * (p2.getY() - p1.getY()) / s);
        point.setZ ( p1.getZ() + i * (p2.getZ() - p1.getZ()) / s);
        point_list.push_back(point);
    }
    std::vector<double> dis;
    for (auto p:point_list)
        dis.push_back(distance_point_to_cube_obstacle(p, cube));

    return std::min_element(*(dis.begin()),*(dis.end()));
}

bool ur10_collision_check(std::vector<double> joint_point, Cube_obstacle obstacle)
{
    std::vector<Eigen::Matrix4d> matrix;
    FK(joint_point, matrix);

    //check Link 1:
    Point_3D o1(0,0,0), o2(0,0,0.21);
    double distance = distance_line_to_cube_obstacle(o1, o2, obstacle);
    if (distance <=0.077425 )
        return true;
    
    //check Link 2:
    Eigen::Vector4d link2_top(0.675, 0, 0.158, 1);
    Eigen::Vector4d link2_bot(-0.073, 0, 0.158, 1);
    Eigen::Vector4d p1 = matrix[1] * link2_top;
    Eigen::Vector4d p2 = matrix[1] * link2_bot;
    o1.setX(p1[0]); o1.setY(p1[1]); o1.setZ(p1[2]);
    o2.setX(p2[0]); o2.setY(p2[1]); o2.setZ(p2[2]);
    distance = distance_line_to_cube_obstacle(o1, o2, obstacle);
    if (distance <= 0.09)
        return true;

    //check Link3:
    Eigen::Vector4d link3_top(0.618, 0, 0.199, 1);
    Eigen::Vector4d link3_bot(-0.06, 0, 0.199, 1);
    p1 = matrix[2] * link3_top;
    p2 = matrix[2] * link3_bot;
    o1.setX(p1[0]); o1.setY(p1[1]); o1.setZ(p1[2]);
    o2.setX(p2[0]); o2.setY(p2[1]); o2.setZ(p2[2]);
    distance = distance_line_to_cube_obstacle(o1, o2, obstacle);
    if (distance <= 0.064)
        return true;

    //check Link4+5+6:
    Point_3D o(matrix[5](0,3), matrix[5](1,3), matrix[5](2,3));
    distance = distance_point_to_cube_obstacle(o,obstacle);
    if (distance <= 0.26)
        return true;

    return false;
}






#endif