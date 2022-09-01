#ifndef _APF_RRT_H
#define _APF_RRT_H

#include<CubeObstacle.h>
#include<ur10_FK.h>

#include<vector>


class APF_RRT
{
    private:
        std::vector<double> start;
        std::vector<double> goal;
        std::vector<Cube_obstacle> cubes;
        Point_3D goal_position;
        double Katt = 1.0, Krep = 10.0;
        double Drep = 0.3;
        double step_length = 2 * M_PI / 180;
        
        double Eatt, Erep;

    public:
        APF_RRT(std::vector<double> start, std::vector<double> goal, std::vector<Cube_obstacle> cubes):start(start), goal(goal), cubes(cubes){};
        ~APF_RRT(){};
        void setKatt(double Catt){this->Katt = Catt;};
        void setKrep(double Crep){this->Krep = Crep;};
        void setDrep(double distance){this->Drep = distance;};
        void setStep(double angel){this->step_length = angel * M_PI /180;};
        double getKatt(){return this->Katt;};
        double getKrep(){return this->Krep;};
        double getDrep(){return this->Drep;};
        double getStep(){return (this->step_length * 180 / M_PI);};

        std::vector<Eigen::Matrix4d> GoalInCarte();
        void setGoalPosition();

        double calEatt(){};
        double calErep(){};
};

std::vector<Eigen::Matrix4d> APF_RRT::GoalInCarte()
{
    std::vector<Eigen::Matrix4d> matrix;
    FK(this->goal, matrix);
    return matrix;
}

void APF_RRT::setGoalPosition()
{
    std::vector<Eigen::Matrix4d> T = this->GoalInCarte();
    this->goal_position.setX(T[5][0, 3]);
    this->goal_position.setY(T[5][1, 3]);
    this->goal_position.setZ(T[5][2, 3]);
}

double APF_RRT::calEatt()
{
    double energy_att = 0;
    for (size_t i = 0; i < this->start.size(); i++)
        energy_att += pow(this->start[i] - this->goal[i], 2);
    energy_att = Katt * sqrt(energy_att);
    return energy_att;
}




#endif