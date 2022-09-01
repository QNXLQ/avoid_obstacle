#ifndef _CUBEOBSTACLE_H
#define _CUBEOBSTACLE_H

#include <cstdlib>

class  Point_3D
{
	private:
		double x, y, z;

	public:
		Point_3D(){};
		Point_3D(double x, double y, double z) : x(x), y(y), z(z){};
		~Point_3D(){};
		bool operator == (const Point_3D &p1) const{
			return this->x == p1.x && this->y == p1.y && this->z == p1.z;
		}
		void setX(double px){ x = px;}
		void setY(double py){ y = py;}
		void setZ(double pz){ z = pz;}
		double getX(){return x;}
		double getY(){return y;}
		double getZ(){return z;}

};

class Cube_obstacle
{
    private:
        double x, y, z;
        double length, width, high;

    public:
        Cube_obstacle(){};
        Cube_obstacle(double x, double y, double z, double length, double width, double high) : x(x), y(y), z(z), length(length), width(width), high(high) {};
        Cube_obstacle(double x, double y, double z, double cube_size) : x(x), y(y), z(z), length(cube_size), width(cube_size), high(cube_size) {};
        ~Cube_obstacle(){};
        void setX(double center_x);
        void setY(double center_y);
        void setZ(double center_z);
        void setSize(double cube_length, double cube_width, double cube_high);
        void setSize(double cube_size);
        double getX();
        double getY();
        double getZ();
        double* getSize();

        Cube_obstacle operator+(const Cube_obstacle &obs)
        {
            Cube_obstacle new_obs;
            new_obs.setX((this->x + obs.x) / 2);
            new_obs.setY((this->y + obs.y) / 2);
            new_obs.setZ((this->z + obs.z) / 2);
            new_obs.setSize((this->length / 2 + abs(this->x - obs.x) + obs.length / 2), (this->width / 2 + abs(this->y - obs.y) + obs.width / 2), (this->high / 2 + abs(this->z - obs.z) + obs.high / 2));
            return new_obs;
        }
};

void Cube_obstacle::setX(double center_x)
{
    x = center_x;
}

void Cube_obstacle::setY(double center_y)
{
    y = center_y;
}

void Cube_obstacle::setZ(double center_z)
{
    z = center_z;
}

void Cube_obstacle::setSize(double cube_length, double cube_width, double cube_high)
{
    length = cube_length;
    width = cube_width;
    high = cube_high;
}

void Cube_obstacle::setSize(double cube_size)
{
    length = cube_size;
    width = cube_size;
    high = cube_size;
}

double Cube_obstacle::getX(){return x;}
double Cube_obstacle::getY(){return y;}
double Cube_obstacle::getZ(){return z;}
double* Cube_obstacle::getSize()
{
    double cube_size[3] = {length, width, high};
    return cube_size;
}


#endif