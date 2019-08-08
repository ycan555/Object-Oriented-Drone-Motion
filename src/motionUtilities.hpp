#ifndef _MOTIONUTILITIES_HPP_
#define _MOTIONUTILITIES_HPP_

#include <ros/ros.h>
#include <iostream>

#define FLOAT_EPSILON 0.001
#define CLEARENCE 0.7 //(Coursera: Motion planning lecture)

bool equalFloat(double a, double b)//float representation epsilon comparisson
    {
        return fabs(a-b) <= FLOAT_EPSILON;
    }

bool isObstacle(const std::string &name)//check if given is obstacle
    {
        if(name == "quadrotor" || name == "ground_plane")
            return false;
        return true;
    }

class Coordinate //coordinate data type
    {
        public:    
            double x,y,z; //x,y,z coordinates
            Coordinate(double x, double y, double z) : x(x), y(y), z(z) {} //constructor
            Coordinate() //default constructor
                {
                    x = y = z = 0;
                }
            bool operator==(const Coordinate &rhs) //opeartor to checking if coordinates are equal
                {
                    return equalFloat(x, rhs.x) && equalFloat(y, rhs.y) && equalFloat(z, rhs.z);
                }
            friend std::ostream& operator<<(std::ostream &os, const Coordinate &c)//ostream opeartor
                {
                    os<<"x = "<<c.x<<" y = "<<c.y<<" z = "<<c.z;
                    return os;
                }
            double operator-(Coordinate c2)
                {
                    return sqrt(pow(this->x-c2.x,2)+pow(this->y-c2.y,2)+pow(this->z-c2.z,2));
                }
    };

class Obstacle //obstacle data type
    {
        public:
            virtual bool inObstacle(const Coordinate &c) = 0; // check if in obstacle
            bool operator==(const Obstacle &rhs) // operator to checking if obstacles are the same
                {
                    return name == rhs.name && coord == rhs.coord;
                }
            friend std::ostream &operator<<(std::ostream &os, const Obstacle &obs) //ostream opeartor
                {
                    os<<obs.name<<" "<<obs.coord;
                    return os;
                }
            // public elements
            Coordinate coord;
            std::string name; 
    };

class Cylinder : public Obstacle
    {
        public:
            double r, h; // radius and the height of Cylinder
            Cylinder(const std::string &name, const Coordinate &c, double r, double h) : r(r), h(h)  // constructor
                {
                    this->name = name;
                    coord = c;
                }
            bool inObstacle(const Coordinate &c) // check if in obstacle
                {
                    if (c.z > h)
                    {
                        return false;
                    }
                    return sqrt(pow(c.x - coord.x, 2) + pow(c.y - coord.y, 2)) < r + CLEARENCE;
                }
    };

bool does_sep(Coordinate &p1, Coordinate &p2, double x, double y, double r){ // does it seperate the obstacle
    double m = (p2.y - p1.y)/(p2.x - p1.x); // slope
    double a = ((1+pow(m,2)));
    double b = ((-2*x) + (2*m*p1.y) - (pow(m,2)*p1.x) - m*y);
    double d = (pow(x,2) + pow(p1.y,2) + pow(m,2)*pow(p1.x,2) + pow(y,2) -2*m*p1.x*p1.y -2*p1.y*y +2*p1.y*y + 2*m*p1.x*y - r * r);
    double delta = pow(b , 2) - 4*a*d ;
    double final1,final2;
    if(delta<0) return false;
    else{
        final1=(-b + sqrt(delta))/2*a;
        if(p1.x<p2.x){
            if(p1.x<final1 && final1<p2.x){
                return true;
            }
            else return false;
        }
        else{
            if(p2.x<final1 && final1<1){
                return true;
            }
            else return false;
        }
    }
}
#endif