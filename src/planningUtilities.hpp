#ifndef _PLANNINGUTILITIES_HPP_
#define _PLANNINGUTILITIES_HPP_

#include <vector>
#include "motionUtilities.hpp"

#define INITIAL_HEIGHT 4.0 //not to hit object while going to start point
#define SLOPE_EPSILON 0.79 //Stable value 0.54 - 30 deg // 1.04 - 60 deg
#define HEIGHT 0.4  //from ground
#define PROBABILITY 0.07 // puts A* magic into RRT*

typedef std::vector<std::vector<int> > Matrix; // Matrix definition of program
typedef std::pair<int, int> intint; // pair for our data tyoe




namespace planningUtilities
    {
        //TODO : check for line obstacle collision.(To avoid add edge passes through an object, as rewiring)
        bool isLinear(const Coordinate &a, const Coordinate &b, const Coordinate &c) // check if three coordinate in a line
            {
                double slope1 = a.x == b.x ? -999.0 : (b.y - a.y) / (b.x - a.x);
                double slope2 = b.x == c.x ? -999.0 : (c.y - b.y) / (c.x - b.x);
                return fabs(slope2 - slope1) < SLOPE_EPSILON;
            }
        double dist(const Coordinate &c1, const Coordinate &c2) // return the distance between two coordinate
            {
                return sqrt(pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2) + pow(c1.z - c2.z, 2));
            }
        int dist(intint c1, intint c2) //return the distance between two pair for integer
            {
                //return abs(c1.second - c2.second) + abs(c1.first - c2.first);
                return (int)sqrt(pow((c1.second - c2.second),2) + pow((c1.first - c2.first) ,2));
            }
        std::vector<Coordinate> filterCoordinates(const std::vector<Coordinate> &route)
            {
                if(route.size() < 3)//if coordinate list is full directly return the route
                    return route;
                std::vector<Coordinate> ret; //return vector
                size_t i=0, j=1, k=2;
                for(k = 2; k<route.size(); k++)
                    {
                        if(isLinear(route[i], route[j], route[k]))
                            {
                                j = k;
                            }
                        else
                            {
                                ret.push_back(route[i]);
                                ret.push_back(route[j]);
                                i = j;
                                j = k;
                            }
                    }
                ret.push_back(route[route.size()-1]);
                return ret;
            }
    }

#endif