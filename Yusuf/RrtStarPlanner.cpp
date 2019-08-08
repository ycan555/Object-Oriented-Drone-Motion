#include <ros/ros.h>
#include "/home/e2236982/Desktop/KOVAN/drone_motion-master/src/src/motionUtilities.hpp"
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <hectorquad/coordinate.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include "/home/e2236982/Desktop/KOVAN/drone_motion-master/src/src/planningUtilities.hpp"
#include "RrtClasses.hpp"
#include <stdlib.h>     
#include <time.h>

class RrtStarPlanner{
    private:
        Node StartNode;
        Coordinate EndNode;
        std::vector<Node* > AllStates;
        double StepSize;
        double SearchSize;
    public:
        RrtStarPlanner(Coordinate Start, Coordinate End,double Stepsize, double Searchsize);
        ~RrtStarPlanner();
        bool operator++();
        Node* NearestNode(Coordinate c1);
        std::vector<Node*> nearNodes(Coordinate c1);
        Coordinate* Sample();
        Node* Steer(Node* nearest, Coordinate* Sample);
};

RrtStarPlanner::RrtStarPlanner(Coordinate Start, Coordinate End,double Stepsize, double Searchsize){
    Node Start1 (Start);
    StartNode=Start1;
    EndNode=End;
    StepSize=Stepsize;
    SearchSize=Searchsize;
    srand(time(NULL));
    AllStates.push_back(&StartNode);
}

RrtStarPlanner::~RrtStarPlanner(){
    StartNode.~Node();
    AllStates.clear();
}


Node* RrtStarPlanner::NearestNode(Coordinate c1){
    Node * retnode=&StartNode;
    Node * look;
    double shortest=c1-retnode->GetCoord;
    for(int i=0;i<this->AllStates.size();i++){
        look=AllStates[i];
        double dist = look->GetDist(c1);
        if(dist<shortest){
            retnode=look;
        }
    }
    return retnode;
}

std::vector<Node*> RrtStarPlanner::nearNodes(Coordinate c1){
    std::vector<Node*> retlist;
    for(int i=0;i<AllStates.size();i++){
        Node* NowState = AllStates[i];
        if(NowState->GetDist(c1)<SearchSize){
            retlist.push_back(NowState);
        }
        return retlist;
    }
}

Coordinate* RrtStarPlanner::Sample(){
    double randX = StartNode.GetCoord.x+(double)rand() / (double)RAND_MAX * ( EndNode.x - StartNode.GetCoord.x);
    double randY = StartNode.GetCoord.y+(double)rand() / (double)RAND_MAX * ( EndNode.y - StartNode.GetCoord.y);
    Coordinate* randompt=new Coordinate(randX, randY, HEIGHT);
    return randompt;
}

Node* RrtStarPlanner::Steer(Node* nearest, Coordinate* Sample){
    Coordinate c1 = nearest->GetCoord;
    double diffy=(c1.y - Sample->y);
    double diffx=(c1.x - Sample->x);
    double distance= sqrt(diffy*diffy + diffx*diffx);
    double prop=distance/StepSize;
    double newx=diffx*prop;
    double newy=diffy*prop;
    Coordinate newcoord(newx,newy,HEIGHT);
    Node* retnode=new Node(newcoord,nearest);
    return retnode;
}


bool RrtStarPlanner::operator++(){

}