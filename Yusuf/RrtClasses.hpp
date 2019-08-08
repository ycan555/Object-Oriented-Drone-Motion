#ifndef _RRTCLASSES_HPP_
#define _RRTCLASSES_HPP_

#include "/home/e2236982/Desktop/KOVAN/drone_motion-master/src/src/planningUtilities.hpp"
#include <ros/ros.h>
#include "/home/e2236982/Desktop/KOVAN/drone_motion-master/src/src/motionUtilities.hpp"


class Node{
    private:
        bool is_reachable;
        Node *parrent;
        std::vector<Node*> childs;
        Coordinate coord;
        double cost;
    public:
        Node(Coordinate c1, Node *parrent=NULL);
        Node();
        ~Node();
        std::vector<Node*> getChilds();
        Node* getParrent();
        void MakeReachable(bool state);
        bool operator==(const Node &n2);
        void LinkNewNode(Node& newnode);
        double GetCost() const;
        double GetDist(const Node &n2) const;
        double GetDist(const Coordinate &n2) const;
        void ChangeParrent(Node* parrent);
        std::vector<Coordinate> GetRoute();
        bool IsInGoal(Coordinate goal, double epsilon);
        Coordinate GetCoord();
};

Node::Node(){
    coord=Coordinate(.0 , .0 , .0);
    parrent=NULL;
}

Node::Node(Coordinate c1, Node *parrent){
    this->coord=c1;
    if(parrent!=NULL)
    this->cost=parrent->cost + this->GetDist(*parrent);
    else cost=0;
    this->parrent=parrent;
    if(parrent!=NULL) parrent->LinkNewNode(*this);
    if(parrent != NULL && parrent->is_reachable==true){
        is_reachable=true;
    }
    else if(parrent==NULL) is_reachable=true;
    else is_reachable=false;
}

Node::~Node(){
    Node* parrentof=this->parrent;
    Node* iterator;
    std::vector<Node*> *parrentvector=&(parrentof->childs);

    int i=0;
    for(;i<parrentof->childs.size();i++){
        iterator = parrentof->childs[i];
        if(iterator==this){
            break;
        }
    }

    parrentvector->erase(parrentvector->begin()+i);

    for(int k=0;i<childs.size();i++){
        Node* child;
        std::vector<Node*>::iterator end = childs.end();
        child=*end;
        child->~Node();
    }
}

std::vector<Node*> Node::getChilds(){
    return this->childs;
}

Node* Node::getParrent(){
    return this->parrent;
}

void Node::MakeReachable(bool state){
    for(std::vector<Node*>::iterator it = childs.begin() ;
    it!= childs.end();
    ++it){
        if((*it)->childs.size!=0){
            (*it)->MakeReachable(state);
            (*it)->is_reachable=state;
        }
        if((*it)->childs.size==0){
            (*it)->is_reachable=state;
        }
    }
}

bool Node::operator==(const Node &n2){
    return this==&n2;
}


void Node::LinkNewNode(Node& newnode){
    childs.push_back(&newnode);
}


double Node::GetCost() const{
    return this->cost;
}

double Node::GetDist(const Node &n2) const{
    return (planningUtilities::dist(this->coord,n2.coord));
}

double Node::GetDist(const Coordinate &n2) const{
    return (planningUtilities::dist(this->coord,n2));
}


void Node::ChangeParrent(Node* parrent){
    Node* parrent1= this->parrent;
    int i=0;
    for(;i<parrent1->childs.size();i++){
        if(parrent1->childs[i]==this) break;
    }
    parrent1->childs.erase(parrent1->childs.begin()+i);
    this->parrent=parrent;
    parrent->childs.push_back(this);
    this->cost=parrent->cost + this->GetDist(*parrent);
}

std::vector<Coordinate> Node::GetRoute(){
    std::vector<Coordinate> retvec;
    Node* nodeptr=this;
    while(nodeptr){
        retvec.insert(retvec.begin(),nodeptr->coord);
        nodeptr=nodeptr->parrent;
    }
    return retvec;
}

bool Node::IsInGoal(Coordinate goal, double epsilon){
    if(planningUtilities::dist(this->coord,goal) < epsilon){
        return true;
    }
    else{
        return false;
    }
}

Coordinate Node::GetCoord(){
    return this->coord;
}



#endif