#ifndef Constraints_o_H
#define Constraints_o_H

#include <vector>
#include <math.h>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include "StatesContainer.h"

class Constraints_o
{
public:
    Constraints_o();
    ~Constraints_o(){}
    void init(int width, int height);
    double findEAT(const Node& curNode);
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
    std::vector<std::pair<int,int>> findConflictCells(Node begin, Node end);
    void updateSafeIntervals(const std::vector<std::pair<int,int>> &cells, section sec, bool goal);
    void addConstraints(const std::vector<Node> &sections);
    std::vector<std::pair<double, double> > findIntervals(Node curNode, std::vector<double> &EAT, StatesContainer &states, int w);
    std::pair<double,double> getSafeInterval(int i, int j, int n) {return safe_intervals[i][j][n];}
    std::vector<std::pair<double,double>> getSafeBegins(int i, int j){return safe_intervals[i][j];}
    void copy(const std::vector<std::vector<std::vector<std::pair<double,double>>>> &safe_intervals, const std::vector<std::vector<std::vector<section>>> &constraints)
    {
        this->safe_intervals = safe_intervals;
        this->constraints = constraints;
    }
protected:
    std::pair<double,double> countInterval(section sec, Node curNode);
    std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    std::vector<std::vector<std::vector<section>>> constraints;
    int checkIntersection(Point A, Point B, Point C, Point D, Point &intersec);
    double dist(Node A, Node B){return sqrt(pow(A.i - B.i, 2) + pow(A.j - B.j, 2));}
    double dist(Point A, Point B){return sqrt(pow(A.i - B.i, 2) + pow(A.j - B.j, 2));}
    double minDist(Point A, Point C, Point D);
    double dist(Point A, Point C, Point D)
    {
        return fabs((C.i-D.i)*A.j+(D.j-C.j)*A.i+(C.j*D.i-D.j*C.i))/sqrt(pow(C.i-D.i,2)+pow(C.j-D.j,2));
    }
};

#endif // Constraints_o_H
