#ifndef AA_SIPP_OPTIMAL_H
#define AA_SIPP_OPTIMAL_H
#include "constraints_o.h"
#include "constraints.h"
#include "StatesContainer.h"
#include "structs.h"
#include "searchresult.h"
#include <iomanip>
#include "dynamicobstacles.h"
#include "heuristic.h"
#include "aa_sipp.h"

#ifdef __linux__
    #include <sys/time.h>
#else
    #include <windows.h>
#endif
class AA_SIPP_optimal
{

public:
    AA_SIPP_optimal();
    ~AA_SIPP_optimal();
    ResultPathInfo findPath(const Map &Map, Agent agent, DynamicObstacles &obstacles, Heuristic &h_values_);

private:

    double calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j);
    bool lineOfSight(int i1, int j1, int i2, int j2, const Map &map);
    void findSuccessors(const Node curNode, const Map &Map, std::list<Node> &succs, int numOfCurAgent){}
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);
    void initStates(Agent agent, const Map &Map);
    void addConstraints(){}
    std::vector<conflict> CheckConflicts();
    int constraints_type;
    unsigned int closeSize, openSize;
    std::list<Node> lppath;
    std::vector<Node> hppath;
    Constraints_o constraints;
    StatesContainer states;
    Heuristic h_values;
};

#endif // AA_SIPP_OPTIMAL_H
