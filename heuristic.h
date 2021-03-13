#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "structs.h"
#include "gl_const.h"
#include <vector>
#include <unordered_map>
#include "map.h"
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/composite_key.hpp>
using boost::multi_index_container;
using namespace boost::multi_index;

struct hNode
{
    int i, j, id;
    double g;
    bool operator<(const hNode& n) const
    {
        if(this->i == n.i)
            return this->j < n.j;
        return this->i < n.i;
    }
    hNode(int _i, int _j, int _id, double _g=-1):i(_i), j(_j), id(_id), g(_g){}
};

typedef multi_index_container<
  hNode,
  indexed_by<
    ordered_non_unique<member<hNode, double, &hNode::g>>,
    hashed_unique<member<hNode, int, &hNode::id>>
  >
> Open_Container;

class Heuristic
{
    std::vector<std::vector<double>> h_values;
    Open_Container open;
    int goal_i, goal_j;
    double dist(const hNode& a, const hNode& b){ return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2)); }
public:
    Heuristic(){}
    void init(unsigned int width, unsigned int height);
    void count(const Map &map, Agent agent);
    void set_goal(int i, int j) { goal_i = i; goal_j = j;}
    unsigned int get_size() const { return h_values[0].size(); }
    double get_value(int i, int j) { if(h_values.empty())
                                        return sqrt(pow(i - goal_i,2) + pow(j - goal_j,2));
                                     return h_values[i][j]; }
};

#endif // HEURISTIC_H
