#include "heuristic.h"

void Heuristic::init(unsigned int width, unsigned int height)
{
    h_values.clear();
    h_values.resize(height);
    for(unsigned int i = 0; i < height; i++)
        h_values[i].resize(width, CN_INFINITY);
}

void Heuristic::count(const Map &map, Agent agent)
{
    LineOfSight los;
    los.setSize(0.5);
    hNode curNode(agent.goal_i, agent.goal_j, 0);
    open.clear();
    open.insert(curNode);
    int k=0;
    while(!open.empty())
    {
        curNode = *open.get<0>().begin();
        open.get<0>().erase(open.get<0>().begin());
        k++;
        h_values[curNode.i][curNode.j] = curNode.g;
        int h = map.height, w = map.width;
        for(int i1=-h; i1<h; i1++)
            for(int j1=-w; j1<w; j1++)
            {
                hNode newNode(curNode.i + i1, curNode.j + j1, (curNode.i + i1)*map.width + curNode.j + j1);
                newNode.g = curNode.g + dist(curNode, newNode);
                if(map.CellOnGrid(newNode.i, newNode.j))
                    if(!map.CellIsObstacle(newNode.i, newNode.j))
                        if(h_values[newNode.i][newNode.j] > newNode.g)
                            if(los.checkLine(curNode.i, curNode.j, newNode.i, newNode.j, map))
                            {
                                h_values[newNode.i][newNode.j] = newNode.g;
                                auto it = open.get<1>().find(newNode.id);
                                if(it != open.get<1>().end())
                                {
                                    if(it->g > newNode.g)
                                        open.get<1>().erase(it);
                                    else
                                        continue;
                                }
                                open.insert(newNode);
                            }
            }
    }
}
