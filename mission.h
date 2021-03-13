#ifndef MISSION_H
#define MISSION_H

#include "map.h"
#include "config.h"
#include "xmlLogger.h"
#include "searchresult.h"
#include "aa_sipp.h"
#include "aa_sipp_optimal.h"
#include "task.h"
#include "dynamicobstacles.h"
#include "heuristic.h"

class Mission
{
public:
    Mission();
    ~Mission();

    bool getMap();
    bool getTask();
    bool getConfig();
    bool getObstacles();
    void createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();
    void setFileNames(const char *taskName, const char* mapName, const char *configName, const char *obstaclesName);
    void calcHeuristic();
private:
    Map              m_map;
    Task             m_task;
    Heuristic        m_heuristic;
    Config           m_config;
    DynamicObstacles m_obstacles;
    AA_SIPP*         m_pSearch;
    AA_SIPP_optimal* aa_sipp_optimal;
    XmlLogger*       m_pLogger;
    SearchResult     sr;
    const char*      mapName;
    const char*      taskName;
    const char*      configName;
    const char*      obstaclesName;
    std::ofstream    out;
    double h_count_time;

};

#endif

