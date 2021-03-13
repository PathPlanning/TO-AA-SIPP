#include"mission.h"

Mission::Mission()
{
    m_pSearch = nullptr;
    m_pLogger = nullptr;
}

Mission::~Mission()
{
    delete m_pSearch;
    delete aa_sipp_optimal;
    delete m_pLogger;
}

void Mission::setFileNames(const char *taskName, const char *mapName, const char *configName, const char *obstaclesName)
{
    this->mapName = mapName;
    this->taskName = taskName;
    this->configName = configName;
    this->obstaclesName = obstaclesName;
}

void Mission::calcHeuristic()
{
    if(m_config.use_perfect_h)
    {
        LARGE_INTEGER begin1, end1, freq1;
        QueryPerformanceCounter(&begin1);
        QueryPerformanceFrequency(&freq1);
        m_heuristic.init(m_map.width, m_map.height);
        m_heuristic.count(m_map, m_task.getAgent(0));
        QueryPerformanceCounter(&end1);
        h_count_time = static_cast<double long>(end1.QuadPart-begin1.QuadPart) / freq1.QuadPart;
    }
    return;
}
bool Mission::getMap()
{
    return m_map.getMap(mapName);
}

bool Mission::getTask()
{
    return (m_task.getTask(taskName) && m_task.validateTask(m_map));
}

bool Mission::getConfig()
{
    return m_config.getConfig(configName);
}

bool Mission::getObstacles()
{
    if(obstaclesName)
        return m_obstacles.getObstacles(obstaclesName);
    else
        return false;
}

void Mission::createSearch()
{
    aa_sipp_optimal = new AA_SIPP_optimal();
    m_pSearch = new AA_SIPP(m_config);
}

void Mission::createLog()
{
    if(m_config.loglevel != CN_LOGLVL_NO)
    {
        m_pLogger = new XmlLogger(m_config.loglevel);
        m_pLogger->createLog(taskName);
    }
}

void Mission::startSearch()
{
    //std::cout<<"SEARCH STARTED\n";
    if(m_config.use_to_aa_sipp)
    {
        sr.pathInfo[0] = aa_sipp_optimal->findPath(m_map, m_task.getAgent(0), m_obstacles, m_heuristic);
        sr.pathfound = sr.pathInfo[0].pathfound;
    }
    else
        sr = m_pSearch->startSearch(m_map, m_task, m_obstacles, m_heuristic);
}

void Mission::printSearchResultsToConsole()
{
    std::cout<<"Runtime: "<<sr.pathInfo[0].runtime
             <<"\nSolution cost: "<<sr.pathInfo[0].pathlength
             <<"\nNodes created: "<<sr.pathInfo[0].nodescreated
             <<"\nSteps: "<<sr.pathInfo[0].numberofsteps
             <<"\nAverage valid moves: "<<sr.pathInfo[0].avg_valid_moves<<" \n";
}

void Mission::saveSearchResultsToLog()
{
    if(m_config.loglevel == CN_LOGLVL_NO)
        return;
    //std::cout<<"LOG STARTED\n";
    m_pLogger->writeToLogInput(taskName, mapName, configName, obstaclesName);
    m_pLogger->writeToLogSummary(sr);
    if(sr.pathfound)
    {
        m_pLogger->writeToLogMap(m_map, sr);
        m_pLogger->writeToLogPath(sr, m_task, m_config);
    }
    m_pLogger->saveLog();
    //std::cout<<"LOG SAVED\n";
}

