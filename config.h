#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "gl_const.h"
#include <algorithm>
#include <sstream>


class Config
{
public:
    Config();
    int loglevel;
    int connectedness;
    bool allowanyangle;
    bool planforturns;
    double timelimit;
    int rescheduling;
    double inflatecollisionintervals;
    int initialprioritization;
    double startsafeinterval;
    double additionalwait;
    bool use_to_aa_sipp;
    std::string logfilename;
    std::string logpath;
    bool use_perfect_h;

    bool getConfig(const char* fileName);
};

#endif
