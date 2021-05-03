#pragma once

#include "pch.h"



class Logger : public libfreenect2::Logger
{
public:
    Logger(const char *filename);
    bool good();
    void setLevel(Level level);
    virtual void log(Level level, const std::string &message);
private:
    Level output_level;
    std::ofstream logfile_;
};

extern Logger *logger;
