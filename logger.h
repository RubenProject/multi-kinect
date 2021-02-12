#pragma once

#include "pch.h"



class Logger : public libfreenect2::Logger
{
public:
    Logger(const char *filename) {
        if (filename)
            logfile_.open(filename);
        level_ = Debug;
    }
    bool good()
    {
        return logfile_.is_open() && logfile_.good();
    }
    virtual void log(Level level, const std::string &message)
    {
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
        std::cout << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
    }

private:
    std::ofstream logfile_;
};
