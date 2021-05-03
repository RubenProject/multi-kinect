#include "logger.h"


Logger::Logger(const char *filename)
{
    if (filename){
        logfile_.open(filename);
        level_ = Debug;
        if (good()){
            libfreenect2::setGlobalLogger(this);
        }
    }
    output_level = Level::Debug;
}


bool Logger::good()
{
    return logfile_.is_open() && logfile_.good();
}

void Logger::setLevel(Level level)
{
    output_level = level;
}


void Logger::log(Level level, const std::string &message)
{
    if (level <= output_level) {
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "]" << message << std::endl;
        std::cout << "[" << libfreenect2::Logger::level2str(level) << "]" << message << std::endl;
    }

}
