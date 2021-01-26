#pragma once

#include "common.h"


class RecordManager
{
public:
    RecordManager(std::shared_ptr<Context> context);
    ~RecordManager();

    void writeRecording(const std::vector<std::string> &bodyString);
    void readRecording(std::vector<std::string> &bodyString, const int idx);
private:
    int getNextIdx();

    std::vector<int> mRecordIdx;
    std::shared_ptr<Context> mContext;
};
