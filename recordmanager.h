#pragma once

#include "pch.h"
#include "common.h"
#include "context.h"


class RecordManager
{
public:
    RecordManager();
    ~RecordManager();

    //void writeRecording(const std::vector<std::string> &bodyString);
    void writeRecording(const std::array<YAML::Node, KINECT_COUNT> &root);

    //void readRecording(std::vector<std::string> &bodyString, const int idx);
    void readRecording(std::array<YAML::Node, KINECT_COUNT> &root, const int idx);
private:
    int getNextIdx();

    std::vector<int> mRecordIdx;
};
