#include "recordmanager.h"

#include <fstream>
#include <algorithm>
#include "dirent.h"


RecordManager::RecordManager(std::shared_ptr<Context> context)
    :mContext(context)
{
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (mContext->mRecordFolder.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            if (strcmp(ent->d_name, ".") == 0)
                continue;
            else if (strcmp(ent->d_name, "..") == 0)
                continue;
            else {
                int idx = 0;
                for (size_t i = 1; i < strlen(ent->d_name); ++i) {
                    if (ent->d_name[i] == 'K') {
                        break;
                    }
                    if (ent->d_name[i] >= '0' && ent->d_name[i] <= '9') {
                        idx *= 10;
                        idx += ent->d_name[i] - '0';
                    }
                }
                if (std::find(mRecordIdx.begin(), mRecordIdx.end(), idx) == mRecordIdx.end()){
                    mRecordIdx.push_back(idx);
                }
            }
        }
    }
    std::sort(mRecordIdx.begin(), mRecordIdx.end());
}


RecordManager::~RecordManager()
{}


int RecordManager::getNextIdx()
{
    int lowest = 0;
    for (auto idx : mRecordIdx) {
        if (idx == lowest) {
            lowest++;
        } else {
            return lowest;
        }
    }
    return lowest;
}


void RecordManager::writeRecording(const std::vector<std::string> &bodyString)
{
    int lowest_idx = getNextIdx();
    for (int i = 0; i < KINECT_COUNT; ++i) {
        std::ofstream os((mContext->mRecordFolder
                         + "I" + std::to_string(lowest_idx)
                         + "K" + std::to_string(i) + ".txt"
                         ).c_str(), std::ofstream::out);
        if (os.is_open()) {
            os << bodyString[i];
            os.close();
        }
    }
    mRecordIdx.push_back(lowest_idx);
    std::sort(mRecordIdx.begin(), mRecordIdx.end());
}


void RecordManager::readRecording(std::vector<std::string> &bodyString, const int idx)
{
    for (int i = 0; i < KINECT_COUNT; ++i) {
        std::string filename = mContext->mRecordFolder
                         + "I" + std::to_string(idx)
                         + "K" + std::to_string(i) + ".txt";
        std::ifstream is(filename.c_str());
        if (is) {
            is.seekg(0, is.end);
            int length = is.tellg();
            std::cout << length << std::endl;
            is.seekg(0, is.beg);
            char *buffer = new char[length];
            is.read(buffer, length);
            if (!is){
                std::cout << "failed to read shader!" << std::endl;
            }
            std::string s(buffer);
            delete[] buffer;
            bodyString.push_back(s);
        } else {
            std::cout << "failed to read shader!" << std::endl;
            bodyString.push_back(std::string());
        }
    }
}


