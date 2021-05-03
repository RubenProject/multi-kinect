#include "recordmanager.h"

#include "logger.h"
#include "dirent.h"


RecordManager::RecordManager()
{
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (context->mRecordFolder.c_str())) != NULL) {
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


void RecordManager::writeRecording(const std::array<YAML::Node, KINECT_COUNT> &root)
{
    int lowest_idx = getNextIdx();
    for (size_t i = 0; i < KINECT_COUNT; ++i){
        std::string name = context->mRecordFolder 
                           + "I" + std::to_string(lowest_idx)
                           + "K" + std::to_string(i) + ".yaml";
        std::ofstream fout(name.c_str());
        assert(fout.is_open());
        fout << root[i];
        fout.close();

    }
    mRecordIdx.push_back(lowest_idx);
    std::sort(mRecordIdx.begin(), mRecordIdx.end());
}


void RecordManager::readRecording(std::array<YAML::Node, KINECT_COUNT> &root, const int idx)
{
    auto res = std::find(mRecordIdx.begin(), mRecordIdx.end(), idx);
    if (res == std::end(mRecordIdx)) {
        logger->log(libfreenect2::Logger::Error ,"Recording not in database");
        return;
    }
    for (int i = 0; i < KINECT_COUNT; ++i) {
        std::string name = context->mRecordFolder
                           + "I" + std::to_string(idx)
                           + "K" + std::to_string(i) + ".yaml";
        root[i] = YAML::LoadFile(name.c_str());
    }
}

