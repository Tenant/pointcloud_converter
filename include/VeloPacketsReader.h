//
// Created by akira on 18-7-22.
//

#ifndef VELOPACKETPARSER_ROS_VELOPACKETSREADER_H
#define VELOPACKETPARSER_ROS_VELOPACKETSREADER_H

#include <string>
#include "PacketData.h"
#include <fstream>
#define PACKETSIZE 1206

class VeloPacketsReader {
public:
    VeloPacketsReader();
    ~VeloPacketsReader();
    bool open(std::string packetfile);
    bool nextPacket(PacketData &packet);
    void release();
    void reset();
private:
    std::ifstream ofile;
    unsigned char* packetdata = nullptr;
};


#endif //VELOPACKETPARSER_ROS_VELOPACKETSREADER_H
