//
// Created by akira on 18-7-22.
//

#include "../include/VeloPacketsReader.h"

VeloPacketsReader::VeloPacketsReader() {

}

bool VeloPacketsReader::open(std::string packetfile) {
    ofile.open(packetfile);
    this->packetdata = new unsigned char[PACKETSIZE];
    return ofile.fail();
}

void VeloPacketsReader::release() {
    ofile.close();
    if(this->packetdata != nullptr) {
        delete[] this->packetdata;
        this->packetdata = nullptr;
    }
}

bool VeloPacketsReader::nextPacket(PacketData &packet) {
    packet = PacketData(this->packetdata);
    packet.size(PACKETSIZE);
    long long time;
    ofile.read((char*)&time, 8);
    if(ofile.gcount() != 8)
        return false;
    packet.setHeaderTime(time);
    ofile.read((char*)this->packetdata, PACKETSIZE);
    if(ofile.gcount() != PACKETSIZE)
        return false;
    return true;
}

VeloPacketsReader::~VeloPacketsReader() {
    this->release();
}

void VeloPacketsReader::reset() {
    ofile.seekg(0, std::ios::beg);
}
