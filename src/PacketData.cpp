#include "../include/PacketData.h"

PacketData::PacketData() :_data { nullptr } {}

PacketData::PacketData(const unsigned char* data) :_data{ data } {}

PacketData::~PacketData() {
	_data = nullptr;
}

int PacketData::size() {
	return _size;
}

void PacketData::size(int size) {
	_size = size;
}

const unsigned char* PacketData::payload(unsigned int offset) {
	return _data + offset;
}

int64_t PacketData::getHeaderTime(){
    return this->packetHeaderTime;
}

void PacketData::setHeaderTime(int64_t time){
    this->packetHeaderTime = time;
}
