#ifndef PACKETDATA_H
#define PACKETDATA_H
#include <cstdint>
class PacketData {
private:
	const unsigned char * _data;
	int _size;


public:
	PacketData();
	PacketData(const unsigned char* data);
	~PacketData();
	int64_t packetHeaderTime;

	int size();
	void size(int size);
	const unsigned char* payload(unsigned int offset);
	int64_t getHeaderTime();
    void setHeaderTime(int64_t time);
};

#endif