#ifndef PCAPREADER_H
#define PCAPREADER_H

#include <cstdint>
#include <string>

#include "PcapHeader.h"
#include "Packet.h"

class PcapReader {
private:
	int64_t _index;
	int64_t _pcap_size;
	unsigned const char* _pcap;
	PcapHeader* _header;
	std::streampos filesize;
	int64_t readsize;
	int64_t filepos;
	std::ifstream file;
	char* memblock;


public:
	PcapReader();
	PcapReader(std::string);
	~PcapReader();

	bool open(std::string);

	void release();
	void reset();

	void split(std::string file_name, int count);
	void skip(std::string file_name);

	PcapHeader header();
	bool nextPacket(Packet& packet);
	bool previousPacket(Packet& packet);
	
	int currentIndex() { return _index; };
	void setCurrentIndex(long long index) { _index = index; };

	void scanPacketTimelist(std::string file_name);
    std::vector<long long>packetTimeList;
    std::vector<long long>packetPos;
    bool nextPacket_noPreload(Packet& packet);
	char *block = nullptr;

};

#endif