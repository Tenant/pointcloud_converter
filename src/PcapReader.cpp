#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "../include/PcapReader.h"

PcapReader::PcapReader() :_index{ 24 }, _pcap{ nullptr }, _header{ nullptr } {}

PcapReader::PcapReader(std::string file_name) : _index{ 24 }, _pcap{ nullptr }, _header{ nullptr } {
	if (!open(file_name)) {
		if (_pcap != nullptr) {
			delete[] _pcap;
			_pcap = nullptr;
		}
		if (_header != nullptr) {
			delete _header;
			_header = nullptr;
		}
	}
}

PcapReader::~PcapReader() {
	release();
}

bool PcapReader::open(std::string file_name) {


	file.open(file_name, std::ios::in | std::ios::binary | std::ios::ate);
	if (file.is_open())
	{
		this->filesize = file.tellg();
		int64_t segmentlen = 500 * 1024 * 1024L;
        if(filesize > segmentlen)//500M
            readsize = segmentlen;
        else
            readsize = (int64_t)this->filesize;
		memblock = new char[readsize];
		file.seekg(0, std::ios::beg);
		file.read(memblock, readsize);
        this->filepos = readsize;
//		file.close();

		_pcap_size = (int64_t)this->filesize;
		_pcap = reinterpret_cast<const unsigned char*>(memblock);
	}
	else 
		return false;
	
	return true;
}

void PcapReader::split(std::string file_name, int count) {
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	std::vector<std::ofstream> ofs(count);
	if (file.is_open()) {
		file.seekg(0, file.end);
		long long size = file.tellg();
		float size_per_count = size / (float)count;
		file.seekg(0);

		for (int i = 0; i < count; i++) 
			ofs[i].open(file_name + "_" + std::to_string(i) + ".pcap", std::ifstream::out | std::ifstream::binary);
		
		char* block;
		block = new char[24];
		file.read(block, 24);
		_pcap = reinterpret_cast<const unsigned char*>(block);		
		header();

		for (int i = 0; i < count; i++)
			ofs[i].write(block, 24);
		
		delete[] block;
		_pcap = nullptr;

		long long index = 24;
		while (index < size) {
			
			PacketHeader pheader;
			
			block = new char[16];
			file.read(block, 16);
			pheader.parseHeaderData(reinterpret_cast<const unsigned char*>(block));
			ofs[(int)(index / size_per_count)].write(block, 16);
			index += 16;
			delete[] block;

			block = new char[pheader.incl_len];
			file.read(block, pheader.incl_len);
			ofs[(int)(index / size_per_count)].write(block, pheader.incl_len);
			
			index += pheader.incl_len;
			delete[] block;
		}
	}

	file.close();
}

void PcapReader::skip(std::string file_name) {
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	std::ofstream ofs;
	if (file.is_open()) {
		file.seekg(0, file.end);
		long long size = file.tellg();
		file.seekg(0);

		ofs.open(file_name + "_skip.pcap", std::ifstream::out | std::ifstream::binary);

		char* block;
		block = new char[24];
		file.read(block, 24);
		_pcap = reinterpret_cast<const unsigned char*>(block);
		header();

		ofs.write(block, 24);

		delete[] block;
		_pcap = nullptr;

		long long index = 24;
		bool skip = false;
		while (index < size) {
			skip = false;
			PacketHeader pheader;

			block = new char[16];
			file.read(block, 16);
			pheader.parseHeaderData(reinterpret_cast<const unsigned char*>(block));
			if (pheader.incl_len < 1248) {
				skip = true;
			}
			if (!skip) ofs.write(block, 16);
			index += 16;
			delete[] block;

			block = new char[pheader.incl_len];
			file.read(block, pheader.incl_len);
			if (!skip) ofs.write(block, pheader.incl_len);

			index += pheader.incl_len;
			delete[] block;
		}
	}

	file.close();
}


PcapHeader PcapReader::header() {
	if (_header != nullptr) 
		return *_header;

	_header = new PcapHeader();

	int32_t temp32;
	uint32_t tempu32;
	uint16_t tempu16;
	for (int i = 0; i < 24; i++) {
		temp32 = _pcap[i] << 8 * ((i % 4));
		tempu32 = _pcap[i] << 8 * ((i % 4));
		tempu16 = _pcap[i] << 8 * ((i % 4));
		if (i < 4) _header->magic_number += tempu32;
		else if (i < 6) _header->version_major += tempu16;
		else if (i < 8) _header->version_minor += tempu16;
		else if (i < 12) _header->thiszone += temp32;
		else if (i < 16) _header->sigfigs += tempu32;
		else if (i < 20) _header->snaplen += tempu32;
		else if (i < 24) _header->network += tempu32;
	}

	return *_header;
}

void PcapReader::release() {
	if (_header != nullptr) {
		delete _header;
		_header = nullptr;
	}

    //release memory_block too
//	if (_pcap != nullptr) { // [calc] fix bug
//        delete[] _pcap;
//		_pcap = nullptr;
//	}

    if(file.is_open()){
        file.close();
    }
    if(block != nullptr) {
        delete[](block);
        block = nullptr;
    }
}

void PcapReader::reset() {
	_index = 24;
}

bool PcapReader::nextPacket(Packet& packet) {
    if (_index >= this->readsize && this->filepos >= this->filesize)
		return false;
	int lastinx = _index;

    if (_index + 1264 >= this->readsize) {
        this->filepos -=  (this->readsize - lastinx);

        auto residual = this->filesize - this->filepos;
        if(residual <= 1264) {//packetsize = 1248 + 16
            //close file
            file.close();
            if (this->memblock != nullptr) {
                delete[](this->memblock);
                this->memblock = nullptr;
            }
            return false;
        }
        if(residual < this->readsize)
            this->readsize = residual;

        file.seekg(this->filepos, std::ios::beg);
        file.read(memblock, readsize);
        this->filepos += readsize;
        _pcap = reinterpret_cast<const unsigned char*>(memblock);
        this->_index = 0;
        this->nextPacket(packet);
    }

	PacketHeader* packet_h = new PacketHeader();
	packet_h->parseHeaderData(_pcap + _index);
	_index += 16;

	PacketData* packet_data = new PacketData(_pcap + _index);
	packet_data->size(packet_h->incl_len);
	_index += packet_data->size();

	packet.release();
	packet = Packet(packet_h, packet_data);
	int64_t time = packet_h->ts_sec * 1e6 + packet_h->ts_usec;
	packet_data->setHeaderTime(time);
	return true;
}

bool PcapReader::nextPacket_noPreload(Packet& packet) {
    if(this->block == nullptr)
        this->block = new char[1264];
    if (_index + 1264 > this->filesize)
        return false;

    this->file.seekg(_index, file.beg);
    file.read(block, 1264);

    PacketHeader* packet_h = new PacketHeader();
    packet_h->parseHeaderData(reinterpret_cast<const unsigned char*>(block));
    _index += 16;

    PacketData* packet_data = new PacketData(reinterpret_cast<const unsigned char*>(block + 16));
    packet_data->size(packet_h->incl_len);
    _index += packet_data->size();

    packet.release();
    packet = Packet(packet_h, packet_data);
    int64_t time = packet_h->ts_sec * 1e6 + packet_h->ts_usec;
    packet_data->setHeaderTime(time);

    return true;
}


bool PcapReader::previousPacket(Packet& packet) {
	// std::cout << "Entering previousPacket(Packet& packet)" << std::endl;
	// std::cout << "  " << _index << " < 16 + 1248 == " << (_index < 16 + 1248) << std::endl;
	if (_index < 16 + 1248) return false;

	PacketHeader* packet_h = new PacketHeader();
	packet_h->parseHeaderData(_pcap + _index - 16 - 1248);
	
	// std::cout << "  load index: " << _index - 16 - 1248 << std::endl;
	// std::cout << "  load incl_len:" << packet_h->incl_len << std::endl;

	int number_of_gps_packets = 1;
	while(packet_h->incl_len != 1248) {
		packet_h->parseHeaderData(_pcap + _index - ((16 + 554) * number_of_gps_packets) - 16 - 1248);
		// std::cout << "  1st while index: " << _index - ((16 + 554) * number_of_gps_packets) - 16 - 1248 << std::endl;
		// std::cout << "  1st while incl_len: " << packet_h->incl_len << std::endl;
		number_of_gps_packets++;
	}
	_index = _index - ((16 + 554) * (number_of_gps_packets - 1)) - 16 - 1248;
	// std::cout << "  1st new index: " << _index << std::endl;

	packet_h->parseHeaderData(_pcap + _index);
	// std::cout << "  1st incl_len: " << packet_h->incl_len << std::endl;

	number_of_gps_packets = 1;
	while(packet_h->incl_len != 1248) {
		packet_h->parseHeaderData(_pcap + _index - ((16 + 554) * number_of_gps_packets) - 16 - 1248);
		// std::cout << "  2nd while index: " << _index - ((16 + 554) * number_of_gps_packets) - 16 - 1248 << std::endl;
		// std::cout << "  2nd while incl_len: " << packet_h->incl_len << std::endl;
		number_of_gps_packets++;
	}
	_index = _index - ((16 + 554) * (number_of_gps_packets - 1)) - 16 - 1248;
	// std::cout << "  2nd new index: " << _index << std::endl;
	// std::cout << "  2nd incl_len: " << packet_h->incl_len << std::endl;
	_index += 16;

	PacketData* packet_data = new PacketData(_pcap + _index);
	packet_data->size(packet_h->incl_len);
	_index += packet_data->size();	

	packet.release();
	packet = Packet(packet_h, packet_data);

	return true;
}

void PcapReader::scanPacketTimelist(std::string file_name) {
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
//	std::ofstream outfile("packet_timestamp.csv");
    long long validPacketNum = 0;
	if (file.is_open()) {
		file.seekg(0, file.end);
		long long size = file.tellg();
		file.seekg(0);

		char* block;
		block = new char[24];
		file.read(block, 24);
		_pcap = reinterpret_cast<const unsigned char*>(block);
		header();


		delete[] block;
		_pcap = nullptr;

		long long index = 24;
        block = new char[16];

		while (index < size) {
			PacketHeader pheader;
			file.read(block, 16);
			pheader.parseHeaderData(reinterpret_cast<const unsigned char*>(block));
			if (pheader.incl_len == 1248) {
                long long t = pheader.ts_sec * 1e6 + pheader.ts_usec;
                validPacketNum += 1;
                this->packetTimeList.push_back(t);
                this->packetPos.push_back(index);
                if(validPacketNum % 100000==0)
                    std::cout << "Parsing Packet Time(Frames):" << validPacketNum << std::endl;
			}
			index += 16;
            file.seekg(index + pheader.incl_len, file.beg);
			index += pheader.incl_len;
		}
        delete[] block;
	}

	file.close();
//	outfile.close();
    std::cout << "Parsing Packet Time Over(Total Frames):" << validPacketNum << std::endl;
}
