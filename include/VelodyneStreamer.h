#ifndef VELODYNESTREAMER_H
#define VELODYNESTREAMER_H

#include <list>
#include <vector>
#include "./LiDARData.hpp"
#include <cstring>
#include "GPSInfo.h"
#include "./velocalib.h"
#include "VeloPacketsReader.h"
#include "./LaserPoint.hpp"
#include "PcapReader.h"
//#define ENABLE_ROS

#ifdef ENABLE_ROS

//todo:: implement horizontal bias modification in velostreamer for 64HDL(no problem for 32HDL because all rotCorrection_= 0 in 32db.xml)

#include "VeloROSBagInterface.hpp"
#include "Utils/IMUinfo.hpp"
#endif
//#include "basicdatastruc.hpp"

enum struct SensorType { HDL64, HDL32};

class VelodyneStreamer {
//private:
public:
	void parseAzimuth(const unsigned char* data, int& azimuth);
	void parseDataBlock(const unsigned char* data, int& distance, int& intensity);
	void parseTimeStamp(const unsigned char* data, unsigned int& timestamp);
	bool parseNMEASentence(const char* data,GPSInfo& gps_info);
	void handleGPSPacket(PacketData& packet);
    void handleResidualPacket(PacketData& packet, int inx2process);
	int interpolateAzimuth(int previous_azimuth, int next_azimuth);
    VeloCalib *calib;
    unsigned char* _residualPacketdata;
	bool _needPushResidualPacket = false;
	bool nextFrameHDL32(LiDARData &cloud);
    bool nextFrameHDL64(LiDARData &cloud);
    bool nextFrameHDL32_pcap(LiDARData &cloud);
    bool nextFrameHDL64_pcap(LiDARData &cloud);
	bool getFrameHDL32(LiDARData &cloud, long long timestamp);
	void scanPcapPacketTime();

	LaserPoint parseDataPointHDL32(double distance, int intensityValue, int blockID, float horizontalangle);
    LaserPoint parseDataPointHDL64(double distance, int intensityValue, int blockID, float horizontalangle);
    void pushResidualPacket(LiDARData &cloud);


    std::vector<int> _frame_pointers;
	int _current_frame_idx;
    int _residualPacketValidInx;
//protected:
public:
    VeloPacketsReader _reader;
    PcapReader _preader;
    std::string fileformat;
	unsigned int packetpayloadoffset = 0;
	std::string packetFile;
	bool packetTimeReady = false;

public:
	VelodyneStreamer(std::string packetfile, std::string format);
#ifdef ENABLE_ROS

	VelodyneStreamer(rosbag::View &view);
#endif
	~VelodyneStreamer();

	void close();
    void setCalib(std::string filename);
    bool nextFrame(LiDARData &cloud);
#ifdef ENABLE_ROS

	bool nextFrame_bag(LiDARData &cloud, rosbag::View &view);
#endif

	SensorType sensor;
	GPSInfo gps_info;
	bool dual_distance_return;
	void savevdn(std::string filename, int64_t initime);
    void savevdn(std::string filename, std::vector<std::pair<int, int>> timelist);

#ifdef ENABLE_ROS
public:
    VeloROSBagInterface _rosbagReader;
private:
	std::list<IMUinfo> imulist;
	void getVeloIMU_HDL32E(Packet &packet);
	double gyrScaleFactor = 0.09766; //degree/sec
	double tempScaleFactor = 0.1453;
	double tempBias = 25.0;
	double accScaleFactor = 0.001221; //G

#endif
};

#endif
