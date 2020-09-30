#include "../include/VelodyneStreamer.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>

#include "../include/GPSInfo.h"
#include "../include/PacketData.h"
#include "../include/velocalib.h"

#include "iomanip"
VelodyneStreamer::VelodyneStreamer(std::string packetfile, std::string format){
    this->fileformat = format;
    this->packetFile = packetfile;
    if(format == "pcap") {
        packetpayloadoffset = 42;
        _preader.open(packetfile);
        Packet packet;
        while (_preader.nextPacket(packet)) {
            if (packet.size() >= 1248) {
                if ((int)packet.data().payload(packetpayloadoffset)[1247 - 42] == 0x22) {
                    continue;
//                    sensor = SensorType::VLP16;
//                    dual_distance_return = (int)packet.data().payload()[1246 - 42] == 0x39;
                }
                else if ((int)packet.data().payload(packetpayloadoffset)[1247 - 42] == 0x21) {
                    sensor = SensorType::HDL32;
                    calib = new VeloCalib(32);
                }
                else {
                    sensor = SensorType::HDL64;
                    calib = new VeloCalib(64);
                }
                break;
            }
        }
        _preader.reset();
    }
    else if(format == "hdl") {
        packetpayloadoffset = 0;
        _reader.open(packetfile);
        PacketData packet;
        while (_reader.nextPacket(packet)) {
            if (packet.size() == 1206) {
                if ((int)packet.payload(packetpayloadoffset)[1247 - 42] == 0x21) {
                    sensor = SensorType::HDL32;
                    calib = new VeloCalib(32);
                }
                else {
                    sensor = SensorType::HDL64;
                    calib = new VeloCalib(64);
                }
                break;
            }
        }
        _reader.reset();
    }


    _current_frame_idx = 0;
    _residualPacketdata = nullptr;
}

VelodyneStreamer::~VelodyneStreamer() {
    if(fileformat == "pcap")
        _preader.release();
    else
        _reader.release();
    if(calib!= nullptr){
        delete calib;
        calib = nullptr;
    }
    if(_residualPacketdata!= nullptr){
        delete calib;
        calib = nullptr;
    }

}

void VelodyneStreamer::parseAzimuth(const unsigned char* data, int& azimuth) {
  azimuth = ((int)(data[1] << 8) + (int)data[0]);
}

void VelodyneStreamer::parseDataBlock(const unsigned char* data, int& distance, int& intensity) {
  intensity = (int)data[2];
  distance = ((int)(data[1] << 8) + (int)data[0]) * 2;
}

void VelodyneStreamer::parseTimeStamp(const unsigned char* data, unsigned int& timestamp) {
  timestamp = (unsigned int)(data[3] << 24) + (unsigned int)(data[2] << 16) + (unsigned int)(data[1] << 8) + (unsigned int)data[0];
}

bool VelodyneStreamer::parseNMEASentence(const char* data, GPSInfo& gps_info) {
  
  bool validity = false;
  std::string sentence{ data };
  std::string delimiter = ",";

  size_t pos = 0;
  std::string word;
  int index = 0;
  while ((pos = sentence.find(delimiter)) != std::string::npos) {
    word = sentence.substr(0, pos);
    sentence.erase(0, pos + delimiter.length());
    switch(index) {
      case 1: gps_info.timestamp = atoi(word.c_str()); break;
      case 2: validity = (word[0] == 'A');        
      case 3: {
        char* degree = new char[2];
        char* seconds = new char[8];
        memcpy(degree, word.c_str(), 2);
        memcpy(seconds, word.c_str() + 2, 8);
        gps_info.latitude = atof(degree) + atof(seconds) / 60.0f;
        delete[] degree;
        delete[] seconds;
      }; break;
      case 4: gps_info.north_south = word[0];break;
      case 5: {
        char* degree = new char[3];
        char* seconds = new char[8];
        memcpy(degree, word.c_str(), 3);
        memcpy(seconds, word.c_str() + 3, 8);
        gps_info.longitude = atof(degree) + atof(seconds) / 60.0f;
        delete[] degree;
        delete[] seconds;
      }; break; 
      case 6: gps_info.east_west = word[0]; break;
      case 7: gps_info.speed = atof(word.c_str()) * 1.852f / 3.6f; break;
      case 8: gps_info.true_course = atof(word.c_str()); break;
      case 10: gps_info.variation = atof(word.c_str()); break;
      default: break;
    }
    index++;
  }
  gps_info.east_west_variation = sentence[0];

  return validity;
}

int VelodyneStreamer::interpolateAzimuth(int previous_azimuth, int next_azimuth) {
  if (next_azimuth < previous_azimuth)
    next_azimuth = next_azimuth + 36000;

  float azimuth = previous_azimuth + ((next_azimuth - previous_azimuth) / 2);

  if (azimuth >= 36000) azimuth = azimuth - 36000;
  return azimuth;
}

void VelodyneStreamer::handleResidualPacket(PacketData& packet, int inx2process){
    if(this->_residualPacketdata == nullptr)
        _residualPacketdata = new unsigned char[packet.size()];
    this->_residualPacketValidInx = inx2process;
    memcpy(_residualPacketdata, packet.payload(packetpayloadoffset), packet.size());
    _needPushResidualPacket = true;
}


void VelodyneStreamer::handleGPSPacket(PacketData& packet) {
  const unsigned char *payload = packet.payload(packetpayloadoffset);

  payload += 198; // Unused 198 bytes
  unsigned int timestamp;
  parseTimeStamp(payload, timestamp);
  payload += 4; // Timestamp 4 bytes;
  payload += 4; // Unused 4 bytes;

  char *data = new char[72];
  memcpy(data, payload, 72);
  if (strncmp(data, "$GPRMC", 6) == 0) {
    parseNMEASentence(data, gps_info);
  }

  payload += 72; // NMEA $GPRMC sentence 72 bytes
  payload += 234; // Unused 234 bytes
}


bool VelodyneStreamer::nextFrame(LiDARData &cloud) {
    switch (sensor) {

    case SensorType::HDL32: {
        if(fileformat == "pcap")
            return nextFrameHDL32_pcap(cloud);
        else
            return nextFrameHDL32(cloud);
        break;
    }
    case SensorType::HDL64: {

        if(fileformat == "pcap")
            return nextFrameHDL64_pcap(cloud);

        else if(fileformat == "hdl")
            return nextFrameHDL64(cloud);
        break;
    }
    default:
        throw "Not a valid sensor type!";
    }

    return false;
}


bool VelodyneStreamer::nextFrameHDL32(LiDARData &cloud) {
    cloud.clear();
    PacketData packet;
    LaserPoint p;
    pushResidualPacket(cloud);
    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        const unsigned char *payload = packet.payload(packetpayloadoffset);
        int azimuth;
        int distance = 0.0f;
        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth);
//            std::cout<<azimuth<<std::endl;
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    cloud.timeStamp_tail = packet.getHeaderTime();
                    this->handleResidualPacket(packet, n);
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK
                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                p = this->parseDataPointHDL32(distance, r, i, horizontalangle);
                //TODO: calibrate the index
                bool pushflag = cloud[calib->order[i]].push_back(p);
                assert(pushflag);
            }
        }
    }
    return !first;
}


bool VelodyneStreamer::nextFrameHDL64(LiDARData &cloud) {
    cloud.clear();
    PacketData packet;
    LaserPoint p;
    pushResidualPacket(cloud);
    bool first = true;
    float previous_azimuth = -1.0f;
    while (_reader.nextPacket(packet)) {
        const unsigned char *payload = packet.payload(packetpayloadoffset);
//        const unsigned char *status = packet.payload(1246-42);
//        const unsigned char *statusvalue = (status + 1);
//        long long statusv = (long long)statusvalue[0];
//        if((long long)status[0]==0xF9)
//            std::cout << "dual:"<<statusv<<"\n";

        int azimuth;
        int distance = 0.0f;
        int r;
        bool upper;
        for (int n = 0; n < 12; n++) {
            upper = ((long long)(payload[1] << 8) + (long long)payload[0]) == 0xEEFF;
//            std::cout<<upper<<std::endl;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK

            parseAzimuth(payload, azimuth);
//            std::cout<<azimuth<<std::endl;
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    cloud.timeStamp_tail = packet.getHeaderTime();
                    this->handleResidualPacket(packet, n);
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK
                int laserIDInx = i + !upper * 32;
                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                p = this->parseDataPointHDL64(distance, r, laserIDInx, horizontalangle);

                //TODO:: calibration index
//                std::cout<<laserIDInx<<' '<<calibInfo->idx[i]<<std::endl;
//                assert(calibInfo->idx[i] > 31);
                bool pushflag = cloud[calib->order[laserIDInx]].push_back(p);
                assert(pushflag);
            }
        }
    }
    return !first;
}

void VelodyneStreamer::close() {
    _reader.release();
}




void VelodyneStreamer::setCalib(std::string filename)
{
    this->calib->loadCalib(filename);
}

LaserPoint VelodyneStreamer::parseDataPointHDL64(double distance, int intensityValue, int LaserIDInx, float horizontalangle) {
    LaserPoint p;
    distance = 0.1 * distance + calib->distCorrection[LaserIDInx];

    float cosVertAngle = cosf(calib->vertCorrection[LaserIDInx] * M_PI / 180.0f);
    float sinVertAngle = sinf(calib->vertCorrection[LaserIDInx] * M_PI / 180.0f);

    float RotCorrection = calib->rotCorrection[LaserIDInx] * M_PI / 180.0f;

    float sinRotAngle = sinf(horizontalangle - RotCorrection);
    float cosRotAngle = cosf(horizontalangle - RotCorrection);

    float hOffsetCorr = calib->horizOffsetCorrection[LaserIDInx];
    float vOffsetCorr = calib->vertOffsetCorrection[LaserIDInx];

    float xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

    p.x = (xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle);
    p.y = (xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle);
    p.z = (distance * sinVertAngle + vOffsetCorr * cosVertAngle);

    int intensity = intensityValue;
/*
    if (intensityValue != 0) {
        int minIntensity = 0;
        int maxIntensity = 0;
        float intensityScale = 0;
        minIntensity = calibInfo->minIntensity[LaserIDInx];
        maxIntensity = calibInfo->maxIntensity[LaserIDInx];
        //Get intensity scale
        intensityScale = (maxIntensity - minIntensity);
        // Get firing \93i\94  intensity
        // Get firing \93i\94 distance, here unit is 2mm
        distance *= 20000;


        // Calculate offset according calibration
        float focaloffset = 256 * (1 - calibInfo->focalDistance[LaserIDInx] / 13100)*(1 - calibInfo->focalDistance[LaserIDInx] / 13100);
        // get slope from calibration
        float focalslope = calibInfo->focalSlope[LaserIDInx];

        // Calculate corrected intensity vs distance
        float intensityVal1 = intensityValue + focalslope * (abs(focaloffset - 256 * (1 - distance / 65535)*(1 - distance / 65535)));
        if (intensityVal1 < minIntensity) intensityVal1 = minIntensity;
        if (intensityVal1 > maxIntensity) intensityVal1 = maxIntensity;
        // Scale to new intensity scale
        float intensityColor = (intensityVal1 - minIntensity) / intensityScale;

        //(*(*Intensities)++) = intensityColor;
        if (intensityColor > 255)
            p.intensity = 255;
        else
            p.intensity = intensityColor;


        p.x /= 100.0;
        p.y /= 100.0;
        p.z /= 100.0;
    }
*/
    p.x /= 100.0;
    p.y /= 100.0;
    p.z /= 100.0;
    p.intensity = intensityValue;
    return p;
}

LaserPoint VelodyneStreamer::parseDataPointHDL32(double distance, int intensityValue, int blockID, float horizontalangle) {
    LaserPoint p;
    p.intensity = intensityValue;
    p.x = distance / 1000.0f * cosf(calib->vertCorrection[blockID] * M_PI / 180.0f) * sinf(horizontalangle);
    p.y = distance / 1000.0f * cosf(calib->vertCorrection[blockID] * M_PI / 180.0f) * cosf(horizontalangle);
    p.z = distance / 1000.0f * sinf(calib->vertCorrection[blockID] * M_PI / 180.0f);
    return p;
}

void VelodyneStreamer::pushResidualPacket(LiDARData &cloud) {
    if(!_needPushResidualPacket)
        return;
    int azimuth;
    int distance = 0.0f;
    int r;
    bool upper;
    LaserPoint p;
    const unsigned char *payload;
    for (int n = this->_residualPacketValidInx; n < 12; n++) {
        payload = this->_residualPacketdata + 100 * n;
        upper = ((long long)(payload[1] << 8) + (long long)payload[0]) == 0xEEFF;
        payload = payload + 2; // UPPER-LOWER LASER BLOCK
        parseAzimuth(payload, azimuth);
        payload = payload + 2; // AZIMUTH
        for (int i = 0; i < 32; i++) {
            parseDataBlock(payload, distance, r);
            payload = payload + 3; // DATABLOCK
            int laserIDInx = i + !upper * 32;
            float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
            if(sensor == SensorType::HDL64)
                p = this->parseDataPointHDL64(distance, r, laserIDInx, horizontalangle);
            else if(sensor == SensorType::HDL32)
                p = this->parseDataPointHDL32(distance, r, laserIDInx, horizontalangle);
            //TODO:: calibration index
            int lineInx = calib->order[laserIDInx];
//            std::cout<<" "<<upper<<" "<<i<<" "<<lineInx<<'\n';
            bool pushflag = cloud[lineInx].push_back(p);
            assert(pushflag);
        }
    }
    _needPushResidualPacket = false;
}

void VelodyneStreamer::savevdn(std::string filename, int64_t initime_ms) {
    struct point3fi{
        float x;
        float y;
        float z;
        unsigned char i;
    };
    std::ofstream vdnfilestr(filename);
    PacketData packet;
    LaserPoint p;
    bool first = true;
    int packetno = 0;
    while (_reader.nextPacket(packet)) {
        const unsigned char *payload = packet.payload(packetpayloadoffset);
        int azimuth;
        int distance = 0.0f;
        int r;
        bool upper;
        int64_t timestamp = packet.getHeaderTime();
        int shorttimestamp = timestamp/1000  - initime_ms;
        auto a= sizeof(shorttimestamp);
        vdnfilestr.write((char*)&shorttimestamp, sizeof(shorttimestamp));
        for (int n = 0; n < 12; n++) {
            upper = ((long long)(payload[1] << 8) + (long long)payload[0]) == 0xEEFF;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK
            parseAzimuth(payload, azimuth);
            payload = payload + 2; // AZIMUTH
            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK
                int laserIDInx = i + !upper * 32;
                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                p = this->parseDataPointHDL64(distance, r, laserIDInx, horizontalangle);
                point3fi pt;
                pt.x = p.x;
                pt.y = p.y;
                pt.z = p.z;
                pt.i = p.intensity;
                vdnfilestr.write((char*)&pt, sizeof(pt) / sizeof(char));
//                vdnfilestr.write((char*)&p.x, sizeof(p.x));
//                vdnfilestr.write((char*)&p.y, sizeof(p.y));
//                vdnfilestr.write((char*)&p.z, sizeof(p.z));
//                vdnfilestr.write((char*)&p.intensity, sizeof(p.intensity));
            }
        }
        packetno++;
        if(packetno % 1000 == 0)
            std::cout<<"Complete Packets:"<<packetno<<std::endl;
    }
    vdnfilestr.close();
}

bool VelodyneStreamer::nextFrameHDL32_pcap(LiDARData &cloud) {
    cloud.clear();
    Packet packet;
    LaserPoint p;
    pushResidualPacket(cloud);
    bool first = true;
    float previous_azimuth = -1.0f;
    while (_preader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) {
//            this->getVeloIMU_HDL32E(packet);
            continue;
        }
//            this->getVeloIMU_HDL32E(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);
        int azimuth;
        int distance = 0.0f;
        int r;
        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE

            parseAzimuth(payload, azimuth);
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    cloud.timeStamp_tail = packet.data().getHeaderTime();
                    PacketData dta = packet.data();
                    this->handleResidualPacket(dta, n);
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK
                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                p = this->parseDataPointHDL32(distance, r, i, horizontalangle);
                //TODO: calibrate the index
                bool pushflag = cloud[calib->order[i]].push_back(p);
                assert(pushflag);
            }
        }
    }
    return false; // [calc] fix bug
}

bool VelodyneStreamer::nextFrameHDL64_pcap(LiDARData &cloud) {
    cloud.clear();
    Packet packet;
    LaserPoint p;
    pushResidualPacket(cloud);
    bool first = true;
    float previous_azimuth = -1.0f;
    //tmp str
    std::ofstream str;
    str.open("tmp_tahe.bi");
    int tmpi= 0;


    while (_preader.nextPacket(packet)) {
        if (packet.header().incl_len < 1248) continue;
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);
        const unsigned char *status = packet.data().payload(1246);
        const unsigned char *statusvalue = (status + 1);
        long long statusv = (long long)statusvalue[0];
//        std::cout <<status[0]<<" "<<statusvalue[0]<<" "<<statusv<<"\n";





        int azimuth;
        int distance = 0.0f;
        int r;
        bool upper;
        for (int n = 0; n < 12; n++) {
            upper = ((long long)(payload[1] << 8) + (long long)payload[0]) == 0xEEFF;
//            std::cout<<upper<<std::endl;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK

            parseAzimuth(payload, azimuth);
            std::cout<<azimuth<<std::endl;
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    cloud.timeStamp_tail = packet.data().getHeaderTime();
                    PacketData dta = packet.data();
                    this->handleResidualPacket(dta, n);
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {

                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK
                int laserIDInx = i + !upper * 32;
                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                p = this->parseDataPointHDL64(distance, r, laserIDInx, horizontalangle);
                //TODO:: calibration index
//                std::cout<<laserIDInx<<' '<<calibInfo->idx[i]<<std::endl;
//                assert(calibInfo->idx[i] > 31);
                bool pushflag = cloud[calib->order[laserIDInx]].push_back(p);
                assert(pushflag);
            }
        }
    }
    return !first;
}

bool compareAzimuth(float a, float b){
    float diff = a - b;
    while (diff < 0){
        diff += M_PI * 2;
    }
    while (diff > M_PI * 2){
        diff -= M_PI * 2;
    }
    if(diff < M_PI)
        return 1;//a>b
    else
        return 0;//b>a
}

float unifyRadian(float d){
    while (d < 0){
        d += M_PI * 2;
    }
    while (d > M_PI * 2){
        d -= M_PI * 2;
    }
    return d;
}
void VelodyneStreamer::scanPcapPacketTime() {
    this->_preader.scanPacketTimelist(this->packetFile);
    this->packetTimeReady = true;
    this->_preader.release();
    this->_preader.open(this->packetFile);
}
bool VelodyneStreamer::getFrameHDL32(LiDARData &cloud, long long timestamp) {
    //get time index
    if(!this->packetTimeReady)
        this->scanPcapPacketTime();

    auto inxIter = std::lower_bound(this->_preader.packetTimeList.begin(), this->_preader.packetTimeList.end(), timestamp);
    if(inxIter == this->_preader.packetPos.end())
        return false;
    int target_packetinx = inxIter - this->_preader.packetTimeList.begin();
    auto filepos = this->_preader.packetPos[target_packetinx];
    _preader.setCurrentIndex(filepos);
    Packet packet;
    _preader.nextPacket_noPreload(packet);
    const unsigned char *payload = packet.data().payload(packetpayloadoffset);
    int azimuth;
    parseAzimuth(payload + 2, azimuth);
    float currentAzimuthDeg = azimuth / 100.0f / 180.0 * M_PI;

    // deg range of this frame: currentAzimuthDeg - pi ~ currentAzimuthDeg + pi
    float lower_azi = unifyRadian(currentAzimuthDeg - M_PI);
    float upper_azi = unifyRadian(currentAzimuthDeg + M_PI);

    //find packet index of start and end

    float azi = currentAzimuthDeg;
    auto iter = target_packetinx;
    //avoid ambiguous when diff =M_PI
    do{
        iter--;
        auto index = this->_preader.packetPos[iter];
        _preader.setCurrentIndex(index);
        Packet packet;
        _preader.nextPacket_noPreload(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);
        int azimuth;
        parseAzimuth(payload + 2, azimuth);
        azi = azimuth / 100.0f / 180.0 * M_PI;
    }while(compareAzimuth(azi, lower_azi) && iter >= 0);
    auto firstIter = iter;


    azi = currentAzimuthDeg;
    iter = target_packetinx;
    //avoid ambiguous
    do{
        iter++;
        auto index = this->_preader.packetPos[iter];
        _preader.setCurrentIndex(index);
        Packet packet;
        _preader.nextPacket_noPreload(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);
        int azimuth;
        parseAzimuth(payload + 2, azimuth);
        azi = azimuth / 100.0f / 180.0 * M_PI;
    }
    while(compareAzimuth(upper_azi, azi) && iter < this->_preader.packetPos.size());
    auto endIter = iter - 1;

    //get point data
    int distance = 0.0f;
    int r;
    LaserPoint p;
    float firstptazi = -1;
    float endptazi = -1;

    cloud.clear();
    cloud.timeStamp_tail = *inxIter;

    for(int i = firstIter; i <= endIter; i++){
        auto index = this->_preader.packetPos[i];
        _preader.setCurrentIndex(index);
        Packet packet;
        _preader.nextPacket_noPreload(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);

        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE
            parseAzimuth(payload, azimuth);
            float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
            if(i == firstIter){
                if(compareAzimuth(lower_azi, horizontalangle)) {
                    payload = payload + 98;
                    continue;
                }
                if(firstptazi == -1){
                    firstptazi = horizontalangle;
                }
            }
            if(i == endIter){
                if(compareAzimuth(horizontalangle, upper_azi)) {
                    endptazi = horizontalangle;
                    return true;
                }
            }
            payload = payload + 2; // AZIMUTH
            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                p = this->parseDataPointHDL32(distance, r, i, horizontalangle);
                //TODO: calibrate the index
                bool pushflag = cloud[calib->order[i]].push_back(p);
                assert(pushflag);
            }
        }

    }
    return true;
}


#ifdef ENABLE_ROS
VelodyneStreamer::VelodyneStreamer(rosbag::View &view):_rosbagReader(view){
    this->fileformat = "bag";
    PacketData packet;
    while (_rosbagReader.nextPacket(view, packet)) {
        if (packet.size() == 1206) {
            if ((int)packet.payload(packetpayloadoffset)[1247 - 42] == 0x21) {
                sensor = SensorType::HDL32;
                calib = new VeloCalib(32);
            }
            else {
                sensor = SensorType::HDL64;
                calib = new VeloCalib(64);
            }
            break;
        }
    }
    _rosbagReader.reset(view);
}

bool VelodyneStreamer::nextFrameHDL64_bag(LiDARData &cloud, rosbag::View &view) {
    cloud.clear();
    PacketData packet;
    LaserPoint p;
    pushResidualPacket(cloud);
    bool first = true;
    float previous_azimuth = -1.0f;
    while (_rosbagReader.nextPacket(view, packet)) {
        const unsigned char *payload = packet.payload(packetpayloadoffset);

        int azimuth;
        int distance = 0.0f;
        int r;
        bool upper;
        for (int n = 0; n < 12; n++) {
            upper = ((long long)(payload[1] << 8) + (long long)payload[0]) == 0xEEFF;
//            std::cout<<upper<<std::endl;
            payload = payload + 2; // UPPER-LOWER LASER BLOCK

            parseAzimuth(payload, azimuth);
//            std::cout<<azimuth<<std::endl;
            if (first) {
                previous_azimuth = azimuth;
                first = false;
            }
            else {
                if (azimuth < previous_azimuth) {
                    cloud.timeStamp_tail = packet.getHeaderTime();
                    this->handleResidualPacket(packet, n);
                    return true;
                }
            }
            previous_azimuth = azimuth;
            payload = payload + 2; // AZIMUTH

            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK
                int laserIDInx = i + !upper * 32;
                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
                p = this->parseDataPointHDL64(distance, r, laserIDInx, horizontalangle);

                //TODO:: calibration index
//                std::cout<<laserIDInx<<' '<<calibInfo->idx[i]<<std::endl;
//                assert(calibInfo->idx[i] > 31);
                bool pushflag = cloud[calib->order[laserIDInx]].push_back(p);
                assert(pushflag);
            }
        }
    }
    return !first;}

bool VelodyneStreamer::nextFrame_bag(LiDARData &cloud, rosbag::View &view) {
    packetpayloadoffset = 0;
    switch (sensor) {


        case SensorType::HDL64: {

            if(fileformat == "bag")
                return nextFrameHDL64_bag(cloud, view);
            break;
        }
        default:
            throw "Not a valid sensor type!";
    }

    return false;}

void VelodyneStreamer::getVeloIMU_HDL32E(Packet &packet) {
    if(packet.header().incl_len != 512 + 42)
        return;
    else{
        if(imulist.size() > 100)
            imulist.resize(100);
        const unsigned char* data = packet.data().payload(56);
        int g1 = ((int)(data[1] << 8) + (int)data[0]) & 0x0fff;
        int temp1 = ((int)(data[3] << 8) + (int)data[2]) & 0x0fff;
        int a1x = ((int)(data[5] << 8) + (int)data[4]) & 0x0fff;
        int a1y = ((int)(data[7] << 8) + (int)data[6]) & 0x0fff;
        int g2 = ((int)(data[9] << 8) + (int)data[8]) & 0x0fff;
        int temp2 = ((int)(data[11] << 8) + (int)data[10]) & 0x0fff;
        int a2x = ((int)(data[13] << 8) + (int)data[12]) & 0x0fff;
        int a2y = ((int)(data[15] << 8) + (int)data[14]) & 0x0fff;
        int g3 = ((int)(data[17] << 8) + (int)data[16]) & 0x0fff;
        int temp3 = ((int)(data[19] << 8) + (int)data[18]) & 0x0fff;
        int a3x = ((int)(data[21] << 8) + (int)data[20]) & 0x0fff;
        int a3y = ((int)(data[23] << 8) + (int)data[22]) & 0x0fff;

        double g1_v = g1 * this->gyrScaleFactor;
        double a1x_v = a1x * this->accScaleFactor;
        double a1y_v = a1y * this->accScaleFactor;
        double temp1_v = temp1 * this->tempScaleFactor + tempBias;

        double g2_v = g2 * this->gyrScaleFactor;
        double a2x_v = a2x * this->accScaleFactor;
        double a2y_v = a2y * this->accScaleFactor;
        double temp2_v = temp2 * this->tempScaleFactor + tempBias;

        double g3_v = g3 * this->gyrScaleFactor;
        double a3x_v = a3x * this->accScaleFactor;
        double a3y_v = a3y * this->accScaleFactor;
        double temp3_v = temp3 * this->tempScaleFactor + tempBias;
        auto st = std::ofstream("veloimu.txt", std::ios_base::app);
        st << std::fixed << std::setprecision(5)<< packet.header().ts_sec + packet.header().ts_usec / 1e6 << ' '
           << g1_v << ' '  << a1x_v << ' ' << a1y_v << ' '
           << g2_v << ' ' << a2x_v << ' ' << a2y_v << ' '
           << g3_v << ' ' << a3x_v << ' ' << a3y_v << std::endl;

    }

}

bool compareAzimuth(float a, float b){
    float diff = a - b;
    while (diff < 0){
        diff += M_PI * 2;
    }
    while (diff > M_PI * 2){
        diff -= M_PI * 2;
    }
    if(diff < M_PI)
        return 1;//a>b
    else
        return 0;//b>a
}

float unifyRadian(float d){
    while (d < 0){
        d += M_PI * 2;
    }
    while (d > M_PI * 2){
        d -= M_PI * 2;
    }
    return d;
}

bool VelodyneStreamer::getFrameHDL32(LiDARData &cloud, long long timestamp) {
    //get time index
    if(!this->packetTimeReady)
        this->scanPcapPacketTime();

    auto inxIter = std::lower_bound(this->_preader.packetTimeList.begin(), this->_preader.packetTimeList.end(), timestamp);
    if(inxIter == this->_preader.packetPos.end())
        return false;
    int target_packetinx = inxIter - this->_preader.packetTimeList.begin();
    auto filepos = this->_preader.packetPos[target_packetinx];
    _preader.setCurrentIndex(filepos);
    Packet packet;
    _preader.nextPacket_noPreload(packet);
    const unsigned char *payload = packet.data().payload(packetpayloadoffset);
    int azimuth;
    parseAzimuth(payload + 2, azimuth);
    float currentAzimuthDeg = azimuth / 100.0f / 180.0 * M_PI;

    // deg range of this frame: currentAzimuthDeg - pi ~ currentAzimuthDeg + pi
    float lower_azi = unifyRadian(currentAzimuthDeg - M_PI);
    float upper_azi = unifyRadian(currentAzimuthDeg + M_PI);

    //find packet index of start and end

    float azi = currentAzimuthDeg;
    auto iter = target_packetinx;
    //avoid ambiguous when diff =M_PI
    do{
        iter--;
        auto index = this->_preader.packetPos[iter];
        _preader.setCurrentIndex(index);
        Packet packet;
        _preader.nextPacket_noPreload(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);
        int azimuth;
        parseAzimuth(payload + 2, azimuth);
        azi = azimuth / 100.0f / 180.0 * M_PI;
    }while(compareAzimuth(azi, lower_azi) && iter >= 0);
    auto firstIter = iter;


    azi = currentAzimuthDeg;
    iter = target_packetinx;
    //avoid ambiguous
    do{
        iter++;
        auto index = this->_preader.packetPos[iter];
        _preader.setCurrentIndex(index);
        Packet packet;
        _preader.nextPacket_noPreload(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);
        int azimuth;
        parseAzimuth(payload + 2, azimuth);
        azi = azimuth / 100.0f / 180.0 * M_PI;
    }
    while(compareAzimuth(upper_azi, azi) && iter < this->_preader.packetPos.size());
    auto endIter = iter - 1;

    //get point data
    int distance = 0.0f;
    int r;
    LaserPoint p;
    float firstptazi = -1;
    float endptazi = -1;

    cloud.clear();
    cloud.timeStamp_tail = *inxIter;

    for(int i = firstIter; i <= endIter; i++){
        auto index = this->_preader.packetPos[i];
        _preader.setCurrentIndex(index);
        Packet packet;
        _preader.nextPacket_noPreload(packet);
        const unsigned char *payload = packet.data().payload(packetpayloadoffset);

        for (int n = 0; n < 12; n++) {
            payload = payload + 2; // 0xFFEE
            parseAzimuth(payload, azimuth);
            float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
            if(i == firstIter){
                if(compareAzimuth(lower_azi, horizontalangle)) {
                    payload = payload + 98;
                    continue;
                }
                if(firstptazi == -1){
                    firstptazi = horizontalangle;
                }
            }
            if(i == endIter){
                if(compareAzimuth(horizontalangle, upper_azi)) {
                    endptazi = horizontalangle;
                    return true;
                }
            }
            payload = payload + 2; // AZIMUTH
            for (int i = 0; i < 32; i++) {
                parseDataBlock(payload, distance, r);
                payload = payload + 3; // DATABLOCK

                p = this->parseDataPointHDL32(distance, r, i, horizontalangle);
                //TODO: calibrate the index
                bool pushflag = cloud[calib->order[i]].push_back(p);
                assert(pushflag);
            }
        }

    }
    return true;
}

void VelodyneStreamer::scanPcapPacketTime() {
    this->_preader.scanPacketTimelist(this->packetFile);
    this->packetTimeReady = true;
    this->_preader.release();
    this->_preader.open(this->packetFile);
}

//void VelodyneStreamer::savevdn(std::string filename, std::vector<std::pair<int, int>> timelist) {
//
//
//    struct point3fi{
//        float x;
//        float y;
//        float z;
//        unsigned char i;
//    };
//    std::ofstream vdnfilestr(filename);
//    PacketData packet;
//    LaserPoint p;
//    bool first = true;
//    int packetno = 0;
//    int64_t initime_ms = 0;
//    int timeIntervalInx = 0;
//    while (_reader.nextPacket(packet)) {
//        const unsigned char *payload = packet.payload(packetpayloadoffset);
//        int azimuth;
//        int distance = 0.0f;
//        int r;
//        bool upper;
//        int64_t timestamp = packet.getHeaderTime();
//
//        if(initime_ms == 0){
//            int64_t longTimeStamp = timestamp;//ms
//            auto localtime_ = localtime(&longTimeStamp);
//            auto tm_ = localtime_;
//            tm_->tm_hour = tm_->tm_min = tm_->tm_sec = 0;
//            initime_ms = mktime(tm_);
//        }
//
//        int shorttimestamp = timestamp/1000  - initime_ms;
//        int stTime = timelist[timeIntervalInx].first;
//        int endTime = timelist[timeIntervalInx].second;
//
//        if(shorttimestamp < stTime)
//            continue;
//        while (shorttimestamp > endTime){
//            timeIntervalInx++;
//            if(timeIntervalInx >= timelist.size())
//                return;
//            int stTime = timelist[timeIntervalInx].first;
//            int endTime = timelist[timeIntervalInx].second;
//        }
//        if(shorttimestamp < stTime)
//            continue;
//
//
//        auto a= sizeof(shorttimestamp);
//        vdnfilestr.write((char*)&shorttimestamp, sizeof(shorttimestamp));
//        for (int n = 0; n < 12; n++) {
//            upper = ((long long)(payload[1] << 8) + (long long)payload[0]) == 0xEEFF;
//            payload = payload + 2; // UPPER-LOWER LASER BLOCK
//            parseAzimuth(payload, azimuth);
//            payload = payload + 2; // AZIMUTH
//            for (int i = 0; i < 32; i++) {
//                parseDataBlock(payload, distance, r);
//                payload = payload + 3; // DATABLOCK
//                int laserIDInx = i + !upper * 32;
//                float horizontalangle = azimuth / 100.0f * M_PI / 180.0f;
//                p = this->parseDataPointHDL64(distance, r, laserIDInx, horizontalangle);
//                point3fi pt;
//                pt.x = p.x;
//                pt.y = p.y;
//                pt.z = p.z;
//                pt.i = p.intensity;
//                vdnfilestr.write((char*)&pt, sizeof(pt) / sizeof(char));
//
//            }
//        }
//        packetno++;
//        if(packetno % 1000 == 0)
//            std::cout<<"Complete Packets:"<<packetno<<std::endl;
//    }
//    vdnfilestr.close();
//}


#endif


