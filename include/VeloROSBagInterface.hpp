//
// Created by akira on 18-10-13.
//

#ifndef SEGREGION_VELOROSBAGINTERFACE_HPP
#define SEGREGION_VELOROSBAGINTERFACE_HPP
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "velodyne_msgs/VelodyneScan.h"
#include "Packet.h"
#define PACKETSIZE 1206
class VeloROSBagInterface{
public:
    VeloROSBagInterface(){};
    VeloROSBagInterface(rosbag::View &view){
        this->iter = view.begin();
        packetinx = 0;

        packet_data = new unsigned char[PACKETSIZE];
    };
    ~ VeloROSBagInterface(){
            if(packet_data!=nullptr)
                delete []packet_data;
    };

    bool nextPacket(rosbag::View &view, PacketData &packet){
        if(packetinx == 0) {
            if (this->iter == view.end()) {
                return false;
            }

            rosbag::MessageInstance const m = *iter;
            iter++;
            scanptr = m.instantiate<velodyne_msgs::VelodyneScan>();
            packets_size = scanptr->packets.size();
        }





        packet = PacketData(this->packet_data);
        packet.size(PACKETSIZE);
        int64_t time = scanptr->packets[packetinx].stamp.sec * 1000000L + scanptr->packets[packetinx].stamp.nsec / 1e3L;
        packet.setHeaderTime(time);
        memcpy((void*)packet_data, (void*)scanptr->packets[packetinx].data.elems, PACKETSIZE);
        packetinx++;

        if(packetinx == packets_size - 1)
            packetinx = 0;
        return true;
    };

    void reset(rosbag::View &view){
        this->iter = view.begin();
        this->packetinx = 0;
    };
private:
    std::vector<std::string> topics;
    std::string topic;
    int64_t messageNum = 0;
    velodyne_msgs::VelodyneScan::ConstPtr scanptr;
    rosbag::View::iterator iter;
    int packets_size = 0;
    int packetinx = 0;
    unsigned char* packet_data = nullptr;
};

#endif //SEGREGION_VELOROSBAGINTERFACE_HPP
