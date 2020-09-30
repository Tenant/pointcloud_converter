//
// Created by akira on 18-7-25.
//

#ifndef SEGREGION_VELODATACORRECTOR_H
#define SEGREGION_VELODATACORRECTOR_H

#include "LiDARData.hpp"
//#include "velocalib.h"
#include "string"
#include "Utils/RigidTransform/coordinatetrans.hpp"
#include "Utils/pose3d.hpp"
#include "Utils/mathcalc.hpp"

class LiDARDataCorrector {
public:
    static void trans2Coodinate(LiDARData &liDARData, Pose3d<double> tfLink);
    void trans2BaseCoodinate(LiDARData &liDARData) const;
    static void compensateVeloScanDelay(LiDARData & liDARData, const std::vector<Pose3d<double>> tfLinkList);
    static void correctCoordinateAll(LiDARData &data, Pose3d<double> tfLink_base, const std::vector<Pose3d<double>> tfLinkList);
    static void compensateRollPitch(LiDARData &data, Pose3d<double> tfLink);
    void loadCalibInfo(std::string calibfile);
    //WARN:: must operated on complete frame
//    static void setHorizontalInxBias(LiDARData &liDARData, VeloCalib const& calib);
private:
//    VeloCalib _calib;
    Pose3d<double> LiDARPose;
    //WARN: This func can be only used in condition in which all lines share a same lineptnum.
    static void _getSegInxList(LiDARData const&liDARData, int segnum, std::vector<std::pair<int, int>>&inxseglist);
//TODO:: to be deprecated
public:
    static void getSegTimeList(int64_t taitime, int64_t headtime, int segnum, std::vector<int64_t> &timelist);

};


#endif //SEGREGION_VELODATACORRECTOR_H
