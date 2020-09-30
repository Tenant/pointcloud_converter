//
// Created by akira on 18-7-25.
//

#include "LiDARDataCorrector.h"
using rigidtrans::CoordinateTrans;
using rigidtrans::Point3d;
using rigidtrans::eulerRPY;
using MathCalc::mulMatrix3d;

void tmpdisplaymat(double *mat){
    for(int i = 0; i !=9;i++){
        std::cout<<mat[i]<<',';
        if(i%3==2)
            std::cout<<'\n';
    }
    std::cout<<'\n';
}

void tmpdisplayvec(Point3d a){
    std::cout<<a.x<<'\n';
    std::cout<<a.y<<'\n';
    std::cout<<a.z<<'\n';
    std::cout<<'\n';
}
//TODO??????????????first shift????
void LiDARDataCorrector::trans2Coodinate(LiDARData &velodata, Pose3d<double> tfLink) {
    eulerRPY rot(tfLink.roll, tfLink.pitch, tfLink.yaw);
    Point3d shv = Point3d(tfLink.x, tfLink.y, tfLink.z);
    for(int i = 0; i != velodata.lineNum; i++) {
        float *x = velodata[i]._x_ptr.get();
        float *y = velodata[i]._y_ptr.get();
        float *z = velodata[i]._z_ptr.get();
        CoordinateTrans::transform<float>(x, y, z, 0, velodata[i]._ptNum, rot);
        CoordinateTrans::transform<float>(x, y, z, 0, velodata[i]._ptNum, shv);
    }
}
//TODO:: wrong!!!! use quaternion!!! -- addressed -- to be tested
void LiDARDataCorrector::compensateVeloScanDelay(LiDARData &data, const std::vector<Pose3d<double>> tfLinkList) {
    int segnum = 10;
    assert(tfLinkList.size() == segnum);
    Pose3d<double> pose;
    //use tail time to compensate
    pose = tfLinkList[segnum - 1];
//    eulerRPY rot_new_(pose.roll, pose.pitch, pose.yaw);
    eulerRPY rot_new_(0.0, 0.0, pose.yaw);
    auto rot_new = CoordinateTrans::euler2quaternion(rot_new_).inverse();
    double *rotMatrix = new double[9];
    double *rotMatrix_new = new double[9];
    double *rotMatrix_accu = new double[9];
    CoordinateTrans::getTransMatrix(rotMatrix_new, rot_new);
    std::vector<std::pair<int, int>>inxseglist;
    std::vector<std::pair<int, int>> seglist;
    LiDARDataCorrector::_getSegInxList(data, segnum, seglist);

    for (int s = 0; s != seglist.size(); s++) {
        auto seg = seglist[s];
        rigidtrans::eulerRPY rot(tfLinkList[s].roll, tfLinkList[s].pitch, tfLinkList[s].yaw);
        Point3d shv = Point3d(-pose.x + tfLinkList[s].x, -pose.y + tfLinkList[s].y, -pose.z + tfLinkList[s].z);
        CoordinateTrans::getTransMatrix(rotMatrix, rot);
        mulMatrix3d(rotMatrix_new, rotMatrix, rotMatrix_accu);
//        tmpdisplaymat(rotMatrix_new);
//        tmpdisplaymat(rotMatrix);
//        tmpdisplaymat(rotMatrix_accu);
        Point3d shv_acc = CoordinateTrans::transform(shv, rot_new);
        for (int i = 0; i != data.lineNum; i++) {
            float *x = data[i]._x_ptr.get();
            float *y = data[i]._y_ptr.get();
            float *z = data[i]._z_ptr.get();

            CoordinateTrans::transform<float>(x, y, z, seg.first, seg.second, rotMatrix_accu);
            CoordinateTrans::transform<float>(x, y, z, seg.first, seg.second, shv_acc);
        }
    }
    delete []rotMatrix;
    delete []rotMatrix_new;
    delete []rotMatrix_accu;
}

void LiDARDataCorrector::correctCoordinateAll(LiDARData &data, Pose3d<double> tfLink_base, const std::vector<Pose3d<double>> tfLinkList) {
    int segnum = 10;
    assert(tfLinkList.size() == segnum);
    Pose3d<double> pose;
    Point3d pt;
    //use tail time to compensate
    pose = tfLinkList[segnum - 1];
    eulerRPY rot_2_(pose.roll, pose.pitch, pose.yaw);
    auto rot_2 = (CoordinateTrans::euler2quaternion(rot_2_)).inverse();
    eulerRPY rot_base(tfLink_base.roll, tfLink_base.pitch, tfLink_base.yaw);
    Point3d shv_base(tfLink_base.x, tfLink_base.y, tfLink_base.z);
    double *rotMatrix_base = new double[9];
    double *rotMatrix_1 = new double[9];
    double *rotMatrix_2 = new double[9];
    double *rotMatrix_2_1_base =  new double[9];
    double *rotMatrix_2_1 =  new double[9];

    CoordinateTrans::getTransMatrix(rotMatrix_2, rot_2);
    CoordinateTrans::getTransMatrix(rotMatrix_base, rot_base);

    std::vector<std::pair<int, int>>inxseglist;
    std::vector<std::pair<int, int>> seglist;
    LiDARDataCorrector::_getSegInxList(data, segnum, seglist);

    for (int s = 0; s != seglist.size(); s++) {
        auto seg = seglist[s];
        rigidtrans::eulerRPY rot_1(tfLinkList[s].roll, tfLinkList[s].pitch, tfLinkList[s].yaw);
        Point3d shv = Point3d(-pose.x + tfLinkList[s].x, -pose.y + tfLinkList[s].y, -pose.z + tfLinkList[s].z);
        CoordinateTrans::getTransMatrix(rotMatrix_1, rot_1);
        mulMatrix3d(rotMatrix_2, rotMatrix_1, rotMatrix_2_1);
        mulMatrix3d(rotMatrix_2_1, rotMatrix_base, rotMatrix_2_1_base);
        shv = CoordinateTrans::transform(shv, rot_2);
        Point3d shv2 = CoordinateTrans::transform(shv_base, rotMatrix_2_1);
        shv = Point3d(shv.x + shv2.x, shv.y + shv2.y, shv.z + shv2.z);
        for (int i = 0; i != data.lineNum; i++) {
            float *x = data[i]._x_ptr.get();
            float *y = data[i]._y_ptr.get();
            float *z = data[i]._z_ptr.get();
            CoordinateTrans::transform<float>(x, y, z, seg.first, seg.second, rotMatrix_2_1_base);
            CoordinateTrans::transform<float>(x, y, z, seg.first, seg.second, shv);
        }
    }
    delete []rotMatrix_base;
    delete []rotMatrix_1;
    delete []rotMatrix_2;
    delete []rotMatrix_2_1_base;
    delete []rotMatrix_2_1;

}
//
//void LiDARDataCorrector::loadCalibInfo(std::string calibfile) {
//    _calib.loadCalib(calibfile);
//    this->LiDARPose = Pose3d<double>(_calib.position_x,
//                                     _calib.position_y,
//                                     _calib.position_z,
//                                     _calib.orientation_r,
//                                     _calib.orientation_p,
//                                     _calib.orientation_y);
//}

void LiDARDataCorrector::trans2BaseCoodinate(LiDARData &velodata) const {
    trans2Coodinate(velodata, this->LiDARPose);
}

void LiDARDataCorrector::_getSegInxList(LiDARData const &velodata, int segnum, std::vector<std::pair<int, int>> &inxseglist) {
    if(!velodata.getLineNum())
        return;
    inxseglist.clear();
    int lineptnum = velodata[0].size();
    int step = lineptnum / segnum;
    for(int i = 1; i != segnum; i++){
        inxseglist.push_back(std::make_pair((i - 1) * step, i * step));
    }
    inxseglist.push_back(std::make_pair((segnum - 1) * step, lineptnum));
}

void LiDARDataCorrector::getSegTimeList(int64_t taitime, int64_t headtime, int segnum, std::vector<int64_t> &timelist) {
    timelist.clear();
    int framemili = taitime - headtime;
    assert(framemili > 80 || framemili < 120);
    int milisecstep = framemili / segnum;
    int64_t firstone = headtime + milisecstep / 2;
    for (int i = 0; i != segnum; i++){
         timelist.push_back(firstone);
        firstone += milisecstep;
    }
}

//void LiDARDataCorrector::setHorizontalInxBias(VeloData &velodata, VeloCalib const &calib) {
//    assert(velodata.lineNum && velodata[0].size());
//    double horiRes = 360.0 / velodata[0].size();
//    for (int i = 0; i != velodata.lineNum; i++) {
//        int inxbias = calib.rotCorrection[calib.idx[i]] / horiRes;
//        velodata[i].setHorizontalInxBias(inxbias);
//    }
//}

void LiDARDataCorrector::compensateRollPitch(LiDARData &data, Pose3d<double> tfLink) {
    Pose3d<double> targetPose;
    targetPose = tfLink;
    targetPose.roll = 0.0;
    targetPose.pitch = 0.0;


}

