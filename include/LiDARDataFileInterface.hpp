//
// Created by akira on 18-7-23.
//

#ifndef LIDARDATAFILEINTERFACE_HPP
#define LIDARDATAFILEINTERFACE_HPP

#include "LiDARData.hpp"
#include "fstream"

//TODO::  Serious bug is to fix the method for accessing to  "maxptnum"
#define VLD_HEADER_FLAG  0xffffffff

class LiDARDataFileInterface {
public:
    static bool withHeader(std::ifstream &file) {
        int headerFlag = 0;
        auto curPos = file.tellg();
        file.seekg(0, std::ios_base::beg);
        file.read((char *) &headerFlag, sizeof(int));
        bool withHeader = ((headerFlag ^ VLD_HEADER_FLAG) == 0x00000000);
        file.seekg(curPos);
        return withHeader;
    }

    static std::pair<int, int> getDataInfo(std::ifstream &file) {
        assert(withHeader(file));
        auto curPos = file.tellg();
        file.seekg(0, std::ios_base::beg);
        int headerFlag = 0;
        file.read((char *) &headerFlag, sizeof(int));
        int lineNum = 0;
        int maxLinePtNum = 0;
        file.read((char *) &lineNum, sizeof(int));
        file.read((char *) &maxLinePtNum, sizeof(int));
        file.seekg(curPos);
        return std::make_pair(lineNum, maxLinePtNum);
    }

    static void writeHeader(std::ofstream &file, LiDARData const &data) {
        auto curPos = file.tellp();
        file.seekp(0, std::ios_base::beg);
        auto begPos = file.tellp();
        assert(begPos == curPos);
        int headFlag = VLD_HEADER_FLAG;
        file.write((char *) (&headFlag), sizeof(int) / sizeof(char));
        file.write((char *) (&data.lineNum), sizeof(int) / sizeof(char));
        file.write((char *) (&data.maxLinePtNum), sizeof(int) / sizeof(char));
    }

    static void writeHeader(std::ofstream &file, int lineNum, int maxLinePtNum) {
        auto curPos = file.tellp();
        file.seekp(0, std::ios_base::beg);
        auto begPos = file.tellp();
        assert(begPos == curPos);
        int headFlag = VLD_HEADER_FLAG;
        file.write((char *) (&headFlag), sizeof(int) / sizeof(char));
        file.write((char *) (&lineNum), sizeof(int) / sizeof(char));
        file.write((char *) (&maxLinePtNum), sizeof(int) / sizeof(char));
    }

    static void writeFrameData(LiDARData const &data, std::ofstream &file) {
        int fixframept = data.maxLinePtNum;
        float *tmp = nullptr;
        tmp = new float[fixframept];
        assert(tmp != nullptr);

        file.write((char *) (&data.timeStamp_tail), sizeof(int64_t) / sizeof(char));
        for (int line = 0; line != data.getLineNum(); line++) {
            int slptnum = fixframept - data[line].size();
            int lineptnum = data[line].size();
            file.write((char *) (&lineptnum), sizeof(int) / sizeof(char));
            file.write((char *) (data[line].getXptr()), lineptnum * sizeof(float) / sizeof(char));
            file.write((char *) (tmp), slptnum * sizeof(float) / sizeof(char));
            file.write((char *) (data[line].getYptr()), lineptnum * sizeof(float) / sizeof(char));
            file.write((char *) (tmp), slptnum * sizeof(float) / sizeof(char));
            file.write((char *) (data[line].getZptr()), lineptnum * sizeof(float) / sizeof(char));
            file.write((char *) (tmp), slptnum * sizeof(float) / sizeof(char));
            file.write((char *) (data[line].getIptr()), lineptnum);
            file.write((char *) (tmp), slptnum);
        }

        assert(tmp != nullptr);
        delete[]tmp;
    }

    static int64_t getFrameBufferSize(int maxLinePtNum, int linenum) {
        return (sizeof(int64_t) + (sizeof(int) / sizeof(char) + sizeof(float) * maxLinePtNum * 3LL +
                                   sizeof(unsigned char) * maxLinePtNum) * linenum);
    }

    static size_t getFileSize(std::ifstream &file) {
        auto curPos = file.tellg();
        file.seekg(0, std::ios_base::beg);
        auto stPos = file.tellg();
        file.seekg(0, std::ios_base::end);
        auto endPos = file.tellg();
        int64_t srcSize = endPos - stPos;
        file.seekg(curPos);
        return srcSize;
    }

    static int getTotalFrameNum(std::ifstream &file, int linenum, int maxLinePtNum) {
        int64_t fileSize = getFileSize(file);
        int64_t frmSize = LiDARDataFileInterface::getFrameBufferSize(maxLinePtNum, linenum);
        if (withHeader(file)) {
            assert(!((fileSize - sizeof(int) * 3) % frmSize));
            return (fileSize - sizeof(int) * 3) / frmSize;
        } else {
            assert(!(fileSize % frmSize));
            return fileSize / frmSize;
        }

    }

    static bool readFrame(std::ifstream &file, LiDARData &data) {
        if (file.eof())
            return false;
        if (withHeader(file)) {
            auto curPos = file.tellg();
            file.seekg(0, std::ios_base::beg);
            auto begPos = file.tellg();
            file.seekg(curPos);

            auto header = getDataInfo(file);
            assert(header.first == data.lineNum);
            assert(header.second == data.maxLinePtNum);
            if (curPos == begPos) {
                int tmp = 0;
                file.read((char *) &tmp, sizeof(int));
                file.read((char *) &tmp, sizeof(int));
                file.read((char *) &tmp, sizeof(int));
            }
        }
        data.clear();
        int fixframept = data.maxLinePtNum;
        file.read((char *) (&data.timeStamp_tail), sizeof(int64_t) / sizeof(char));
        for (int line = 0; line != data.getLineNum(); line++) {
            file.read((char *) (&data[line]._ptNum), sizeof(int) / sizeof(char));
            int cnt = file.gcount();
            if (cnt != sizeof(int) / sizeof(char))
                return false;
            int slptnum = fixframept - data[line].size();
            file.read((char *) (data[line]._x_ptr.get()), data[line]._ptNum * sizeof(float) / sizeof(char));
            file.seekg(slptnum * sizeof(float) / sizeof(char), std::ios_base::cur);
            file.read((char *) (data[line]._y_ptr.get()), data[line]._ptNum * sizeof(float) / sizeof(char));
            file.seekg(slptnum * sizeof(float) / sizeof(char), std::ios_base::cur);
            file.read((char *) (data[line]._z_ptr.get()), data[line]._ptNum * sizeof(float) / sizeof(char));
            file.seekg(slptnum * sizeof(float) / sizeof(char), std::ios_base::cur);
            file.read((data[line]._i_ptr.get()), data[line]._ptNum);
            file.seekg(slptnum, std::ios_base::cur);
        }
        return true;
    }

    static bool readFrame(std::ifstream &file, LiDARData &data, int frameNum) {
        //update: with new format of "*.vld"
        if (!withHeader)
            file.seekg(frameNum * LiDARDataFileInterface::getFrameBufferSize(data.maxLinePtNum, data.lineNum), std::ios::beg);
        else
            file.seekg(sizeof(int) * 3 + frameNum * LiDARDataFileInterface::getFrameBufferSize(data.maxLinePtNum, data.lineNum), std::ios::beg);
        return readFrame(file, data);
    }

    static void writeFrameData2xyzi(LiDARData &data, std::ofstream &file, int maxframePtnum = 2200) {
        int fixframept = data.maxLinePtNum;
//        data.timeStamp_tail *= 1000;
        file.write((char *) (&data.timeStamp_tail), sizeof(int64_t) / sizeof(char));
        char *tmp = new char[maxframePtnum];
        for (int line = 0; line != data.getLineNum(); line++) {
            int lineptnum = data[line].size();
            file.write((char *) (&lineptnum), sizeof(int) / sizeof(char));
            for (int j = 0; j != lineptnum; j++) {
                auto pt = data[line][j];
                file.write((char *) (&pt.x), sizeof(float) / sizeof(char));
                file.write((char *) (&pt.y), sizeof(float) / sizeof(char));
                file.write((char *) (&pt.z), sizeof(float) / sizeof(char));
                file.write((char *) (&pt.intensity), sizeof(char) / sizeof(char));
            }
            int sufbytes = (sizeof(float) / sizeof(char) * 3 + 1) * (maxframePtnum - lineptnum);
            file.write(tmp, sufbytes);
        }
        delete[](tmp);
    }
};

#endif //LIDARDATAFILEINTERFACE_HPP
