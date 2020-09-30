//
// Created by akira on 18-7-20.
//

#ifndef LIDARDATA_HPP
#define LIDARDATA_HPP

#include "./LaserPoint.hpp"
#include <memory>
#include <vector>
#include <assert.h>
#include "./label.hpp"
#include "iostream"
#include "cstring"
//todo:: delete set/getHorizontalInxBias, implement its func in velostreamer
class LineData {
    friend class LiDARDataFileInterface;
    friend class LiDARDataCorrector;
public:
    LineData(int maxLinePtNum = 4000, bool needLabel = false) {
        this->maxLinePtNum = maxLinePtNum;
        this->needLabel = needLabel;
        _ptNum = 0;
        _horizontalBias = 0;
        _x_ptr = std::shared_ptr<float>(new float[maxLinePtNum](), [](float *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }
        });
        _y_ptr = std::shared_ptr<float>(new float[maxLinePtNum](),
                                        [](float *p) {
                                            if (p != nullptr) {
                                                delete[] p;
                                                p = nullptr;
                                            }});
        _z_ptr = std::shared_ptr<float>(new float[maxLinePtNum](), [](float *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }});
        _i_ptr = std::shared_ptr<char>(new char[maxLinePtNum](), [](char *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }});
        if (needLabel) {
            _l_ptr = std::shared_ptr<Label>(new Label[maxLinePtNum], [](Label *p) {
                if (p != nullptr) {
                    delete[] p;
                    p = nullptr;
                }
            });
        }
    };
    ~LineData(){
//        std::cout << "dele LineData" << std::endl;
        _x_ptr = nullptr;
        _y_ptr = nullptr;
        _z_ptr = nullptr;
        _i_ptr = nullptr;
        _l_ptr = nullptr;
    }


    void initializePtNum(int ptNum) {
        assert(ptNum  < maxLinePtNum);
        this->_ptNum = ptNum;
    }


    // Here the _horizontalBias valid----------------
    void setHorizontalInxBias(int bias) {
        this->_horizontalBias = bias;
    };
    int getHorizontalInxBias(int bias) const {
        return this->_horizontalBias;
    };

    LineData clone() const {
//        std::cout << "=======clone LineData" << std::endl;

        LineData tmp (this->maxLinePtNum, this->needLabel);
        memcpy(tmp._x_ptr.get(), _x_ptr.get(), maxLinePtNum * sizeof(float));
        memcpy(tmp._y_ptr.get(), _y_ptr.get(), maxLinePtNum * sizeof(float));
        memcpy(tmp._z_ptr.get(), _z_ptr.get(), maxLinePtNum * sizeof(float));
        memcpy(tmp._i_ptr.get(), _i_ptr.get(), maxLinePtNum * sizeof(char));
        tmp._ptNum = _ptNum;
        tmp._horizontalBias = _horizontalBias;
        if (needLabel) {
            memcpy(tmp._l_ptr.get(), _l_ptr.get(), maxLinePtNum * sizeof(Label));
        }
        return tmp;
    }

    LineData& operator =(const LineData &b) {
        maxLinePtNum = b.maxLinePtNum;
        needLabel = b.needLabel;
        _ptNum = b._ptNum;
        _horizontalBias = b._horizontalBias;
        _x_ptr = b._x_ptr;
        _y_ptr = b._y_ptr;
        _z_ptr = b._z_ptr;
        _i_ptr = b._i_ptr;
        _l_ptr = b._l_ptr;
        return *this;
    }

    LineData(LineData const&b){
        this->maxLinePtNum = b.maxLinePtNum;
        this->needLabel = b.needLabel;
        this->_ptNum = b._ptNum;
        this->_horizontalBias = b._horizontalBias;
        this->_x_ptr = std::shared_ptr<float>(new float[maxLinePtNum](), [](float *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }
        });
        this->_y_ptr = std::shared_ptr<float>(new float[maxLinePtNum](), [](float *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }
        });
        this->_z_ptr = std::shared_ptr<float>(new float[maxLinePtNum](), [](float *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }
        });
        this->_i_ptr = std::shared_ptr<char>(new char[maxLinePtNum](), [](char *p) {
            if (p != nullptr) {
                delete[] p;
                p = nullptr;
            }
        });
        memcpy(this->_x_ptr.get(), b._x_ptr.get(), this->maxLinePtNum * sizeof(float));
        memcpy(this->_y_ptr.get(), b._y_ptr.get(), this->maxLinePtNum * sizeof(float));
        memcpy(this->_z_ptr.get(), b._z_ptr.get(), this->maxLinePtNum * sizeof(float));
        memcpy(this->_i_ptr.get(), b._i_ptr.get(), this->maxLinePtNum * sizeof(char));
        if (needLabel) {
            this->_l_ptr = std::shared_ptr<Label>(new Label[maxLinePtNum],
                                                  [](Label *p) {
                if (p != nullptr) {
                    delete[] p;
                    p = nullptr;
                }
            });
            memcpy(this->_l_ptr.get(), b._l_ptr.get(), this->maxLinePtNum * sizeof(Label));
        }
    }


    inline float getX(size_t inx_) const {
        assert(inx_ < _ptNum); auto inx = _getCorrectedInx(inx_); return _x_ptr.get()[inx];
    };
    inline float getY(size_t inx_) const {
        assert(inx_ < _ptNum); auto inx = _getCorrectedInx(inx_); return _y_ptr.get()[inx];
    };
    inline float getZ(size_t inx_) const {
        assert(inx_ < _ptNum); auto inx = _getCorrectedInx(inx_); return _z_ptr.get()[inx];
    };
    inline unsigned char getI(size_t inx_) const {
        assert(inx_ < _ptNum); auto inx = _getCorrectedInx(inx_); return _i_ptr.get()[inx];
    };
    inline Label getL(size_t inx_) const {
        assert(needLabel && (inx_ < _ptNum)); auto inx = _getCorrectedInx(inx_); return _l_ptr.get()[inx];
    };
    void setL(size_t inx_, Label::LabelType v) {
        assert(needLabel && (inx_ < _ptNum)); auto inx = _getCorrectedInx(inx_); _l_ptr.get()[inx].set(v);
    };
    const LaserPoint operator [](int inx_) const {
        assert(inx_ < _ptNum);
        auto inx = _getCorrectedInx(inx_);
        const LaserPoint a = LaserPoint(_x_ptr.get()[inx], _y_ptr.get()[inx], _z_ptr.get()[inx], _i_ptr.get()[inx]);
        return a;
    };
    void setToZero() {
        memset((void*)(this->_x_ptr.get()), 0, maxLinePtNum * sizeof(float));
        memset((void*)(this->_y_ptr.get()), 0, maxLinePtNum * sizeof(float));
        memset((void*)(this->_z_ptr.get()), 0, maxLinePtNum * sizeof(float));
        memset((void*)(this->_i_ptr.get()), 0, maxLinePtNum * sizeof(char));
    }
    bool resize(int pointnum) {
        if (this->_ptNum < pointnum) {
            return false;
        } else {
            this->_ptNum = pointnum;
        }
        return true;
    }

    bool setPoint(int inx_, LaserPoint p) {
        assert(inx_ < _ptNum);
        auto inx = _getCorrectedInx(inx_);
        _x_ptr.get()[inx] = p.x;
        _y_ptr.get()[inx] = p.y;
        _z_ptr.get()[inx] = p.z;
        _i_ptr.get()[inx] = p.intensity;
    }
    // ----------------------------------------------
    bool push_back(LaserPoint p) {
//        assert(_ptNum < maxLinePtNum);
        if (_ptNum >= maxLinePtNum) {
            return false;
        }
        setX(_ptNum, p.x);
        setY(_ptNum, p.y);
        setZ(_ptNum, p.z);
        setI(_ptNum, p.intensity);
        _ptNum++;
        return true;
    };
    void clear() {
        _ptNum = 0;
    };
    int size() const {
        return _ptNum;
    };
    const float * getXptr() const & {
        return this->_x_ptr.get();
    };
    const float * getYptr() const & {
        return this->_y_ptr.get();
    };
    const float * getZptr() const & {
        return this->_z_ptr.get();
    };
    const char * getIptr() const & {
        return this->_i_ptr.get();
    };

    int maxLinePtNum;
    bool needLabel;

private:

    void setX(size_t inx, float v) {
        _x_ptr.get()[inx] = v;
    };
    void setY(size_t inx, float v) {
        _y_ptr.get()[inx] = v;
    };
    void setZ(size_t inx, float v) {
        _z_ptr.get()[inx] = v;
    };
    void setI(size_t inx, unsigned char v) {
        _i_ptr.get()[inx] = v; assert(v >= 0);
    };
    size_t _getCorrectedInx(size_t inx_) const {
        return (_horizontalBias + inx_ + _ptNum) % _ptNum;
    }

    int _ptNum;
    std::shared_ptr<float> _x_ptr, _y_ptr, _z_ptr;
    std::shared_ptr<char> _i_ptr;
    std::shared_ptr<Label> _l_ptr;
    int _horizontalBias;

};

class LiDARData {
public:
    LiDARData(int lineNum = 64, int maxLinePtNum = 4000, bool needLabel = false) {
        this->maxLinePtNum = maxLinePtNum;
        this->needLabel = needLabel;
        this->lineNum = lineNum;
        for (int i = 0; i != lineNum; i++) {
            _ptInfo.push_back(LineData(maxLinePtNum, needLabel));
        }
    }
    void setLinePtNum(int ptNum) {
        assert(ptNum < maxLinePtNum);
        for (auto &line: _ptInfo) {
            line.initializePtNum(ptNum);
        }
    }


    LiDARData clone() const {
        LiDARData data(lineNum, maxLinePtNum, needLabel);
        for (int i = 0; i != data.lineNum; i++) {
            data._ptInfo[i] = _ptInfo[i].clone();
        }
        data.timeStamp_tail = timeStamp_tail;
        data.timestamp = timestamp;
        return data;
    }

    unsigned lineNum;
    int maxLinePtNum;
    bool needLabel;
    LineData& operator [](int index) {
        assert(index < lineNum); return _ptInfo[index];
    };
    const LineData & operator [](int index) const {
        assert(index < lineNum); return _ptInfo[index];
    };
    int64_t timeStamp_tail;
    double timestamp;
    void clear() {
        for (auto &line: _ptInfo) {
            line.clear();
        }
    };
    void setToZero() {
        for (auto &line: _ptInfo) {
            line.setToZero();
        }
    };
    size_t getLineNum() const {
        return this->_ptInfo.size();
    };

// public:
private:
    std::vector<LineData>_ptInfo;
};

#endif // LIDARDATA_HPP
