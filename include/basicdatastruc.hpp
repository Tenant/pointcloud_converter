#ifndef BASICDATASTRUC_HPP
#define BASICDATASTRUC_HPP
struct Point3d
{
    Point3d() {this->x = this->y = this->z = 0;}
    Point3d(double x, double y, double z) {this->x = x; this->y = y; this->z = z;}
    double x;
    double y;
    double z;
};

struct Point3i
{
    Point3i() {this->x = this->y = this->z = 0;}
    Point3i(int x, int y, int z) {this->x = x; this->y = y; this->z = z;}
    int x;
    int y;
    int z;
};

struct Point2d
{
    Point2d() {this->x = this->y = 0;}
    Point2d(double x, double y) {this->x = x; this->y = y; }
    double x;
    double y;
};

struct Point2i
{
    Point2i() {this->x = this->y = 0;}
    Point2i(int x, int y) {this->x = x; this->y = y;}
    int x;
    int y;
};
#endif // BASICDATASTRUC_HPP
