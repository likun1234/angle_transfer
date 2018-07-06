#ifndef EULER_H
#define EULER_H
#include <iostream>
#include <cmath>
const double pi=3.1415926;
using namespace std;
enum RotSeq{zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy,xzx};
struct Quaternion{
  Quaternion():x(0), y(0), z(0), w(1){};
  Quaternion(double x, double y, double z, double w):x(x), y(y), z(z), w(w){};
 
  void normalize(){
    double norm = std::sqrt(x*x + y*y + z*z + w*w);
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  }
    double norm(){
        return std::sqrt(x*x + y*y + z*z + w*w);
    }
    
    double x;
    double y;
    double z;
    double w;
};


void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]);    
void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]);
void quaternion2Euler(const Quaternion& q, double res[], RotSeq rotSeq);
Quaternion operator*(Quaternion& q1, Quaternion& q2);
ostream& operator <<(std::ostream& stream, const Quaternion& q); 
double rad2deg(double rad);
#endif
