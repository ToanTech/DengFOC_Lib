#include <Arduino.h> 
#include "Wire.h"

class Sensor_AS5600
{
  public:
    Sensor_AS5600(int Mot_Num);
    void Sensor_init(TwoWire* _wire = &Wire);
    void Sensor_update();
    float getAngle();
    float getVelocity();
    float getMechanicalAngle();
    double getSensorAngle();
  private:
    int _Mot_Num;
    //AS5600 变量定义
    //int sensor_direction=1;       //编码器旋转方向定义
    float angle_prev=0; // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    long angle_prev_ts=0; // 上次调用 getAngle 的时间戳
    float vel_angle_prev=0; // 最后一次调用 getVelocity 时的角度
    long vel_angle_prev_ts=0; // 最后速度计算时间戳
    int32_t full_rotations=0; // 总圈数计数
    int32_t vel_full_rotations=0; //用于速度计算的先前完整旋转圈数
TwoWire* wire;
};
