//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了


#include "Wire.h" 
#include "AS5600.h"
#include <Arduino.h> 

int _raw_ang_hi = 0x0c;
int _raw_ang_lo = 0x0d;
int _ams5600_Address = 0x36;
int ledtime = 0;
int32_t full_rotations=0; // full rotation tracking;
float angle_prev=0; 

void BeginSensor() {
  Wire.begin(19,18, 400000UL);
  delay(1000);
}
//readTwoBytes(int in_adr_hi, int in_adr_lo)这段代码是一个函数，其目的是从I2C设备（在代码中的变量名为_ams5600_Address）中读取两个字节数据，并将其合并成一个16位的无符号整数返回。
//具体来说，函数接受两个整型参数in_adr_hi和in_adr_lo，它们用于指定需要读取的两个字节数据的地址。函数中首先通过Wire库开始I2C传输，向设备写入in_adr_lo和in_adr_hi分别作为数据地址，然后读取相应的字节数据。
//在每个Wire.requestFrom()调用之后，通过一个while循环等待数据接收完毕。然后读取接收到的低字节和高字节，并使用位运算将它们合并成一个16位的无符号整数。
//最后，返回合并后的整数。如果读取过程中出现错误或者函数没有成功读取到数据，则函数返回-1。
word readTwoBytes(int in_adr_hi, int in_adr_lo)
{
  word retVal = -1;
 
  /* 读低位 */
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr_lo);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
  while(Wire.available() == 0);
  int low = Wire.read();
 
  /* 读高位 */  
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr_hi);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
  while(Wire.available() == 0);
  int high = Wire.read();
  
  retVal = (high << 8) | low;
  
  return retVal;
}

word getRawAngle()
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

float getAngle_Without_track()
{
  return getRawAngle()*0.08789* PI / 180;    //得到弧度制的角度
}

float getAngle()
{
    float val = getAngle_Without_track();
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(abs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
    
}
