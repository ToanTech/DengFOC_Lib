//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了
//函数声明

void setPwm(float Ua, float Ub, float Uc);
float setTorque(float Uq,float angle_el);
float _normalizeAngle(float angle);
void DFOC_Vbus(float power_supply);
void DFOC_alignSensor(int _PP,int _DIR);
float _electricalAngle();

float serial_motor_target();
String serialReceiveUserCommand();
//传感器读取
float DFOC_M0_Velocity();
float DFOC_M0_Angle();
//PID
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp);
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp);
float DFOC_M0_VEL_PID(float error);
float DFOC_M0_ANGLE_PID(float error);
//接口函数
void DFOC_M0_set_Velocity_Angle(float Target);
void DFOC_M0_setVelocity(float Target);
void DFOC_M0_set_Force_Angle(float Target);
void DFOC_M0_setTorque(float Target);
