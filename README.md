# DengFOC 极简FOC库 -- 开源

# 需要注意，当前库尚未发布，将随着B站《灯哥手把手教你写FOC算法》系列教学进程发布，在第七、八章将发布该库基础版

课程教案和将来的库文档发布：dengfoc.com

## 1 概述

DengFOC 库是灯哥开源的，基于 Arduino 的开源 FOC 库。

本FOC库参考了 SimpleFOC 的部分设计，但更秉承极简主义和实用主意，拥有比SimpleFOC更简易但更丰富的二次开发和应用特性：

1. **资源占用低**：内存占用比SImpleFOC少2/3

2. **开放度更广**：库把基本的FOC算法接口（如电角度、Iaplha 、IBeta）等以一种及其简单的方式向用户开放，使得用户可以直接接触到 FOC 算法实现的全过程，易于学习和在此基础上更进一步的开发 FOC 算法。
3. **外接能力强**：支持与OpenMV、树莓派、Python等多种硬件方案和软件语言直接的对接和互相调用，可以以比SimpleFOC更快的方式完成无刷电机应用的开发。
4. **即插即用，无需校准**：先进的参数自识别功能可以使得用户无需配置任何参数，直接插入电机和编码器即可跑FOC
5. **无线控制支持**：高速UDP，ESPNow 通讯，无需信号线即可控制电机
6. **脚本支持**：库内建强大的脚本语言 Lua，可以在不编译的情况下快速建立FOC应用
7. **强大的工具链支持**：支持与 Matlab 、Simulink 、ROS 等系统直接通讯，秒速建立机器人应用
8. **高精度减速器应用支持**：支持双编码器减速机应用

本 FOC 库与 DengFOC 硬件联合组成一整套完整可用的 FOC 电机驱动方案。

## 2 怎样写 DengFOC 代码

灯哥在设计DengFOC库的时候，秉承极简和低内存占用的设计思路。因此，所有的功能块都是走**即用即引入，不用不引入**的原则，以此获得极低的资源占用率。

目前库已经有的模块有：

1. DengFOC 主库 -- 包含DengFOC基础运行函数
2. AS5600 编码器库  -- 传感器库
3. InlineCurrent 库 -- INA240 电流传感器库

目前已实现的功能：

1. 双路开环速度
2. 双路AS5600传感器读取
3. 双路在线电流检测
4. 双路力位闭环（基于电压Uq）
5. 双路力位闭环（基于电流Iq）

下面用两个力+位置闭环代码（力位闭环，力作为内环，位置作为外环），来示范 DengFOC 库的代码编写。

### 2.1 双电机力位闭环（基于Voltage）

```
#include "DengFOC.h"
#include "AS5600.h"

//初始化M0、M1电机
BLDC M0=BLDC(0);
BLDC M1=BLDC(1);
//初始化M0、M1电机

//初始化M0、M1编码器
Sensor S0=Sensor(0);
TwoWire S0_I2C = TwoWire(0);

Sensor S1=Sensor(1);
TwoWire S1_I2C = TwoWire(1);
//初始化M0、M1编码器

void setup()
{
  Serial.begin(115200);
  M0.voltage_power_supply=16.8;
  M1.voltage_power_supply=16.8;

  delay(500);
  S0_I2C.begin(19,18, 400000UL);
  S1_I2C.begin(23,5, 400000UL);
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
  S1.Sensor_init(&S1_I2C);   //初始化编码器1
  Serial.println("AS5600 好");
  delay(500);
  
  }

// 电角度求解
float S0_electricalAngle(int sensor_direction,int PP_value){
  return  _normalizeAngle((float)(sensor_direction * PP_value) * S0.getMechanicalAngle());
}

float S1_electricalAngle(int sensor_direction,int PP_value){
  return  _normalizeAngle((float)(sensor_direction * PP_value) * S1.getMechanicalAngle());
}


void loop() {
  S0.Sensor_update();
  S1.Sensor_update();
  
  //FOC 实现
  M0.setPhaseVoltage(-5*(0-S0.getAngle()),  0,S0_electricalAngle(-1,14));   //力位控制角度环
  M1.setPhaseVoltage(-5*(0-S1.getAngle()),  0,S1_electricalAngle(1,14));   //力位控制角度环
  //FOC 实现
  
}

```

### 2.2 双电机力位闭环（基于电流环）

```
#include "DengFOC.h"
#include "AS5600.h"
#include "InlineCurrent.h"

//初始化M0、M1电流传感器
CurrSense CS_M0= CurrSense(0);
CurrSense CS_M1= CurrSense(1);
//初始化M0、M1电流传感器

BLDC M0=BLDC(0);
BLDC M1=BLDC(1);

Sensor S0=Sensor(0);
TwoWire S0_I2C = TwoWire(0);

Sensor S1=Sensor(1);
TwoWire S1_I2C = TwoWire(1);

void setup()
{
  Serial.begin(115200);
  M0.voltage_power_supply=16.8;
  M1.voltage_power_supply=16.8;

  delay(500);
  S0_I2C.begin(19,18, 400000UL);
  S1_I2C.begin(23,5, 400000UL);
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
  S1.Sensor_init(&S1_I2C);   //初始化编码器1

  CS_M0.init();
  CS_M1.init();
  
  Serial.println("AS5600 好");
  delay(500);
  
  }

// 电角度求解
float S0_electricalAngle(int sensor_direction,int PP_value){
  return  _normalizeAngle((float)(sensor_direction * PP_value) * S0.getMechanicalAngle());
}

float S1_electricalAngle(int sensor_direction,int PP_value){
  return  _normalizeAngle((float)(sensor_direction * PP_value) * S1.getMechanicalAngle());
}


float I_alpha,I_beta;
float I_d,I_q;
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f


void loop() {
  S0.Sensor_update();
  S1.Sensor_update();
  CS_M0.getPhaseCurrents();
  CS_M1.getPhaseCurrents();

  //计算 Iq、Id
  I_alpha=CS_M1.current_a;
  I_beta = _1_SQRT3 * CS_M1.current_a + _2_SQRT3 * CS_M1.current_b;

  float ct = cos(S1_electricalAngle(1,14));
  float st = sin(S1_electricalAngle(1,14));
  I_d = I_alpha * ct + I_beta * st;
  I_q = I_beta * ct - I_alpha * st;
  //计算 Iq、Id
  
  //FOC实现
  M0.setPhaseVoltage(-5*(0-S0.getAngle()),  0,S0_electricalAngle(-1,14));   //力位控制角度环（基于Uq）
  M1.setPhaseVoltage(1.2*(-1.5*(0-S1.getAngle())-I_q),  0,S1_electricalAngle(1,14));   //力位控制角度环（基于电流检测）
  //FOC实现
}

```

