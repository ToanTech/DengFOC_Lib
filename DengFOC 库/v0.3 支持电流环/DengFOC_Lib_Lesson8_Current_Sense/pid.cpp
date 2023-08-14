#include "pid.h"
#include <Arduino.h>
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // PID控制器加速度限幅
    , limit(limit)         // PID控制器输出限幅
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros();
}

// PID 控制器函数
float PIDController::operator() (float error){
    // 计算两次循环中间的间隔时间
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
    // P环
    float proportional = P * error;
    // Tustin 散点积分（I环）
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    integral = _constrain(integral, -limit, limit);
    // D环（微分环节）
    float derivative = D*(error - error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);

    if(output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    // 保存值（为了下一次循环）
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
