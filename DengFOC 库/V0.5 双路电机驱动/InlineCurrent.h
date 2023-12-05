#include <Arduino.h>

class CurrSense
{
  public:
    CurrSense(int Mot_Num);
    float readADCVoltageInline(const int pinA);
    void configureADCInline(const int pinA,const int pinB, const int pinC);
    void calibrateOffsets();
    void init();
    void getPhaseCurrents();
    float current_a,current_b,current_c;
    int pinA;
    int pinB;
    int pinC;
    float offset_ia;
    float offset_ib;
    float offset_ic;
    float _shunt_resistor;
    float amp_gain;
    
    float volts_to_amps_ratio;
    
    float gain_a;
    float gain_b;
    float gain_c;
  private:
    int _Mot_Num;
};
