#ifndef __ds1820_h_
#define __ds1820_h_

#include <Arduino.h>
#include <sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class DS1820 : public LocalSensor {
  public:
    DS1820(int id, int pin_dq, int pin_pctlz) : 
      LocalSensor(id, 2, 100, 12),
      _pin_dq(pin_dq), _pin_pctlz(pin_pctlz) {};
    void init(void);
 
  protected:
    OneWire *_onewire;
    DallasTemperature *_sensors;
    DeviceAddress _address;

    int _pin_dq;
    int _pin_pctlz;

    void do_feedback(void);
    int32_t get_raw_value(void);
    int32_t convert(int32_t reading);
};

#endif