#include <Arduino.h>
#include <sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <canbus.h>

#include "ds1820.h"

void DS1820::init(void)
{
  _onewire = new OneWire(_pin_dq);
  _sensors = new DallasTemperature(_onewire, _pin_pctlz);

  _sensors->begin();
  _sensors->setResolution(_bits);

  int count = _sensors->getDeviceCount();
  Log.notice("Found %d DS1820 devices", count);

  bool displayed = false;
  while (!_sensors->getAddress(_address, 0)) {
    if (!displayed) {
      Log.error("Unable to get address for device 0");
      displayed = true;
    }
    delay(1000);
  }

  Log.notice("Found DS1820 device");
  _connected = true;
  _valid = true;
}

void DS1820::do_feedback(void)
{
  if (_value != UNUSED_VALUE) {
    canbus_output_value(_id, _value, 2);
  }
}

int32_t DS1820::get_raw_value(void)
{
  if (!_sensors->isConnected(_address)) {
    return UNUSED_VALUE;
  }

  _sensors->requestTemperaturesByAddress(_address);
  return _sensors->getTemp(_address);   // in 1/128 degC
}

int32_t DS1820::convert(int32_t reading)
{
  if (reading == UNUSED_VALUE) {
    return UNUSED_VALUE;
  }
  return (reading * 100) >> 7;
}
