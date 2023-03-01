#include <Arduino.h>
#include <ArduinoLog.h>
#include <sensor.h>
#include <Beirdo-Utilities.h>
#include <canbus_stm32.h>
#include <canbus_ids.h>
#include <canbus.h>

#include "project.h"
#include "linbus_bridge.h"

LINBusBridge linbusBridge(CANBUS_ID_LINBUS_BRIDGE, CANBUS_ID_LINBUS_BASE, CANBUS_ID_LINBUS_MASK);

CAN_filter_t filters[] = {
  {0, 0, 0, 0, 
   0xFF00 | STM32_CAN::makeFilter16(CANBUS_ID_COOLANT_TEMP_WEBASTO, STANDARD_FORMAT, REMOTE_FRAME),
   0xFF00 | STM32_CAN::makeFilter16(CANBUS_ID_COOLANT_TEMP_WEBASTO | CANBUS_ID_WRITE_MODIFIER, STANDARD_FORMAT, DATA_FRAME)},  
  {1, 0, 0, 1, 
   0xFF00 | STM32_CAN::makeFilter16(CANBUS_ID_BATTERY_VOLTAGE, STANDARD_FORMAT, REMOTE_FRAME),
   0xFF00 | STM32_CAN::makeFilter16(CANBUS_ID_BATTERY_VOLTAGE | CANBUS_ID_WRITE_MODIFIER, STANDARD_FORMAT, DATA_FRAME)},  
};
int filter_count = NELEMS(filters);

void setup(void)
{
  linbusBridge.begin();

  init_canbus_stm32_internal(PIN_CAN_EN, filters, filter_count);
}

void loop(void)
{
  int topOfLoop = millis();
  
  update_canbus_rx();
  update_canbus_tx();

  int duration = millis() - topOfLoop;
  if (duration >= 100) {
    Log.warning("Loop duration (%dms) > 100ms", duration);
  }

  int delay_ms = clamp<int>(100 - duration, 1, 100);

  delay(delay_ms);
}
