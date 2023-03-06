#ifndef DISABLE_LOGGING
#define DISABLE_LOGGING
#endif

#include <Arduino.h>
#include <Beirdo-Utilities.h>
#include <SPI.h>
#include <sensor.h>
#include <canbus_mcp2517fd.h>
#include <canbus_ids.h>
#include <canbus.h>
#include <wbus.h>

#include "project.h"

void setup(void)
{
  init_wbus();
  init_canbus_mcp2517fd(&SPI, PIN_SPI_SS, PIN_CAN_INT);
}

void loop(void)
{
  int topOfLoop = millis();

  update_canbus_rx();
  update_canbus_tx();

  process_wbus();

  int duration = millis() - topOfLoop;
  int delay_ms = clamp<int>(100 - duration, 1, 100);

  delay(delay_ms);
}
