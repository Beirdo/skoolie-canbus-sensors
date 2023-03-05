#ifndef DISABLE_LOGGING
#define DISABLE_LOGGING
#endif

#include <Arduino.h>
#include <Beirdo-Utilities.h>
#include <sensor.h>
#include <canbus_stm32.h>
#include <canbus_ids.h>
#include <canbus.h>

#include "project.h"


CAN_filter_t filters[] = {
  {0, 0, 0, 0,
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_START_RUN, STANDARD_FORMAT, REMOTE_FRAME),
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_START_RUN | CANBUS_ID_WRITE_MODIFIER, STANDARD_FORMAT, DATA_FRAME)},
  {1, 0, 0, 1,
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_IGNITION_SENSE, STANDARD_FORMAT, REMOTE_FRAME),
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_IGNITION_SENSE | CANBUS_ID_WRITE_MODIFIER, STANDARD_FORMAT, DATA_FRAME)},
  {2, 0, 0, 1,
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_EMERGENCY_STOP, STANDARD_FORMAT, REMOTE_FRAME),
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_EMERGENCY_STOP | CANBUS_ID_WRITE_MODIFIER, STANDARD_FORMAT, DATA_FRAME)},
};
int filter_count = NELEMS(filters);

void setup(void)
{
  Serial.setRx(PIN_USART2_RX);
  Serial.setTx(PIN_USART2_TX);
  Serial.begin(12500);

  init_canbus_stm32_internal(PIN_CAN_EN, filters, filter_count);
}

void loop(void)
{
  int topOfLoop = millis();


  update_canbus_rx();
  update_canbus_tx();

  int duration = millis() - topOfLoop;
  int delay_ms = clamp<int>(100 - duration, 1, 100);

  delay(delay_ms);
}
