#include <Arduino.h>
#include <ArduinoLog.h>

#include <sensor.h>
#include <Beirdo-Utilities.h>
#include <canbus_stm32.h>
#include <canbus_ids.h>
#include <canbus.h>

#include "project.h"
#include "ds1820.h"

DS1820 ds1820(CANBUS_ID_EXTERNAL_TEMP, PIN_DQ, PIN_PCTLZ);

CAN_filter_t filters[] = {
  {0, 0, 0, 0,
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_EXTERNAL_TEMP, STANDARD_FORMAT, REMOTE_FRAME),
   0xFFFF0000 | STM32_CAN::makeFilter16(CANBUS_ID_EXTERNAL_TEMP | CANBUS_ID_WRITE_MODIFIER, STANDARD_FORMAT, DATA_FRAME)},
};
int filter_count = NELEMS(filters);

void setup(void)
{
  Serial2.setRx(PIN_USART2_RX);
  Serial2.setTx(PIN_USART2_TX);
  Serial2.begin(115200);
  setup_logging(LOG_LEVEL_VERBOSE, &Serial2);

  ds1820.init();

  init_canbus_stm32_internal(PIN_CAN_EN, filters, filter_count);
}

void loop(void)
{
  int topOfLoop = millis();

  ds1820.update();

  update_canbus_rx();
  update_canbus_tx();

  int duration = millis() - topOfLoop;
  if (duration >= 100) {
    Log.warning("Loop duration (%dms) > 100ms", duration);
  }

  int delay_ms = clamp<int>(100 - duration, 1, 100);

  delay(delay_ms);
}
