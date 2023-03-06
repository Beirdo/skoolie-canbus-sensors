#include <Arduino.h>
#include <canbus_ids.h>
#include <canbus_dispatch.h>
#include <canbus.h>
#include <stdlib.h>

#include "project.h"
#include "wbus.h"
#include "sensor_registry.h"

void canbus_dispatch(int id, uint8_t *buf, int len, uint8_t type)
{
  
  if (type == CAN_DATA) {
    switch (id) {
      case CANBUS_ID_FSM_STATE:
      case CANBUS_ID_FSM_MODE:
      case CANBUS_ID_DEVICE_STATUS:
      case CANBUS_ID_COMBUSTION_FAN_PERCENT:
      case CANBUS_ID_GLOW_PLUG_PERCENT:
      case CANBUS_ID_BURN_POWER:
      case CANBUS_ID_CIRCULATION_PUMP_PERCENT:
      case CANBUS_ID_FLAME_DETECTOR_ENABLED:
      case CANBUS_ID_FUEL_PUMP_FREQUENCY:
      case CANBUS_ID_EXTERNAL_TEMP:
      case CANBUS_ID_IGNITION_SENSE:
      case CANBUS_ID_VEHICLE_FAN_PERCENT:
      case CANBUS_ID_BATTERY_VOLTAGE:
      case CANBUS_ID_FLAME_DETECTOR:
        {
          RemoteSensor *sensor = sensorRegistry.get<RemoteSensor>(id);
          if (sensor) {
            int32_t value = sensor->convert_from_packet(buf, len);
            sensor->set_value(value);
          }
        }
        break;

      default:
        break;
    }
  } else if (type == CANFD_WITH_BIT_RATE_SWITCH) {
    if (id == CANBUS_ID_MAINBOARD) {
      // KLine Response
      uint8_t *newbuf = new uint8_t[len];
      memcpy(newbuf, buf, len);
      wbus_queue_rx_packet(newbuf, len);
    }
  }
}