#include <Arduino.h>
#include <sensor.h>
#include <canbus_ids.h>

#include "sensor_registry.h"

void init_sensors(void)
{
  // On the mainboard
  sensorRegistry.add(CANBUS_ID_INTERNAL_TEMP, new RemoteCANBusSensor(CANBUS_ID_INTERNAL_TEMP, 2, 1));
  sensorRegistry.add(CANBUS_ID_FLAME_DETECTOR, new RemoteCANBusSensor(CANBUS_ID_FLAME_DETECTOR, 2, 100));
  sensorRegistry.add(CANBUS_ID_VSYS_VOLTAGE, new RemoteCANBusSensor(CANBUS_ID_VSYS_VOLTAGE, 2, 100));
  sensorRegistry.add(CANBUS_ID_FSM_STATE, new RemoteCANBusSensor(CANBUS_ID_FSM_STATE, 1, 1));
  sensorRegistry.add(CANBUS_ID_FSM_MODE, new RemoteCANBusSensor(CANBUS_ID_FSM_MODE, 1, 1));

  // Remote CANBus
  sensorRegistry.add(CANBUS_ID_EXTERNAL_TEMP, new RemoteCANBusSensor(CANBUS_ID_EXTERNAL_TEMP, 2, 100));
  sensorRegistry.add(CANBUS_ID_BATTERY_VOLTAGE, new RemoteCANBusSensor(CANBUS_ID_BATTERY_VOLTAGE, 2, 100));
  sensorRegistry.add(CANBUS_ID_COOLANT_TEMP_WEBASTO, new RemoteCANBusSensor(CANBUS_ID_COOLANT_TEMP_WEBASTO, 2, 100));
  sensorRegistry.add(CANBUS_ID_EXHAUST_TEMP, new RemoteCANBusSensor(CANBUS_ID_EXHAUST_TEMP, 2, 100));
  sensorRegistry.add(CANBUS_ID_IGNITION_SENSE, new RemoteCANBusSensor(CANBUS_ID_IGNITION_SENSE, 1, 1));
  sensorRegistry.add(CANBUS_ID_EMERGENCY_STOP, new RemoteCANBusSensor(CANBUS_ID_EMERGENCY_STOP, 1, 1));
  sensorRegistry.add(CANBUS_ID_START_RUN, new RemoteCANBusSensor(CANBUS_ID_START_RUN, 1, 1));

  // Remote LINBus
  sensorRegistry.add(CANBUS_ID_VEHICLE_FAN_PERCENT, new RemoteCANBusSensor(CANBUS_ID_VEHICLE_FAN_PERCENT, 1, 5));
  sensorRegistry.add(CANBUS_ID_VEHICLE_FAN_SPEED, new RemoteCANBusSensor(CANBUS_ID_VEHICLE_FAN_SPEED, 2, 50));
  sensorRegistry.add(CANBUS_ID_VEHICLE_FAN_INT_TEMP, new RemoteCANBusSensor(CANBUS_ID_VEHICLE_FAN_INT_TEMP, 1, 1));
  sensorRegistry.add(CANBUS_ID_VEHICLE_FAN_EXT_TEMP, new RemoteCANBusSensor(CANBUS_ID_VEHICLE_FAN_EXT_TEMP, 2, 100));
}

void update_sensors(void)
{

}


void RemoteCANBusSensor::do_feedback(void)
{
  switch (_id) {
    case CANBUS_ID_EXTERNAL_TEMP:
      break;
    case CANBUS_ID_BATTERY_VOLTAGE:
      break;
    case CANBUS_ID_COOLANT_TEMP_WEBASTO:
      break;
    case CANBUS_ID_EXHAUST_TEMP:
      break;
    case CANBUS_ID_IGNITION_SENSE:
      break;
    case CANBUS_ID_START_RUN:
      break;
    case CANBUS_ID_EMERGENCY_STOP:
      break;
    case CANBUS_ID_VEHICLE_FAN_SPEED:
      break;
    default:
      break;
   }
}
