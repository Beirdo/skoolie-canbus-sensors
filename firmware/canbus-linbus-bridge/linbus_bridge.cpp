#include "linbus_map.h"
#include <Arduino.h>
#include <LINBus_stack.h>
#include <canbus_ids.h>
#include <canbus.h>
#include <Beirdo-Utilities.h>
#include <stdlib.h>
#include <sensor.h>     // for UNUSED_VALUE

#include "project.h"
#include "linbus_bridge.h"
#include "global_timer.h"


// typedef struct {
//   uint8_t canbus_id;
//   uint8_t linbus_id;
//   uint8_t register_index;
//   uint8_t type;
//   uint8_t location;
//   bool writeable;
//   int last_update;
//   bool active;
//   int32_t last_value;
//   int period;
// } linbus_map_t;

const linbus_map_t default_map[] = {
  {CANBUS_ID_VEHICLE_FAN_PERCENT,   0x00, 2, 1, BOARD_TYPE_FAN_CONTROL, LOC_HEATER_CORE, true,  0, false, 0, 0},
  {CANBUS_ID_VEHICLE_FAN_SPEED,     0x00, 3, 2, BOARD_TYPE_FAN_CONTROL, LOC_HEATER_CORE, false, 0, false, 0, 0},
  {CANBUS_ID_VEHICLE_FAN_INT_TEMP,  0x00, 5, 1, BOARD_TYPE_FAN_CONTROL, LOC_HEATER_CORE, false, 0, false, 0, 0},
  {CANBUS_ID_VEHICLE_FAN_EXT_TEMP,  0x00, 6, 2, BOARD_TYPE_FAN_CONTROL, LOC_HEATER_CORE, false, 0, false, 0, 0},
  {CANBUS_ID_RADIATOR_FAN_PERCENT,  0x01, 2, 1, BOARD_TYPE_FAN_CONTROL, LOC_RADIATOR,    true,  0, false, 0, 0},
  {CANBUS_ID_RADIATOR_FAN_SPEED,    0x01, 3, 2, BOARD_TYPE_FAN_CONTROL, LOC_RADIATOR,    false, 0, false, 0, 0},
  {CANBUS_ID_RADIATOR_FAN_INT_TEMP, 0x01, 5, 1, BOARD_TYPE_FAN_CONTROL, LOC_RADIATOR,    false, 0, false, 0, 0},
  {CANBUS_ID_RADIATOR_FAN_EXT_TEMP, 0x01, 6, 2, BOARD_TYPE_FAN_CONTROL, LOC_RADIATOR,    false, 0, false, 0, 0},
  {CANBUS_ID_PELTIER_FAN_PERCENT,   0x02, 2, 1, BOARD_TYPE_FAN_CONTROL, LOC_PELTIER,     true,  0, false, 0, 0},
  {CANBUS_ID_PELTIER_FAN_SPEED,     0x02, 3, 2, BOARD_TYPE_FAN_CONTROL, LOC_PELTIER,     false, 0, false, 0, 0},
  {CANBUS_ID_PELTIER_FAN_INT_TEMP,  0x02, 5, 1, BOARD_TYPE_FAN_CONTROL, LOC_PELTIER,     false, 0, false, 0, 0},
  {CANBUS_ID_PELTIER_FAN_EXT_TEMP,  0x02, 6, 2, BOARD_TYPE_FAN_CONTROL, LOC_PELTIER,     false, 0, false, 0, 0},

  {CANBUS_ID_VALVE_BYPASS_ENGINE_CONTROL, 0x03, 3, 1, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_ENGINE_BYPASS, true, 0, false, 0, 0},
  {CANBUS_ID_VALVE_BYPASS_ENGINE_CURRENT, 0x03, 4, 2, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_ENGINE_BYPASS, false, 0, false, 0, 0},
  {CANBUS_ID_VALVE_HEAT_EXCHANGE_CONTROL, 0x04, 3, 1, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_HEAT_EXCHANGER, true, 0, false, 0, 0},
  {CANBUS_ID_VALVE_HEAT_EXCHANGE_CURRENT, 0x04, 4, 2, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_HEAT_EXCHANGER, false, 0, false, 0, 0},
  {CANBUS_ID_VALVE_BYPASS_HEATER_CONTROL, 0x05, 3, 1, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_HEATER_BYPASS, true, 0, false, 0, 0},
  {CANBUS_ID_VALVE_BYPASS_HEATER_CURRENT, 0x05, 4, 2, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_HEATER_BYPASS, false, 0, false, 0, 0},
  {CANBUS_ID_VALVE_BYPASS_RADIATOR_CONTROL, 0x06, 3, 1, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_RADIATOR_BYPASS, true, 0, false, 0, 0},
  {CANBUS_ID_VALVE_BYPASS_RADIATOR_CURRENT, 0x06, 4, 2, BOARD_TYPE_VALVE_CONTROL, LOC_VALVE_RADIATOR_BYPASS, false, 0, false, 0, 0},

  {CANBUS_ID_PELTIER_CONTROL, 0x07, 2, 1, BOARD_TYPE_PELTIER_CONTROL, LOC_PELTIER, true, 0, false, 0, 0},
  {CANBUS_ID_PELTIER_CURRENT, 0x07, 3, 2, BOARD_TYPE_PELTIER_CONTROL, LOC_PELTIER, false, 0, false, 0, 0},

  {CANBUS_ID_PUMP_HEAT_EXCHANGE_CONTROL, 0x08, 2, 1, BOARD_TYPE_PUMP_CONTROL, LOC_HEAT_EXCH_INT_INGRESS, true, 0, false, 0, 0},
  {CANBUS_ID_PUMP_HEAT_EXCHANGE_CURRENT, 0x08, 3, 2, BOARD_TYPE_PUMP_CONTROL, LOC_HEAT_EXCH_EXT_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_PUMP_EXTERNAL_LOOP_CONTROL, 0x09, 2, 1, BOARD_TYPE_PUMP_CONTROL, LOC_EXTERIOR_LOOP_INGRESS, true, 0, false, 0, 0},
  {CANBUS_ID_PUMP_EXTERNAL_LOOP_CURRENT, 0x09, 3, 2, BOARD_TYPE_PUMP_CONTROL, LOC_EXTERIOR_LOOP_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_PUMP_PELTIER_CONTROL, 0x0A, 2, 1, BOARD_TYPE_PUMP_CONTROL, LOC_PELTIER, true, 0, false, 0, 0},
  {CANBUS_ID_PUMP_PELTIER_CURRENT, 0x0A, 3, 2, BOARD_TYPE_PUMP_CONTROL, LOC_PELTIER, false, 0, false, 0, 0},

  {CANBUS_ID_FLOW_EXTERNAL_INGRESS, 0x0B, 2, 2, BOARD_TYPE_FLOW_SENSOR, LOC_EXTERIOR_LOOP_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_EXTERNAL_EGRESS, 0x0B, 4, 2, BOARD_TYPE_FLOW_SENSOR, LOC_EXTERIOR_LOOP_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_HEAT_EXCHANGE_COOLANT_INGRESS, 0x0C, 2, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEAT_EXCH_INT_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_HEAT_EXCHANGE_COOLANT_EGRESS, 0x0C, 4, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEAT_EXCH_INT_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_HEAT_EXCHANGE_EXTERNAL_INGRESS, 0x0D, 2, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEAT_EXCH_EXT_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_HEAT_EXCHANGE_EXTERNAL_EGRESS, 0x0D, 4, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEAT_EXCH_EXT_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_HEATER_CORE_INGRESS, 0x0E, 2, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEATER_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_HEATER_CORE_EGRESS, 0x0E, 4, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEATER_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_ENGINE_EGRESS, 0x0F, 2, 2, BOARD_TYPE_FLOW_SENSOR, LOC_ENGINE_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_WEBASTO_INGRESS, 0x0F, 4, 2, BOARD_TYPE_FLOW_SENSOR, LOC_WEBASTO_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_FLOW_PELTIER_INGRESS, 0x10, 2, 2, BOARD_TYPE_FLOW_SENSOR, LOC_HEATER_INGRESS, false, 0, false, 0, 0},

  {CANBUS_ID_COOLANT_TEMP_EXTERNAL_INGRESS, 0x11, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_EXTERIOR_LOOP_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_EXTERNAL_EGRESS, 0x11, 4, 2, BOARD_TYPE_COOLANT_TEMP, LOC_EXTERIOR_LOOP_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_HEAT_EXCHANGE_COOLANT_INGRESS, 0x12, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_HEAT_EXCH_INT_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_HEAT_EXCHANGE_COOLANT_EGRESS, 0x12, 4, 2, BOARD_TYPE_COOLANT_TEMP, LOC_HEAT_EXCH_INT_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_HEAT_EXCHANGE_EXTERNAL_INGRESS, 0x13, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_HEAT_EXCH_EXT_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_HEAT_EXCHANGE_EXTERNAL_EGRESS, 0x13, 4, 2, BOARD_TYPE_COOLANT_TEMP, LOC_HEAT_EXCH_EXT_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_HEATER_CORE_INGRESS, 0x14, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_HEATER_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_HEATER_CORE_EGRESS, 0x14, 4, 2, BOARD_TYPE_COOLANT_TEMP, LOC_HEATER_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_ENGINE_EGRESS, 0x15, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_ENGINE_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_WEBASTO_INGRESS, 0x15, 4, 2, BOARD_TYPE_COOLANT_TEMP, LOC_WEBASTO_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_PELTIER_INGRESS, 0x16, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_PELTIER_INGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_PELTIER_EGRESS, 0x16, 4, 2, BOARD_TYPE_COOLANT_TEMP, LOC_PELTIER_EGRESS, false, 0, false, 0, 0},
  {CANBUS_ID_COOLANT_TEMP_WATER_COOLER, 0x17, 2, 2, BOARD_TYPE_COOLANT_TEMP, LOC_WATER_COOLER, false, 0, false, 0, 0},
};
int default_map_count = NELEMS(default_map);


void linbus_callback(int index, int delay_ms);

void LINBusBridge::begin(HardwareSerial &serial)
{
  _linbus = new LINBus_stack(serial, 19200);

  setOpenDrainOutput(PIN_LIN_SLP, false, true);
  setOpenDrainOutput(PIN_LIN_WAKE, false, true);

  probe();

  if (_map) {
    delete [] _map;
  }

  _map = new linbus_map_t[default_map_count];
  memcpy(_map, (char *)default_map, sizeof(default_map));
  _map_count = default_map_count;

  for (int i = 0; i < _map_count; i++) {
    linbus_map_t *item = &_map[i];

    item->active = false;
    enable(i, _slaves[item->linbus_id].active);
  }

  _linbus->begin(PIN_LIN_WAKE, PIN_LIN_SLP);
}

void LINBusBridge::update(int index)
{
  if (index >= _map_count || !_map || !_map[index].active) {
    return;
  }

  linbus_map_t *item = &_map[index];

  int linbus_id = item->linbus_id;
  uint8_t register_index = item->register_index;
  uint8_t bytes = clamp<uint8_t>(item->register_bytes, 1, 4);

  bool cached = false;
  int now = millis();
  if (now - item->last_update < item->period * 2) {
    cached = true;
    canbus_output_value(item->canbus_id, item->last_value, bytes);
  }

  uint32_t data;
  uint8_t *buf = (uint8_t *)&data;

  _linbus->write(linbus_id, &register_index, 1);
  _linbus->writeRequest(linbus_id);
  _linbus->readStream(buf, bytes);

  item->last_update = now;
  _slaves[linbus_id].last_update = now;

  uint32_t value = __bswap32(data);
  item->last_value = (int32_t)value;

  if (!cached) {
    canbus_output_value(item->canbus_id, value, bytes);
  }
}

void LINBusBridge::probe(void)
{
  uint8_t  buf[3];
  int len;

  _linbus->busWakeUp();
  for (int i = 0; i < 64; i++) {
    buf[0] = 0;
    _linbus->write(i, buf, 1);
    _linbus->writeRequest(i);
    len = _linbus->readStream(buf, 3);
    if (len == 3 && _linbus->validateChecksum(buf, 3)) {
      _slaves[i].active = true;
      _slaves[i].last_update = 0;
      _slaves[i].type = buf[0];
      _slaves[i].location = buf[1];
      _slaves[i].linbus_id = i;
    } else {
      _slaves[i].active = false;
    }
  }
}

void LINBusBridge::callback(int index, int delay_ms)
{
  if (index >= _map_count || !_map || !_map[index].active) {
    return;
  }

  update(index);

  linbus_map_t *item = &_map[index];

  int slop = delay_ms - item->period;
  delay_ms = clamp<int>(item->period - slop, 1, item->period);
  globalTimer.register_timer(index, delay_ms, linbus_callback);
}

int LINBusBridge::getMapIndex(int canbus_id)
{
  if (!_map) {
    return -1;
  }

  for (int i = 0; i < _map_count; i++) {
    if (_map[i].canbus_id == canbus_id) {
      return i;
    }
  }

  return -1;
}

void LINBusBridge::write(int index, uint8_t *buf, int len)
{
  if (index >= _map_count || !_map || !_map[index].active || !_map[index].writeable) {
    return;
  }

  len = clamp<int>(len, 1, 2);
  _linbus->write(_map[index].linbus_id, buf, len);
}

void LINBusBridge::enable(int index, bool enable)
{
  if (index < 0) {
    return;
  }
  linbus_map_t *item = &_map[index];
  int linbus_id = item->linbus_id;

  if (!_slaves[linbus_id].active) {
    enable = false;
  }

  bool active = item->active;
  item->active = enable;

  if (!active && enable) {
    item->last_update = 0;
    item->last_value = UNUSED_VALUE;
    if (item->period <= 0)
    {
      item->period = _period;
    }
    globalTimer.register_timer(index, item->period, linbus_callback);
  }
}


void linbus_callback(int index, int delay_ms)
{
  linbusBridge.callback(index, delay_ms);
}
