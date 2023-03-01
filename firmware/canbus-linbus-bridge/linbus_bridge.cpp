#include <Arduino.h>
#include <LINBus_stack.h>
#include <canbus_ids.h>
#include <canbus.h>
#include <Beirdo-Utilities.h>
#include <stdlib.h>

#include "project.h"
#include "linbus_bridge.h"
#include "global_timer.h"


// typedef struct {
//   uint8_t canbus_id;
//   uint8_t linbus_id;
//   uint8_t register_index;
//   uint8_t type;
//   uint8_t location;
//   int last_update;
//   bool active;
// } linbus_map_t;

const linbus_map_t default_map[] = {
  {CANBUS_ID_VEHICLE_FAN_SPEED, 0x00, 3, BOARD_TYPE_FAN_CONTROL, LOC_HEATER_CORE, 0, false},  
  {CANBUS_ID_VEHICLE_FAN_TEMP, 0x00, 6, BOARD_TYPE_FAN_CONTROL, LOC_HEATER_CORE, 0, false},  
};
int default_map_count = NELEMS(default_map);


void linbus_callback(int timer_id, int delay_ms);

void LINBusBridge::begin(void)
{
  _linbus = new LINBus_stack(Serial, 19200);
  
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
    if (_slaves[_map[i].linbus_id].active) {
      _map[i].active = 0;
      _map[i].last_update = 0;
      globalTimer.register_timer(i, 500, linbus_callback);
    }    
  }
}

void LINBusBridge::update(int index)
{
  if (index >= _map_count || !_map || !_map[index].active) {
    return;
  }

  int linbus_id = _map[index].linbus_id;
  uint8_t register_index = _map[index].register_index;

  uint16_t data;
  uint8_t *buf = (uint8_t *)&data;

  _linbus->write(linbus_id, &register_index, 1);
  _linbus->writeRequest(linbus_id);
  _linbus->readStream(buf, 2);

  int now = millis();
  _map[index].last_update = now;
  _slaves[linbus_id].last_update = now;

  int32_t value = (((int32_t)buf[0]) << 8) | buf[1];

  canbus_output_value(_map[index].canbus_id, value, 2);
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

  delay_ms = clamp<int>(1000 - delay_ms, 1, 500);
  globalTimer.register_timer(index, delay_ms, linbus_callback);

  update(index);
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
  if (index >= _map_count || !_map || !_map[index].active) {
    return;    
  }

  len = clamp<int>(len, 1, 2);
  _linbus->write(_map[index].linbus_id, buf, len);
}


void linbus_callback(int timer_id, int delay_ms)
{
  linbusBridge.callback(timer_id, delay_ms);
}
