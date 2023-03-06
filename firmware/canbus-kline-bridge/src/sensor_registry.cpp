#include <Arduino.h>
#include <sensor.h>

#include "sensor_registry.h"

SensorRegistry sensorRegistry;

void SensorRegistry::add(int id, Sensor *sensor)
{
  sensor_reg_item_t *item = new sensor_reg_item_t;
  item->id = id;
  item->sensor = sensor;

  sensor_reg_item_t *curr, *prev = 0;
  for (curr = _head; curr && curr->id < id; prev = curr, curr = curr->next);

  item->next = curr;
  item->prev = prev;

  if (curr) {
    curr->prev = item;
  }

  if (prev) {
    prev->next = item;
  } else {
    _head = item;
  }
}

Sensor *SensorRegistry::get(int id) 
{
  sensor_reg_item_t *curr;

  for (curr = _head; curr && curr->id < id; curr = curr->next);

  if (!curr || curr->id != id) {
    return 0;
  }

  return curr->sensor;
}

void SensorRegistry::remove(int id)
{
  sensor_reg_item_t *curr, *prev = 0;

  for (curr = _head; curr && curr->id < id; prev = curr, curr = curr->next);

  if (!curr && curr->id != id) {
    return;
  }

  // OK, found it.
  if (prev) {
    prev->next = curr->next;
  } else {
    _head = curr->next;
  }

  if (curr->next) {
    curr->next->prev = prev;
  }

  delete curr;
}
