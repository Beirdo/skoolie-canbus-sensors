#include "HardwareSerial.h"
#ifndef __linbus_bridge_h_
#define __linbus_bridge_h_

#include <Arduino.h>
#include <LINBus_stack.h>
#include "linbus_map.h"

typedef struct {
  bool active;
  int last_update;
  uint8_t linbus_id;
  uint8_t type;
  uint8_t location;
} linbus_slave_t;

class LINBusBridge {
  public:
    LINBusBridge(int id, int base, int mask) : _id(id), _base(base), _mask(mask), _linbus(0), _map(0), _map_count(0) {};
    void begin(HardwareSerial &serial);
    void update(int index);
    void callback(int index, int delay_ms);
    int getMapIndex(int canbus_id);
    void write(int index, uint8_t *buf, int len);

  protected:
    void probe(void);

    int _id;
    int _base;
    int _mask;
    LINBus_stack *_linbus;
    linbus_slave_t _slaves[64];
    linbus_map_t *_map;
    int _map_count;
};

#endif