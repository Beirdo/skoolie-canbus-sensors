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

typedef struct {
  uint8_t canbus_id;
  uint8_t linbus_id;
  uint8_t register_index;
  uint8_t register_bytes;
  uint8_t type;
  uint8_t location;
  bool writeable;
  int last_update;
  bool active;
  int32_t last_value;
  int period;
} linbus_map_t;

class LINBusBridge {
  public:
    LINBusBridge(int period = 500) :
      _linbus(0), _map(0), _map_count(0), _period(period) {};
    void begin(HardwareSerial &serial);
    void update(int index);
    void callback(int index, int delay_ms);
    int getMapIndex(int canbus_id);
    void enable(int index, bool enable);
    void write(int index, uint8_t *buf, int len);

  protected:
    void probe(void);

    LINBus_stack *_linbus;
    linbus_slave_t _slaves[64];
    linbus_map_t *_map;
    int _map_count;
    int _period;
};

#endif