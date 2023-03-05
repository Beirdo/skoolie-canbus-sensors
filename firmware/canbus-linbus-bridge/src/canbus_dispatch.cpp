#include <Arduino.h>
#include <canbus_ids.h>
#include <canbus_dispatch.h>

#include "project.h"
#include "linbus_bridge.h"

void canbus_dispatch(int id, uint8_t *buf, int len, uint8_t type)
{
  switch (id) {
    case CANBUS_ID_LINBUS_BRIDGE:
      // We have no local data to send.
      break;

    case CANBUS_ID_LINBUS_BRIDGE | CANBUS_ID_WRITE_MODIFIER:
    default:
      if (len == 3) {
        bool enable = !(!buf[0]);
        int canbus_id = (((uint16_t)buf[1]) << 8) | buf[2];
        int index = linbusBridge.getMapIndex(canbus_id);
        linbusBridge.enable(index, enable);
      }
      break;
  }

  // These are REMOTE frames
  if (id >= CANBUS_ID_LINBUS_BASE && id <= (CANBUS_ID_LINBUS_BASE | CANBUS_ID_LINBUS_MASK)) {
    int index = linbusBridge.getMapIndex(id);
    if (index >= 0) {
      linbusBridge.update(index);
    }
  }

  // These are DATA frames with writes
  if (id >= (CANBUS_ID_LINBUS_BASE | CANBUS_ID_WRITE_MODIFIER) && id <= (CANBUS_ID_LINBUS_BASE | CANBUS_ID_LINBUS_MASK | CANBUS_ID_WRITE_MODIFIER)) {
    int index = linbusBridge.getMapIndex(id & ~CANBUS_ID_WRITE_MODIFIER);
    if (index >= 0 && len >= 1) {
      linbusBridge.write(index, buf, len); 
    }
  }
}