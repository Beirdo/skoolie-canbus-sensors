#include <Arduino.h>
#include <canbus_ids.h>
#include <canbus_dispatch.h>

#include "project.h"
#include "ds1820.h"

void canbus_dispatch(int id, uint8_t *buf, int len, uint8_t type)
{
  switch (id) {
    case CANBUS_ID_EXTERNAL_TEMP:
      ds1820.update(); 
      break;

    case CANBUS_ID_EXTERNAL_TEMP | CANBUS_ID_WRITE_MODIFIER:
    default:
      // This sensor does have any control knobs.  Ignore em.
      break;
  }
}