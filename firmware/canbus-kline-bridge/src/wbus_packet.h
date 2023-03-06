#ifndef __wbus_packet_h
#define __wbus_packet_h

#include <Arduino.h>

typedef struct {
  uint8_t *buf;
  int len;
} wbusPacket_t;


#endif
