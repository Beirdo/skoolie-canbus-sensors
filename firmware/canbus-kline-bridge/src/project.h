#ifndef __project_h_
#define __project_h_

#include <Arduino.h>
#include <Beirdo-Utilities.h>

// Serial2 -> Console
#ifdef PIN_SERIAL2_TX
#undef PIN_SERIAL2_TX
#endif

#ifdef PIN_SERIAL2_RX
#undef PIN_SERIAL2_RX
#endif


#define PIN_KLINE_EN          0   // PA_0
#define PIN_USART2_TX         2   // PA_2
#define PIN_USART2_RX         3   // PA_3
#define PIN_SWDIO             10  // PA_13
#define PIN_SWDCLK            11  // PA_14
#define PIN_CAN_EN            12  // PB_1
#define PIN_BOOTROM_SEL       13  // PB_8
#define PIN_CAN_RX            16  // PA_11_R
#define PIN_CAN_TX            17  // PA_12_R


#endif