#ifndef __project_h_
#define __project_h_

#include <Arduino.h>
#include <Beirdo-Utilities.h>
#include <ds1820.h>

#define PIN_DQ                0   // PA_0
#define PIN_PCTLZ             1   // PA_1
#define PIN_USART2_TX         2   // PA_2
#define PIN_USART2_RX         3   // PA_3
#define PIN_SWDIO             10  // PA_13
#define PIN_SWDCLK            11  // PA_14
#define PIN_CAN_EN            12  // PB_1
#define PIN_BOOTROM_SEL       13  // PB_8
#define PIN_CAN_RX            16  // PA_11_R
#define PIN_USB_DN            16  // PA_11_R
#define PIN_CAN_TX            17  // PA_12_R
#define PIN_USB_DP            17  // PA_12_R

extern DS1820 ds1820;

#endif
