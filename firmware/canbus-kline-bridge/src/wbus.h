#ifndef __wbus_h_
#define __wbus_h_

#include <Arduino.h>
#include "wbus_packet.h"

void init_wbus(void);
void send_break(void);
void wbus_send_response(wbusPacket_t *respPacket);
void wbus_send_next_packet(void);
void receive_wbus_from_serial(void);
void wbus_queue_rx_packet(uint8_t *buf, int len);
void process_wbus(void);

wbusPacket_t *wbus_rx_dispatch(wbusPacket_t *packet, uint8_t cmd);
uint8_t *allocate_response(uint8_t command, uint8_t len, uint8_t subcommand = 0);

uint8_t *wbus_command_diagnostics(void);
uint8_t *wbus_command_read_sensor(wbusPacket_t *packet);
uint8_t *wbus_command_read_voltage_data(uint8_t index);

uint8_t *wbus_read_status_sensor(void);
uint8_t *wbus_read_subsystem_enabled_sensor(void);
uint8_t *wbus_read_fuel_param_sensor(void);
uint8_t *wbus_read_operational_sensor(void);
uint8_t *wbus_read_state_sensor(void);
uint8_t *wbus_read_subsystem_status_sensor(void);
uint8_t *wbus_read_temperature_thresh_sensor(void);

#endif
