#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <cppQueue.h>
#include <canbus_ids.h>
#include <canbus.h>
#include <sensor.h>

#include "project.h"
#include "wbus_packet.h"
#include "wbus.h"
#include "webasto.h"
#include "canbus.h"
#include "sensor_registry.h"

#define WBUS_RX_MATCH_ADDR 0xF4
#define WBUS_TX_ADDR       0x4F

#define WBUS_FIFO_SIZE   64
#define WBUS_BUFFER_SIZE 64

int tx_active;
int rx_active;

uint8_t wbus_rx_buffer[WBUS_BUFFER_SIZE];
int wbus_rx_tail;

cppQueue wbus_rx_q(sizeof(wbusPacket_t *), WBUS_FIFO_SIZE, FIFO);
cppQueue wbus_tx_q(sizeof(wbusPacket_t *), WBUS_FIFO_SIZE, FIFO);

uint8_t *wbus_canbus_send(wbusPacket_t *packet);

void init_wbus(void)
{
  Serial.setRx(PIN_USART2_RX);
  Serial.setTx(PIN_USART2_TX);
  Serial.begin(2400, SERIAL_8E1);
  Serial.flush();

  pinMode(PIN_KLINE_EN, OUTPUT);
  digitalWrite(PIN_KLINE_EN, LOW);
  tx_active = false;
  rx_active = false;
  wbus_rx_tail = 0;
}

void send_break(void)
{
  Serial.end();

  // Send a break - may not be needed as we are the slave, not the master.
  digitalWrite(PIN_USART2_TX, HIGH);
  pinMode(PIN_USART2_TX, OUTPUT);
  digitalWrite(PIN_USART2_TX, LOW);
  delay(50);
  digitalWrite(PIN_USART2_TX, HIGH);
  delay(50);
  pinMode(PIN_USART2_TX, INPUT);

  Serial.setRx(PIN_USART2_RX);
  Serial.setTx(PIN_USART2_TX);
  Serial.begin(2400, SERIAL_8E1);
  Serial.flush();
}

void wbus_send_response(wbusPacket_t *respPacket)
{
  if (!respPacket) {
    return;
  }

  wbus_tx_q.push((const wbusPacket_t *)&respPacket);

  if (!tx_active) {
    wbus_send_next_packet();
  }
}

void wbus_send_next_packet(void)
{
  if (tx_active) {
    return;
  }

  if (wbus_tx_q.isEmpty()) {
    return;
  }

  wbusPacket_t *packet;
  wbus_tx_q.pop(&packet);

  if (!packet) {
    return;
  }

  int len = packet->len;

  tx_active = true;
  digitalWrite(PIN_KLINE_EN, HIGH);
  // send_break();  ?? - I think as slave we don't need to.
  Serial.write(packet->buf, len);
  digitalWrite(PIN_KLINE_EN, LOW);
  tx_active = false;

  free(packet->buf);
  free(packet);
}

uint8_t calc_wbus_checksum(uint8_t *buf, int len)
{
  uint8_t checksum = 0x00;
  for (int i = 0; i < len; i++) {
    checksum ^= buf[i];
  }
  return checksum;
}

uint8_t *allocate_response(uint8_t command, uint8_t len, uint8_t subcommand)
{
  uint8_t *buf = (uint8_t *)malloc(len);
  buf[1] = len - 2;
  buf[2] = command ^ 0x80;
  if (subcommand) {
    buf[3] = subcommand;
  }
  buf[len - 1] = 0x00;

  return buf;
}

void receive_wbus_from_serial(void)
{
  if (1) { // Serial.getBreakReceived()) {
    rx_active = true;
    wbus_rx_tail = 0;
    Serial.flush();
  }

  while(Serial && Serial.available()) {
    if (!rx_active) {
      Serial.flush();
      continue;
    }

    uint8_t ch = Serial.read();
    if (wbus_rx_tail == 0 && ch != WBUS_RX_MATCH_ADDR) {
      rx_active = false;
      Serial.flush();
      continue;
    }

    wbus_rx_buffer[wbus_rx_tail++] = ch;
    if (wbus_rx_tail > 2) {
      int len = wbus_rx_buffer[1] + 2;
      if (wbus_rx_tail == len) {
        // Full frame received
        if (calc_wbus_checksum(wbus_rx_buffer, len)) {
          // Bad checksum.  Turf it.  Wait for new break
          rx_active = false;
          Serial.flush();
          continue;
        }

        uint8_t *buf = (uint8_t *)malloc(len);
        memcpy(buf, (const void *)wbus_rx_buffer, len);

        wbus_queue_rx_packet(buf, len);
        wbus_rx_tail = 0;
      }
    }
  }
}

void wbus_queue_rx_packet(uint8_t *buf, int len)
{
  wbusPacket_t *packet = (wbusPacket_t *)malloc(sizeof(wbusPacket_t));
  packet->buf = buf;
  packet->len = len;

  wbus_rx_q.push(&packet);
}

void process_wbus(void)
{
  while(!wbus_rx_q.isEmpty()) {
    wbusPacket_t *packet;
    wbus_rx_q.pop(&packet);
    if (!packet) {
      continue;
    }

    uint8_t cmd = packet->buf[2];
    wbusPacket_t *respPacket = wbus_rx_dispatch(packet, cmd);
    if (respPacket) {
      wbus_send_response(respPacket);
    }

    free(packet->buf);
    free(packet);
  }

  if (!tx_active) {
    wbus_send_next_packet();
  }
}

wbusPacket_t *wbus_rx_dispatch(wbusPacket_t *packet, uint8_t cmd)
{
  if (!packet) {
    return 0;
  }

  int len;
  wbusPacket_t *respPacket = 0;
  uint8_t *buf = 0;
  uint8_t minutes = 0;

  switch(cmd) {
    case 0x20:
      // Start for x minutes, default mode
    case 0x21:
      // Start for x minutes, parking heater on
    case 0x22:
      // Start for x minutes, ventilation on
    case 0x23:
      // Start for x minutes, supplemental heating on
    case 0x24:
      // Start for x minutes, circulation pump on
    case 0x25:
      // Start for x minutes, boost on
    case 0x26:
      // Start for x minutes, coolng on
    case 0x10:
      // Shutdown, no data
    case 0x44:
      // Seems to be a timer keepalive, I'm going to assume it adds the same number of minutes originally requested
      // and returns a 16-bit "minutes remaining" time
    case 0x48:
      // Component test
    case 0x51:
      // Read stuff
    case 0x56:
      // event log
    case 0x57:
      // CO2 calibration (umm, we have a sensor for that?!)
       buf = wbus_canbus_send(packet);
    default:
      break;

    case 0x38:
      // Diagnostic message, unknown use
      buf = wbus_command_diagnostics();
      break;

    case 0x50:
      // Read sensors
      buf = wbus_command_read_sensor(packet);
      break;

    case 0x53:
      // Read voltage data
      buf = wbus_command_read_voltage_data(packet->buf[3]);
      break;
  }

  if (buf) {
    len = buf[1] + 2;
    buf[0] = WBUS_TX_ADDR;
    if (!buf[2]) {
      buf[2] = cmd ^ 0x80;
    }
    buf[len - 1] = 0x00;
    buf[len - 1] = calc_wbus_checksum(buf, len);

    respPacket = (wbusPacket_t *)malloc(sizeof(wbusPacket_t));
    respPacket->buf = buf;
    respPacket->len = len;
  }

  return respPacket;
}

uint8_t *wbus_canbus_send(wbusPacket_t *packet)
{
  canbus_send(CANBUS_ID_MAINBOARD | CANBUS_ID_WRITE_MODIFIER, packet->buf, packet->len, CANFD_WITH_BIT_RATE_SWITCH);
  // our response will come back from CANBus and be queued for delivery.
  return 0;
}


uint8_t *wbus_command_diagnostics(void)
{
  static const uint8_t canned_reply[7] = {
    0x0B, 0x00, 0x00, 0x00, 0x00, 0x03, 0xDD
  };
  uint8_t *buf = allocate_response(0x30, 11);
  memcpy(&buf[3], canned_reply, 7);
  return buf;
}

// Some of these will be locally cached, some will not.
uint8_t *wbus_command_read_sensor(wbusPacket_t *packet)
{
  uint8_t sensornum = packet->buf[3];

  switch (sensornum) {
    case 0x02:
      // Status flags
      return wbus_read_status_sensor();
    case 0x03:
      // Subsystem on flags
      return wbus_read_subsystem_enabled_sensor();
    case 0x04:
      // Fuel parameters
      return wbus_read_fuel_param_sensor();
    case 0x05:
      // Operational measurements
      return wbus_read_operational_sensor();
    case 0x07:
      // Operating state
      return wbus_read_state_sensor();
    case 0x0F:
      // subsystem status
      return wbus_read_subsystem_status_sensor();
    case 0x11:
      // Temperature thresholds
      return wbus_read_temperature_thresh_sensor();

    case 0x06:
      // Operating times
    case 0x0A:
      // Burning duration
    case 0x0B:
      // Operating duration
    case 0x0C:
      // Start counters
    case 0x12:
      // Ventilation duration
      return wbus_canbus_send(packet);

    case 0x13:
      // Fuel prewarming status
      // This is only wired in on ThermoTop V.  I have a ThermoTop C.
    case 0x14:
      // Spark Transmission
      // This is only on gasoline models.  I have a diesel model.
    default:
      return 0;
  }
}

uint8_t *wbus_command_read_voltage_data(uint8_t index)
{
  if (index != 0x02) {
    return 0;
  }

  //   bytes:
  // 0: dont know
  // 1,2: Minimum Voltage threshold
  // 3,4,5,6: dont know
  // 7: Minimum voltage detection delay (delay)
  // 8,9: Maximum voltage threshold
  // 10,11,12,13: dont know
  // 14: Max voltage detection delay (seconds)

  static const uint8_t canned_reply[14] = {
    0x2C, 0x24, 0x25, 0x1C, 0x30, 0xD4, 0xFA, 0x40, 0x74, 0x00, 0x00, 0x63, 0x9C, 0x05
  };
  uint8_t *buf = allocate_response(0x53, 19, index);
  memcpy(&buf[4], canned_reply, 14);
  return buf;
}

uint8_t *wbus_read_status_sensor(void)
{
  uint8_t *buf = allocate_response(0x50, 9, 0x02);
  uint8_t flags = 0x00;

  Sensor *fsmMode = sensorRegistry.get(CANBUS_ID_FSM_MODE);
  int32_t fsm_mode = fsmMode->get_value();
  flags |= (fsm_mode == WEBASTO_MODE_SUPPLEMENTAL_HEATER ? 0x10 : 0x00);
  flags |= ((fsm_mode == WEBASTO_MODE_SUPPLEMENTAL_HEATER || fsm_mode == WEBASTO_MODE_PARKING_HEATER) ? 0x01 : 0x00);
  buf[4] = flags;

  flags = 0x00;
  
  Sensor *externalTempSensor = sensorRegistry.get(CANBUS_ID_EXTERNAL_TEMP);
  flags |= (externalTempSensor->get_value() >= 1000 ? 0x01 : 0x00);    // >= 10C - summer.  Below - winter
  buf[5] = flags;
  buf[6] = 0x00;      // Generator signal D+  (whaaa?)
  buf[7] = 0x00;      // boost mode, auxiliary drive

  flags = 0x00;

  Sensor *ignitionSensor = sensorRegistry.get(CANBUS_ID_IGNITION_SENSE);
  int32_t ignitionOn = ignitionSensor->get_value();
  flags |= (ignitionOn ? 0x01 : 0x00);
  buf[8] = flags;
  return buf;
}

uint8_t *wbus_read_subsystem_enabled_sensor(void)
{
  uint8_t *buf = allocate_response(0x50, 5, 0x03);
  uint8_t flags = 0x00;

  Sensor *combustionFan = sensorRegistry.get(CANBUS_ID_COMBUSTION_FAN_PERCENT);
  flags |= (combustionFan->get_value() ? 0x01 : 0x00);

  Sensor *glowPlugOn = sensorRegistry.get(CANBUS_ID_GLOW_PLUG_PERCENT);
  flags |= (glowPlugOn->get_value() ? 0x02 : 0x00);

  Sensor *burnPower = sensorRegistry.get(CANBUS_ID_BURN_POWER);
  flags |= (burnPower->get_value() ? 0x04 : 0x00);

  Sensor *circulationPump = sensorRegistry.get(CANBUS_ID_CIRCULATION_PUMP_PERCENT);
  flags |= (circulationPump->get_value() ? 0x08 : 0x00);

  Sensor *vehicleFanPercent = sensorRegistry.get(CANBUS_ID_VEHICLE_FAN_PERCENT);
  flags |= (vehicleFanPercent->get_value() ? 0x10 : 0x00);

  flags |= 0x00;        // Nozzle stock heating... we don't have that??!

  Sensor *flameSensor = sensorRegistry.get(CANBUS_ID_FLAME_DETECTOR_ENABLED);
  flags |= (flameSensor->get_value() ? 0x40 : 0x00);
  buf[4] = flags;
  return buf;
}

uint8_t *wbus_read_fuel_param_sensor(void)
{
  uint8_t *buf = allocate_response(0x50, 7, 0x04);
  buf[4] = 0x1D;      // from libwbus example, change to what it reads with OEM controller
  buf[5] = 0x3C;      // from libwbus example, change to what it reads with OEM controller
  buf[6] = 0x3C;      // from libwbus example, change to what it reads with OEM controller
  return buf;
}

uint8_t *wbus_read_operational_sensor(void)
{
  uint8_t *buf = allocate_response(0x50, 12, 0x05);

  Sensor *externalTempSensor = sensorRegistry.get(CANBUS_ID_EXTERNAL_TEMP);
  buf[4] = (uint8_t)(((externalTempSensor->get_value() / 50) + 1 / 2) + 50);

  Sensor *batteryVoltageSensor = sensorRegistry.get(CANBUS_ID_BATTERY_VOLTAGE);
  int vbat = batteryVoltageSensor->get_value();
  buf[5] = HI_BYTE(vbat);
  buf[6] = LO_BYTE(vbat);

  Sensor *flameSensor = sensorRegistry.get(CANBUS_ID_FLAME_DETECTOR_ENABLED);
  buf[7] = (flameSensor->get_value() ? 0x01 : 0x00);

  Sensor *burnPower = sensorRegistry.get(CANBUS_ID_BURN_POWER);
  int power = burnPower->get_value();
  buf[8] = HI_BYTE(power);
  buf[9] = LO_BYTE(power);

  Sensor *flameDetectorSensor = sensorRegistry.get(CANBUS_ID_FLAME_DETECTOR);
  int milliohms = flameDetectorSensor->get_value();
  buf[10] = HI_BYTE(milliohms);
  buf[11] = LO_BYTE(milliohms);

  return buf;
}

uint8_t *wbus_read_state_sensor(void)
{
  uint8_t *buf = allocate_response(0x50, 10, 0x07);

  Sensor *fsmState = sensorRegistry.get(CANBUS_ID_FSM_STATE);
  buf[4] = (uint8_t)fsmState->get_value();
  
  buf[5] = 0x00;                // Operating state state number ???

  Sensor *deviceStatus = sensorRegistry.get(CANBUS_ID_DEVICE_STATUS);
  buf[6] = (uint8_t)deviceStatus->get_value(); // Device state bitfield,  0x01 = STFL, 0x02 = UEHFL, 0x04 = SAFL, 0x08 = RZFL
  
  buf[7] = 0x00;                // unknown
  buf[8] = 0x00;                // unknown
  buf[9] = 0x00;                // unknown
  return buf;
}

uint8_t *wbus_read_subsystem_status_sensor(void)
{
  uint8_t *buf = allocate_response(0x50, 9, 0x0F);

  Sensor *glowPlug = sensorRegistry.get(CANBUS_ID_GLOW_PLUG_PERCENT);
  buf[4] = (uint8_t)glowPlug->get_value();

  Sensor *fuelPump = sensorRegistry.get(CANBUS_ID_FUEL_PUMP_FREQUENCY);
  buf[5] = (uint8_t)fuelPump->get_value();

  Sensor *combustionFan = sensorRegistry.get(CANBUS_ID_COMBUSTION_FAN_PERCENT);
  buf[6] = (uint8_t)combustionFan->get_value();

  buf[7] = 0x00;    // Unknown
  
  Sensor *circulationPump = sensorRegistry.get(CANBUS_ID_CIRCULATION_PUMP_PERCENT);
  buf[8] = (uint8_t)circulationPump->get_value();
  return buf;
}

uint8_t *wbus_read_temperature_thresh_sensor(void)
{
  // lower and upper temperature thresholds (degC + 50)
  // TODO: read from current firmware in unit
  uint8_t *buf = allocate_response(0x50, 6, 0x11);
  buf[4] = 40;    // -10C
  buf[5] = 70;    // 20C
  return buf;
}
