#include <Arduino.h>
#include <EEPROM.h>
#include "haversine.h"
#include "CRC.h"
#include "vehicle.h"


//#define DebugMavLink
//#define DebugTouch
//#define DebugEEPROM
//#define DebugOLED

#include <U8g2lib.h>
#include <SPI.h>

const uint8_t PinCS    = 9;
const uint8_t PinDC    = 8;
const uint8_t PinRES   = 7;
const uint8_t PinSCK   = 14;

const uint8_t PinTouch = 1;
const uint8_t PinLed   = 13;

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, PinCS, PinDC, PinRES);

const uint32_t DisplayRefreshCycle_ms = 200;

#include "drawGeoQr.h"
#include "texts.h"

const uint8_t MessageCountMonitored = 6;
uint32_t timestampMessageReceived[MessageCountMonitored];

const uint8_t MessageIndexFromValue[LabelRowCount][LabelColumnCount] =
{
  { 0xF3, 0xFF }, // POS
  { 0xF3, 0xF3 }, // ALT RAL
  { 0xF5, 0xF5 }, // ASP GSP
  { 0xF5, 0xF5 }, // HDG CLB
  { 0xF5, 0xF0 }, // THR MOD
  { 0xF1, 0xF1 }, // VOL CUR
  { 0xF2, 0xF2 }, // SAT HDO
  { 0xF2, 0x43 }  // FIX HOM
};
// packed format, each nibble represents a message index

const uint32_t MessageTimeout_ms[MessageCountMonitored] = 
{
  10000,
  5000,
  5000,
  5000,
  0xFFFFFFFF, // needs to be received only once
  5000
};

bool getValueTimeout(uint8_t row, uint8_t column, bool enablePowerOnTimeout = false)
{
  uint8_t msg1, msg2;
  uint32_t now;
  msg1 = msg2 = MessageIndexFromValue[row][column];
  msg1 &= 0x0F;
  now = millis();
  if(msg1 < MessageCountMonitored)
  {
    if(timestampMessageReceived[msg1] > 0L)
    {
      if((now - timestampMessageReceived[msg1]) >= MessageTimeout_ms[msg1]) return true;
    }
    else
    {
      if(enablePowerOnTimeout) return true;
    }
  }
  msg2 >>= 4;
  if(msg2 < MessageCountMonitored)
  {
    if(timestampMessageReceived[msg2] > 0L)
    {
      if((now - timestampMessageReceived[msg2]) >= MessageTimeout_ms[msg2]) return true;
    }
    else
    {
      if(enablePowerOnTimeout) return true;
    }
  }
  return false;
}

const uint32_t TimeoutBlinkPeriod_ms = 800;

void drawBox(uint8_t row, uint8_t column, const char * text, bool invert)
{
  u8g2.setDrawColor(invert ? 0 : 1);
  u8g2.drawBox(column * 64, row * 8, 14, 7);
  u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
  u8g2.setDrawColor(invert ? 1 : 0);
  u8g2.drawStr(column * 64 + 1, row * 8 + 6, text);
}

void drawBoxes()
{
  uint32_t now = millis() % TimeoutBlinkPeriod_ms;
  uint8_t row, column;
  for(row = 0; row < LabelRowCount; row++)
  {
    for(column = 0; column < LabelColumnCount; column++)
    {
      if(boxLabel[row][column] != NULL)
      {
        drawBox(row, column, boxLabel[row][column], getValueTimeout(row, column, true) && (now >= TimeoutBlinkPeriod_ms / 2));
      }
    }
  }
}

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "mavlink/ardupilotmega/mavlink.h"

struct 
{
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int32_t relative_alt;
  float airspeed;
  float groundspeed;
  int16_t heading;
  float climb;
  uint16_t throttle;
  uint16_t voltage;
  int16_t current;
  uint8_t satellites_visible;
  uint16_t hdop;
  uint8_t fix_type;
  uint32_t custom_mode;
  uint8_t type;
  int32_t home_lat;
  int32_t home_lon;
  uint32_t crc32;
} d;

void updateDataCrc()
{
  d.crc32 = crc32((uint8_t *)&d, sizeof(d) - 4);
}

bool crcValid()
{
  return d.crc32 == crc32((uint8_t *)&d, sizeof(d) - 4);
}

void drawValues()
{
  char s[32];
  uint8_t a, b;

  u8g2.setDrawColor(1);

  // POS
  u8g2.setFont(u8g2_font_tom_thumb_4x6_mf);
  snprintf(s, sizeof(s), "% 3li.%07lu   % 4li.%07lu", d.lat / 10000000L, abs(d.lat) % 10000000L, d.lon / 10000000L, abs(d.lon) % 10000000L);
  u8g2.drawStr(17, 6, s);
  
  u8g2.setFont(u8g2_font_profont10_mf); // 5x7

  // ALT
  snprintf(s, sizeof(s), "% 6lim", d.alt / 1000);
  u8g2.drawStr(15, 15, s);

  // RAL
  snprintf(s, sizeof(s), "% 6lim", d.relative_alt / 1000);
  u8g2.drawStr(79, 15, s);

  // ASP
  snprintf(s, sizeof(s), "% 5ikm/h", (int16_t)d.airspeed);
  u8g2.drawStr(15, 23, s);

  // GSP
  snprintf(s, sizeof(s), "% 5ikm/h", (int16_t)d.groundspeed);
  u8g2.drawStr(79, 23, s);

  // HDG
  snprintf(s, sizeof(s), "% 6i°", d.heading);
  u8g2.drawUTF8(15, 31, s);

  // CLB
  snprintf(s, sizeof(s), "% 4i.%01um/s", (int16_t)d.climb, abs( (int16_t)(10.0 * d.climb) ) % 10);
  u8g2.drawStr(79, 31, s);

  // THR
  snprintf(s, sizeof(s), "%6u", d.throttle);
  u8g2.drawStr(15, 39, s);

  // MOD
  s[0] = 0;
  if( (d.type == 1) || (d.type == 16) || ((d.type >= 19) && (d.type <= 25)) )
  {
    // plane
    if(d.custom_mode < PlaneModeTextCount)
    {
      snprintf(s, sizeof(s), "%s", PlaneModeText[d.custom_mode]);
    }
  }
  else if( ((d.type >= 2) && (d.type <= 4)) || ((d.type >= 13) && (d.type <= 15)) )
  {
    // copter
    if(d.custom_mode < CopterModeTextCount)
    {
      snprintf(s, sizeof(s), "%s", CopterModeText[d.custom_mode]);
    }
  }
  else if( (d.type == 10) || (d.type == 11) )
  {
    // rover
    if(d.custom_mode < RoverModeTextCount)
    {
      snprintf(s, sizeof(s), "%s", RoverModeText[d.custom_mode]);
    }
  }
  a = u8g2.getStrWidth(s);
  if(a > 30)
    b = 79;
  else
    b = 79 + 30 - a;
  u8g2.drawStr(b, 39, s);

  // VOL
  snprintf(s, sizeof(s), "%4u.%uV", d.voltage / 1000, (d.voltage / 100) % 10);
  u8g2.drawStr(15, 47, s);

  // CUR
  snprintf(s, sizeof(s), "% 4i.%01uA", d.current / 100, (abs(d.current) % 100) / 10 );
  u8g2.drawStr(79, 47, s);

  // SAT
  snprintf(s, sizeof(s), "%6u", d.satellites_visible);
  u8g2.drawStr(15, 55, s);

  // HDO
  snprintf(s, sizeof(s), "%3u.%02u", d.hdop / 100, d.hdop % 100);
  u8g2.drawStr(79, 55, s);

  // FIX
  snprintf(s, sizeof(s), "%s", GpsFixText[d.fix_type]);
  a = u8g2.getStrWidth(s);
  if(a > 30)
    b = 15;
  else
    b = 15 + 30 - a;
  u8g2.drawStr(b, 63, s);

  // HOM
  {
    float distance_km;
    uint32_t distance_m;
    distance_km = haversine_km((double)d.lat * 10e-7, (double)d.lon * 10e-7, (double)d.home_lat * 10e-7, (double)d.home_lon * 10e-7);
    distance_m = (uint32_t)(distance_km * 1000.0);
    snprintf(s, sizeof(s), "%6lum", distance_m);
    u8g2.drawStr(79, 63, s);
  }
}

void saveDataOnTimeout()
{ // check for timeout of position message and save position and all other data into EEPROM if timeout was found
  static bool positionTimeout = false;;
  bool t;
  
  t = getValueTimeout(0, 0);
  if(t)
  {
    if(!positionTimeout)
    {
      uint8_t i, *p;
      
#ifdef DebugEEPROM
      digitalWrite(PinLed, HIGH);
#endif
      updateDataCrc();
#ifdef DebugEEPROM
      Serial.println("saving EEPROM data...");
      Serial.print("CRC = ");
      Serial.println(d.crc32, HEX);
#endif
      
      // save data into EEPROM
      for(i = 0, p = (uint8_t *)&d; i < sizeof(d); i++, p++)
      {
        EEPROM[i] = *p;
      }
#ifdef DebugEEPROM
      digitalWrite(PinLed, LOW);
#endif
    }
  }
  positionTimeout = t;
}

void setup()
{
  uint8_t i;
  uint8_t *p;
  
  pinMode(PinLed, OUTPUT);
  
  Serial.begin(115200); // debug via USB
  Serial1.begin(57600); // MavLink
  
  Serial.println("MavLinkView is starting...");

  SPI.setSCK(PinSCK); // use alternative pin configuration to spare LED pin
  u8g2.begin();
  
  // load EEPROM data
#ifdef DebugEEPROM
  digitalWrite(PinLed, HIGH);
  Serial.println("loading EEPROM data...");
#endif
  for(i = 0, p = (uint8_t *)&d; i < sizeof(d); i++, p++)
  {
    *p = EEPROM[i];
  }
#ifdef DebugEEPROM
  Serial.print("CRC = ");
  Serial.println(d.crc32, HEX);
#endif
  if(!crcValid())
  {
    Serial.println("EEPROM CRC not valid!");
    memset(&d, 0, sizeof(d));
  }
  memset(timestampMessageReceived, 0, sizeof(timestampMessageReceived));
#ifdef DebugEEPROM
  digitalWrite(PinLed, LOW);
#endif
  
  // test data
#if 0
  d.lat = -127654321;
  d.lon = -1237654321;
  d.alt = -123000;
  d.relative_alt = -456000;
  d.airspeed = 65.0;
  d.groundspeed = 67.0;
  d.heading = 321;
  d.climb = -34.7;
  d.throttle = 100;
  d.voltage = 24400;
  d.current = -12345;
  d.satellites_visible = 255;
  d.hdop = 0xFFFF;
  d.fix_type = 5;
  d.custom_mode = 5;
  d.type = 1;
  d.home_lat = -127654321;
  d.home_lon = -1237654321;
#endif
}

void loop()
{
  static mavlink_status_t status;
  static mavlink_message_t msg;
  const int chan = MAVLINK_COMM_0;

  static uint32_t nextDisplayEvent = 0;
  uint32_t now;
  
  static bool sensorTouched = false;
  int touchLevel;
  const int TouchOnLevel = 680;
  const int TouchOffLevel = 620;

  static uint8_t displayState = 0;
  const uint32_t maxQrShowTime = 30000;
  
  
  while(Serial1.available())
  {
    uint8_t byte = Serial1.read();
    if(mavlink_parse_char(chan, byte, &msg, &status))
    {
#ifdef DebugMavLink
  #if 1
      char s[80];
      snprintf(s, sizeof(s), "msgid %6lu, compid %3u, sysid %3u", msg.msgid, msg.compid, msg.sysid);
      Serial.println(s);
  #else
      Serial.print("msg_received: ");
      Serial.print(status.msg_received);
      Serial.print(" buffer_overrun: ");
      Serial.print(status.buffer_overrun);
      Serial.print(" parse_error: ");
      Serial.print(status.parse_error);
      Serial.print(" packet_rx_success_count: ");
      Serial.print(status.packet_rx_success_count);
      Serial.print(" packet_rx_drop_count: ");
      Serial.println(status.packet_rx_drop_count);
  #endif
#endif
      if((msg.compid == MavLinkComponentId) && (msg.sysid == MavLinkSystemtId))
      {
        switch(msg.msgid)
        {
          case MAVLINK_MSG_ID_HEARTBEAT: // #0: flight mode and type of vehicle
            timestampMessageReceived[0] = millis();
            d.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
            d.type = mavlink_msg_heartbeat_get_type(&msg);
            break;

          case MAVLINK_MSG_ID_SYS_STATUS: // #1: voltage_battery, current_battery, battery_remaining
            timestampMessageReceived[1] = millis();
            d.voltage = mavlink_msg_sys_status_get_voltage_battery(&msg);
            d.current = mavlink_msg_sys_status_get_current_battery(&msg);
            break;
            
          case MAVLINK_MSG_ID_GPS_RAW_INT: // #24
            timestampMessageReceived[2] = millis();
            d.satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
            d.hdop = mavlink_msg_gps_raw_int_get_eph(&msg);
            d.fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
            break;
            
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // #33
            timestampMessageReceived[3] = millis();
            d.lat = mavlink_msg_global_position_int_get_lat(&msg);
            d.lon = mavlink_msg_global_position_int_get_lon(&msg);
            d.alt = mavlink_msg_global_position_int_get_alt(&msg);
            d.relative_alt = mavlink_msg_global_position_int_get_relative_alt(&msg);
            break;

          case MAVLINK_MSG_ID_HOME_POSITION: // #242 home position
            timestampMessageReceived[4] = millis();
            d.home_lat = mavlink_msg_home_position_get_latitude(&msg);
            d.home_lon = mavlink_msg_home_position_get_longitude(&msg);
            break;
            
          case MAVLINK_MSG_ID_VFR_HUD: // #74: airspeed, groundspeed, alt, climb, heading, throttle
            timestampMessageReceived[5] = millis();
            d.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
            d.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
            d.heading = mavlink_msg_vfr_hud_get_heading(&msg);
            d.climb = mavlink_msg_vfr_hud_get_climb(&msg);
            d.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
            break;
        }
      }
    }
  }

  saveDataOnTimeout();
  
  touchLevel = touchRead(PinTouch);
  if(!sensorTouched && (touchLevel >= TouchOnLevel))
  {
    sensorTouched = true;
    if(displayState == 0) displayState = 1;
    if(displayState == 2) displayState = 3;
  }
  if(sensorTouched && (touchLevel <= TouchOffLevel))
  {
    sensorTouched = false;
  }

#ifdef DebugTouch
  char s[32];
  snprintf(s, sizeof(s), "% 4i %u %u", touchLevel, sensorTouched, displayState);
  Serial.println(s);
#endif
  
  switch(displayState)
  {
    case 0: // show and update numbers
      now = millis();
      if(now >= nextDisplayEvent)
      {
#ifdef DebugOLED
        digitalWrite(PinLed, HIGH);
#endif
        u8g2.clearBuffer();
        
        drawBoxes();
        drawValues();

        u8g2.sendBuffer();
        nextDisplayEvent += DisplayRefreshCycle_ms;
#ifdef DebugOLED
        digitalWrite(PinLed, LOW);
#endif
      }
      break;
      
    case 1: // draw Geo QR Code
      u8g2.clearBuffer();
      drawGeoQr(64, 32, d.lat, d.lon);
      u8g2.sendBuffer();
      displayState = 2;
      nextDisplayEvent = millis() + maxQrShowTime;
      break;
      
    case 2: // show QR, no refresh
      now = millis();
      if(now >= nextDisplayEvent)
      {
        displayState = 3;
      }
      break;
      
    case 3: // prepare state 0
      nextDisplayEvent = millis(); // prepare refresh
      displayState = 0;
      break;
  }
  
}
