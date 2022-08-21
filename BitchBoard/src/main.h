#pragma once
// All libraries
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_MCP23X17.h>
#include <IMU380.h>
#include <CD74HC4067.h>
#include <SPI.h>
#include <SD.h>
#include "hw_config.h"
#include "structs.h"
#include <SPI_MSTransfer_T4.h>

const int     debug_mode =  1;
uint32_t      _new_fcrx = 0;

// TODO: Use normal SPI library? I think I did this for more optionality tho.
// Flight Computer SPI Device Interface (cs, sck, miso, mosi) (should be SPI0)
#define FLIGHT_COMPUTER_CS 10
#define RX_LEN 66
#define TX_LEN 66
#define N_COUNT 16

// Construct the SPI tx/rx structs
spi_data_t    spi_data;     // data from eeb to up
spi_command_t spi_command;  // data from up to eeb
File dataFile;

uint8_t  fc_rx_buffer[sizeof(spi_command_t)];
uint8_t  fc_tx_buffer[sizeof(spi_data_t)];
uint8_t  _relay_commands[N_COUNT];
int      _relay_timers[N_COUNT];

// Adafruit_SPIDevice fc_spi = Adafruit_SPIDevice(FLIGHT_COMPUTER_CS, 13, 12, 11);
// Tonton18's confusing Teensy slavemode bullshit.
SPI_MSTransfer_T4<&SPI, 0x1234> fc_spi;

// TinyGPS Object
TinyGPSPlus gps1;
TinyGPSPlus gps2;

// MCP Object, Uses I2C0, pin 18/19 for SDA/SCL
Adafruit_MCP23X17 RelayDriver;

// Set up the IMU, on Teensy SPI1 port
IMU380 imu(SPI1, 0);
uint16_t _gyro_status;
float _imu_omega[3];
float _imu_accel[3];
 
// Set up the Analog Mux -- (s0 s1 s2 s3)
CD74HC4067 adc_mux(3, 4, 5, 6);
#define adc_common_input A10
#define adc_enable 2

// Set up the SD Card
const int SD_cs = BUILTIN_SDCARD; 

////////////////////////////////////////////////////////////////////////////////////////
void myCB_debug(uint16_t *buffer, uint16_t length, AsyncMST info) {
  for ( int i = 0; i < length; i++ ) {
    Serial.print(buffer[i], HEX); Serial.print(" ");
  }
  Serial.print(" --> Length: "); Serial.print(length);
  Serial.print(" --> PacketID: "); Serial.println(info.packetID, HEX);
}

// Populate Buffer NOT SURE IF THIS IS RIGHT...
void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {
  if(length == sizeof(spi_command)){
    // Transfer the buffer over
    for (uint8_t i=0; i < length; i++) fc_rx_buffer[i] = buffer[i];
  }
}

// /* Transmit the data struct to teh flight computer
//  * create a message union, package it up and 
//  * transfer it... make this work.
//  * TODO: add delay? fix data types?
//  */
// void tx_data_to_fc(spi_data_t table){
//   customMessageUnion msgUnion;
//   msgUnion.message = table;
//   // https://blog.veles.rs/sending-struct-via-spi-between-arduino-nano-and-arduino-mega-2560/
//   // TODO: NEED TO FIX THIS -- INCOMPATABLE DATA TYPES
//   fc_spi.transfer16(msgUnion.bytes, sizeof(b_s), 0);
//   // delay(10);
// }

void populate_fc_rx_data(){
    customMessageUnionFC msgUnion;
    for (size_t i = 0; i < sizeof(b_r); i++)
      msgUnion.bytes[i] = fc_rx_buffer[i];

    spi_command = msgUnion.message;
    _new_fcrx = 0;
}

// TODO make one large string, then print that?
// TODO STRING WITH LARGER PRECISION!
void print_data_to_sd(){
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.flags[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.num_sats[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.hdop[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.lat[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.lng[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.age[i]) + ",");
  for (size_t i = 0; i < sizeof(spi_data.date1); i++)
    dataFile.print(String(spi_data.date1[i]));
  dataFile.print(",");
  for (size_t i = 0; i < sizeof(spi_data.time1); i++)
    dataFile.print(String(spi_data.time1[i]));
  dataFile.print(",");
  for (size_t i = 0; i < sizeof(spi_data.date2); i++)
    dataFile.print(String(spi_data.date2[i]));
  dataFile.print(",");
  for (size_t i = 0; i < sizeof(spi_data.time2); i++)
    dataFile.print(String(spi_data.time2[i]));
  dataFile.print(",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.altitude_m[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.speed_mps[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.failedChecksum[i]) + ",");
  for (size_t i = 0; i < 3; i++) dataFile.print(String(spi_data.imu_w[i]) + ",");
  for (size_t i = 0; i < 3; i++) dataFile.print(String(spi_data.imu_a[i]) + ",");
  dataFile.print(String(spi_data.imu_temp) + ",");
  for (size_t i = 0; i < 16; i++) dataFile.print(String(spi_data.relay_states[i]) + ",");
  for (size_t i = 0; i < 16; i++) dataFile.print(String(spi_data.adc[i]) + ",");
  dataFile.print(String(spi_data.counter) + ',');
  dataFile.println(String(spi_data.checksum));
}










////////////////////////////////////////////////////////////////////////////////////
/* 
 * List of Utilities initially used in GPS debugging
*/
// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelayGPS1(unsigned long ms)
{
  unsigned long start = millis();
  // Do this once, and do it more 
  do 
  {
    while (GPS1.available())
    {
      gps1.encode(GPS1.read());
    }
  } while (millis() - start < ms);
}
void smartDelayGPS2(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS2.available())
      gps2.encode(GPS2.read());
  } while (millis() - start < ms);
}

// Print Statements, good for debugging. Prints Starts if invalid.
void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
}

void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
}

void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
}

void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
}

void gps_debug_to_console(TinyGPSPlus gps)
{
  printInt(   gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat( gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat( gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat( gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(   gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat( gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat( gps.speed.mps(), gps.speed.isValid(), 6, 2);
  printStr(   gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
  printInt(   gps.failedChecksum(), true, 9);
  Serial.println();
}

void gps_populate_telem_struct(){
  spi_data.num_sats[0] = gps1.satellites.value() * gps1.satellites.isValid();
  spi_data.num_sats[1] = gps2.satellites.value() * gps2.satellites.isValid();
  spi_data.hdop[0]  = gps1.hdop.hdop() * gps1.hdop.isValid();
  spi_data.hdop[1]  = gps2.hdop.hdop() * gps2.hdop.isValid();
  spi_data.lat[0]   = gps1.location.lat() * gps1.location.isValid();
  spi_data.lat[1]   = gps2.location.lat() * gps2.location.isValid();
  spi_data.age[0]   = gps1.location.age() * gps1.location.isValid();
  spi_data.age[1]   = gps2.location.age() * gps2.location.isValid();
  if(gps1.date.isValid()){
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", gps1.date.month(), gps1.date.day(), gps1.date.year());
    for (uint8_t i = 0; i < 32; i++) spi_data.date1[i] = sz[i]; 
  }
  if(gps2.date.isValid()){
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", gps2.date.month(), gps2.date.day(), gps2.date.year());
    for (uint8_t i = 0; i < 32; i++) spi_data.date2[i] = sz[i]; 
  }
  if(gps1.time.isValid()){
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", gps1.time.hour(), gps1.time.minute(), gps1.time.second());
    for (uint8_t i = 0; i < 32; i++) spi_data.time1[i] = sz[i]; 
  }
  if(gps2.time.isValid()){
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", gps2.time.hour(), gps2.time.minute(), gps2.time.second());
    for (uint8_t i = 0; i < 32; i++) spi_data.time2[i] = sz[i]; 
  }
  spi_data.altitude_m[0] = gps1.altitude.meters() * gps1.altitude.isValid();
  spi_data.altitude_m[1] = gps2.altitude.meters() * gps2.altitude.isValid();
  spi_data.speed_mps[0] = gps1.speed.mps() * gps1.speed.isValid();
  spi_data.speed_mps[1] = gps2.speed.mps() * gps2.speed.isValid();
  spi_data.failedChecksum[0] = gps1.failedChecksum();
  spi_data.failedChecksum[1] = gps2.failedChecksum();
}

