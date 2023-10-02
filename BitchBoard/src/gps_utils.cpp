#include "gps_utils.h"

////////////////////////////////////////////////////////////////////////////////////
/* 
 * List of Utilities initially used in GPS debugging
*/

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

// make sure that * isValid is correct
void gps_populate_telem_struct(spi_data_t& spi_data, TinyGPSPlus gps1, TinyGPSPlus gps2){
  spi_data.num_sats[0] = gps1.satellites.value() * gps1.satellites.isValid();
  spi_data.num_sats[1] = gps2.satellites.value() * gps2.satellites.isValid();
  spi_data.hdop[0]  = gps1.hdop.hdop() * gps1.hdop.isValid();
  spi_data.hdop[1]  = gps2.hdop.hdop() * gps2.hdop.isValid();
  spi_data.lat[0]   = gps1.location.lat() * gps1.location.isValid();
  spi_data.lat[1]   = gps2.location.lat() * gps2.location.isValid();
  spi_data.lng[0]   = gps1.location.lng() * gps1.location.isValid();
  spi_data.lng[1]   = gps2.location.lng() * gps2.location.isValid();
  spi_data.age[0]   = gps1.location.age() * gps1.location.isValid();
  spi_data.age[1]   = gps2.location.age() * gps2.location.isValid();
  if(gps1.date.isValid()){
    char sz[10];
    sprintf(sz, "%02d/%02d/%02d ", gps1.date.month(), gps1.date.day(), gps1.date.year());
    for (uint8_t i = 0; i < 10; i++) spi_data.date1[i] = sz[i]; 
  }
  if(gps2.date.isValid()){
    char sz[10];
    sprintf(sz, "%02d/%02d/%02d ", gps2.date.month(), gps2.date.day(), gps2.date.year());
    for (uint8_t i = 0; i < 10; i++) spi_data.date2[i] = sz[i]; 
  }
  if(gps1.time.isValid()){
    char sz[8];
    sprintf(sz, "%02d:%02d:%02d ", gps1.time.hour(), gps1.time.minute(), gps1.time.second());
    for (uint8_t i = 0; i < 8; i++) spi_data.time1[i] = sz[i]; 
  }
  if(gps2.time.isValid()){
    char sz[8];
    sprintf(sz, "%02d:%02d:%02d ", gps2.time.hour(), gps2.time.minute(), gps2.time.second());
    for (uint8_t i = 0; i < 8; i++) spi_data.time2[i] = sz[i]; 
  }
  spi_data.altitude_m[0] = gps1.altitude.meters() * gps1.altitude.isValid();
  spi_data.altitude_m[1] = gps2.altitude.meters() * gps2.altitude.isValid();
  spi_data.speed_mps[0] = gps1.speed.mps() * gps1.speed.isValid();
  spi_data.speed_mps[1] = gps2.speed.mps() * gps2.speed.isValid();
  spi_data.failedChecksum[0] = gps1.failedChecksum();
  spi_data.failedChecksum[1] = gps2.failedChecksum();
}

