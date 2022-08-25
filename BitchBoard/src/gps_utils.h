#pragma once
#include "common.h"

// Print Statements, good for debugging. Prints Starts if invalid.
void printFloat(float val, bool valid, int len, int prec);
void printInt(unsigned long val, bool valid, int len);
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
void printStr(const char *str, int len);
void gps_debug_to_console(TinyGPSPlus gps);
void gps_populate_telem_struct(spi_data_t& spi_data, TinyGPSPlus gps1, TinyGPSPlus gps2);