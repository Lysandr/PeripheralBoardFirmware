#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_MCP23X17.h>
#include <IMU380.h>
#include <CD74HC4067.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include "structs.h"
#include "hw_config.h"
#include "gps_utils.h"
#include "SPISlave_T4.h"

/* Links you may find important later!
 * https://forum.pjrc.com/threads/59254-SPI-Slave-Mode-on-Teensy-4
 * https://forum.pjrc.com/threads/66389-SPISlave_T4/page3
 * https://forum.pjrc.com/threads/66389-SPISlave_T4
 * https://github.com/jacgoudsmit/SPISlave_T4
 * https://github.com/tonton81/SPI_MSTransfer_T4
 * https://forum.up-community.org/discussion/4781/wrtiting-and-spi-program-for-upboard
 * https://blog.veles.rs/sending-struct-via-spi-between-arduino-nano-and-arduino-mega-2560/
 * https://github.com/Aceinna/docs_aceinna-dmu380/blob/master/software/SPImessaging.rst
 * https://www.mouser.com/datasheet/2/940/6020_3885_01_G_OpenIMU300ZI_Datasheet-2939155.pdf
 * https://buildmedia.readthedocs.org/media/pdf/openimu/latest/openimu.pdf
 * https://www.mouser.com/ProductDetail/ACEINNA/OpenIMU300ZI?qs=wnTfsH77Xs4swZX47E%2FWeA%3D%3D
 * 
 * 
 */