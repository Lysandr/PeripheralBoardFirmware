/* Anything generally annoying will go here instead
 * of the main header
 */

// Bind Serial ports to their devices
#define GPS1 Serial7
#define GPS2 Serial8

// MCP23017 Hardware Address, all pins grounded
#define MCP_ADDR      (0x40)

// Configure the first SPI port as the serial line to FC
// #define fc_spi SPI

// Teensy Side RX and TX pins for each of these ports
const uint32_t GPSBaud = 38400;

// Define all the relay attachment pins
// See this for pin mappings: https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
#define RELAY1_PIN 0
#define RELAY2_PIN 1
#define RELAY3_PIN 2
#define RELAY4_PIN 3
#define RELAY5_PIN 4
#define RELAY6_PIN 5
#define RELAY7_PIN 6
#define RELAY8_PIN 7
#define RELAY9_PIN 8
#define RELAY10_PIN 9
#define RELAY11_PIN 10
#define RELAY12_PIN 11
#define RELAY13_PIN 12
#define RELAY14_PIN 13
#define RELAY15_PIN 14
#define RELAY16_PIN 15
