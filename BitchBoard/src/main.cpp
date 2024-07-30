#define DEBUG 0    // SET TO 0 OUT TO REMOVE TRACES

/* Padraig Lysandrou April 2022
 * "Everything Else Board" or "Bitchboard" Firmware
 * Devices: GPS1, GPS2, Backup IMU,
 *  ADC-16, Relay-16, SD Card, SPI to FC
 *  IMU reports in deg/s and m/s2
 */
#include "common.h"

/*  
* Set up all the peripheral devices
*/
SPISlave_T4 mySPI(0, SPI_8_BITS);
File dataFile;
Adafruit_MCP23X17 RelayDriver;  // MCP Object, Uses I2C0, pin 18/19 for SDA/SCL
IMU380 imu(SPI1, 0);
CD74HC4067 adc_mux(3, 4, 5, 6);
TinyGPSPlus gps1;
TinyGPSPlus gps2;

// All Interrupt Timers
IntervalTimer ParseAndPackTimer;
IntervalTimer SDCardWriteTimer;

// Global Telemetry Structs
spi_data_t          spi_data; // = {1};
spi_command_t       spi_command;
customMessageUnion  spi_data_union;
customMessageUnionFC spi_command_union;

// Configure filename
char file_name[80] = "bitchboard_telemetry";

// Cycle time, debug flag, and number of cycles for the main loop
unsigned int  n_ms_main = 100; // 1/100ms = 10Hz
unsigned int  n_ms_SD   = 500; // 1/500ms = 2Hz
uint32_t  error_flag = 0b00000000;
bool      SD_card_valid = 0;
uint8_t   _relay_commands[N_COUNT];
int       _relay_timers[N_COUNT];
uint16_t  _gyro_status;
int       spi_telem_mode = 0;
const int rx_packet_idx_max = sizeof(b_r);
int       rx_packet_idx = 0;
int       new_rx_packet = 0;
const int tx_packet_idx_max = sizeof(b_s);
int       tx_packet_idx = 0;

void print_data_to_sd()
{
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.flags[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.num_sats[i]) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.hdop[i], 10) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.lat[i], 10) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.lng[i], 10) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.age[i], 10) + ",");
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
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.altitude_m[i], 10) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.speed_mps[i], 10) + ",");
  for (size_t i = 0; i < 2; i++) dataFile.print(String(spi_data.failedChecksum[i]) + ",");
  for (size_t i = 0; i < 3; i++) dataFile.print(String(spi_data.imu_w[i], 10) + ",");
  for (size_t i = 0; i < 3; i++) dataFile.print(String(spi_data.imu_a[i], 10) + ",");
  dataFile.print(String(spi_data.imu_temp) + ",");
  for (size_t i = 0; i < 16; i++) dataFile.print(String(spi_data.relay_states[i]) + ",");
  for (size_t i = 0; i < 16; i++) dataFile.print(String(spi_data.adc[i], 10) + ","); 
  dataFile.print(String(spi_data.counter) + ',');
  dataFile.println(String(spi_data.checksum));
}

/*
 * SPI Rx and Tx Function
 * gets called by an interrupt service routine.
*/
void flight_computer_spi()
{
  // We expect this to be available for the correct number of clock cycles 
  // from the master
  uint32_t i = 0;
  /*
  if (mySPI.active() == 0) {
    D_println("SPI Slave Not Active.");
    return;
  }
  */
  while(mySPI.active()) {
    if (mySPI.available()) {
      uint32_t startbyte = mySPI.popr();

      if (i == 0)
      {
        // If the first byte is not a start byte, we're out of sync
        switch (startbyte) {
          case 0x69:
            spi_telem_mode = 1;
            rx_packet_idx = 0;
            break;
          case 0x42:
            spi_telem_mode = 2;
            tx_packet_idx = 0;
            break;
          default:
            spi_telem_mode = 0;
            rx_packet_idx = 0;
            tx_packet_idx = 0;
            break;
        }
      }

      // If the incoming packet is a relay command packet
      if(spi_telem_mode == 1){
        // Pack spi_command via the custom message union
        spi_command_union.bytes[rx_packet_idx] = startbyte;
        rx_packet_idx ++;

        // Convert the message to the struct. The reset the telem mode, and the index.
        if(rx_packet_idx == rx_packet_idx_max){
          spi_command = spi_command_union.message;
          new_rx_packet = 1;
          spi_telem_mode = 0;
          rx_packet_idx = 0;
        }
      }
      // If the FC has requested telem, push out the struct
      // byte by byte until the end. The FC should command more bytes
      // Than required
      if(spi_telem_mode == 2){
        // Push out telemetry byte by byte
        mySPI.pushr(spi_data_union.bytes[tx_packet_idx]);
        tx_packet_idx ++;

        // If we have transmitted out all the packets then we're done!
        if(tx_packet_idx == tx_packet_idx_max){
          spi_telem_mode = 0;
          tx_packet_idx = 0;
        }
      }
      i++;
    }


  }
}

/* 
 * This custom version of delay() ensures that the gps object
 * is being "fed". I'm being lazy by making two of them.
*/
void smartDelayGPS1(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS1.available())
      gps1.encode(GPS1.read());
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

////////////////////////////////////////////////////////////////////////////////////////
// Objective: Rx/Tx updates to/from FC at N-Hz.
// Send: GPS1, GPS2, IMU, ADC states, Relay States
// Receive: Relay State Commands
// Other: Store Telemetry on SD Card at a lower rate.
void flight_loop()
{
  spi_data.counter++;

  /* 
   * Pull imu data, and populate telem struct
   */
  imu.readSensor();
  _gyro_status  = imu.getStatus();
  spi_data.imu_w[0] = imu.getGyroX();
  spi_data.imu_w[1] = imu.getGyroY();
  spi_data.imu_w[2] = imu.getGyroZ();
  spi_data.imu_a[0] = imu.getAccelX();
  spi_data.imu_a[1] = imu.getAccelY();
  spi_data.imu_a[2] = imu.getAccelZ();
  spi_data.imu_temp = imu.getTemp();
  spi_data.flags[0] = error_flag;
  spi_data.flags[1] = (uint32_t)_gyro_status;

  /* 
   * Now get ADC data (does this need a delay? 16ms)
   * Configure the mux address lines via channel(i)
   * The Teensy ADC is measuring 5V at the top end.
  */
  for (int i = 0; i < N_COUNT; i++)
  {
    adc_mux.channel(i);
    delay(1);
    // Need to account for the voltage divider on the mux (1/2)
    spi_data.adc[i] = ((double)analogRead(adc_common_input)/1023.0) * 3.3 * 2.0;
  }

  // Keep buffers fed.
  smartDelayGPS1(10);
  smartDelayGPS2(10);

  // Populate the telemetry struct with GPS1 and GPS2 data
  gps_populate_telem_struct(spi_data, gps1, gps2);

  // Pack the data union
  // strcpy(spi_data.date1, "HELLLD\0");
  spi_data_union.message = spi_data;
  // memcpy(&(spi_data_union.message), &(spi_data), sizeof(b_s));


    // If we got a new command packet, we should stuff the timers!
  if (new_rx_packet)
  {
    for (uint8_t i = 0; i < N_COUNT; i++){
      _relay_commands[i] = spi_command.relay_states_desired[i];
      _relay_timers[i]   = spi_command.relay_times_desired_ms[i];
      }
    new_rx_packet = 0;
  }

  /* Activate relays based on FC commands. 
  * _relay_commands is a little redundant but offers extra masking 
  * ability for any flight code that wants to act as an override (?)
  */
  for (uint8_t i = 0; i < N_COUNT; i++){
    if(_relay_timers[i] > 0 && _relay_commands[i] == 1){
      // Low means relays are active.
      RelayDriver.digitalWrite(i, LOW);
      spi_data.relay_states[i] = HIGH;
      _relay_timers[i] -= n_ms_main; // Decrement for the time the valve is on
    }
    else{
      // High means relays aren't activated.
      RelayDriver.digitalWrite(i, HIGH);
      spi_data.relay_states[i] = LOW;
      _relay_timers[i] = 0;  //Set the timer values to zero
    }
  }

}

// Could be happening mid write cycle...
void write_to_sd_card()
{
  if (SD_card_valid)
  {
    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    D_println("Echo, I am Alive and Writing to SD Card, Counter: " + String(spi_data.counter));
    dataFile = SD.open(file_name, FILE_WRITE);
    print_data_to_sd();
    // dataFile.flush();
    dataFile.close();
  }
}

void setup()
{
  // Delay for X seconds while all other things get up to speed
  delay(5000);

  // pinMode(LED_BUILTIN, OUTPUT);

  // Fill up the SPI Buffers with zeros
  memset(&spi_data,    0, sizeof(spi_data_t));
  memset(&spi_command, 0, sizeof(spi_command_t));
  
  // Debug Serial Port and Configure GPS Serial
  D_SerialBegin(115200);
  
  GPS1.begin(GPSBaud);
  GPS2.begin(GPSBaud);
  D_println("GPS Serial Ports Opened.");

  // Keep buffers fed.
  smartDelayGPS1(2000);
  smartDelayGPS2(2000);

  // Make sure the I2C device is responding. (do we need to use MCP_ADDR?)
  // Might need to make a TwoWire object here?
  if (!RelayDriver.begin_I2C())
  {
    D_println("MCP23017 Error.");
    error_flag = error_flag | 0b00000001;
  }
  else
  {
    D_println("MCP23017 I2C Opened and Pinmodes Configured.");
    delay(5);
    // Configure All RelayDriver pins as outputs and not being actuated
    for (uint8_t i = 0; i < 16; i++)
    {
      RelayDriver.pinMode(i, OUTPUT);
      RelayDriver.digitalWrite(i, 1);
    }
  }
  
  // Make sure Flight Computer is Chatting over SPI
  // NEW SPI_MSTransfer library does not return anything
  // fc_spi.begin();
  mySPI.onReceive(flight_computer_spi);
  mySPI.begin();
  mySPI.swapPins(true); // false);
  D_println("FC SPI Line Opened.");

  // Configure the IMU with the following (TBD!)
  if(imu.begin() < 0)
  {
    D_println("IMU SPI Failed.");
    error_flag = error_flag | 0b00000010;
  }
  else{
    // Configure IMU settings
    imu.setGyroRange(IMU380::GYRO_RANGE_250DPS);
    imu.setFilter(IMU380::BUTTERWORTH_20HZ);
    imu.setDataRate(IMU380::ODR_50HZ);
    imu.setDataReady(false, false);
    D_println("IMU Configured.");
  }

  // TODO: Write Calibration Routine
  imu.setBias(0.2912730622, -0.1078606371, -0.2662787127, 0.0, 0.0, 0.0);
  // imu.setScale(1.007145919, 0.999380455, 0.998160113);

  // Configure the MUX common analog read input, ADC enable, and enable it (must be low)
  pinMode(adc_common_input, INPUT);
  pinMode(adc_enable, OUTPUT);
  digitalWrite(adc_enable, LOW);

  /*
  * Set up the SD Card (Should be SPI2 by default)
  * Open a new telemetry file with a date and write to it...
  */ 
  if (!SD.begin(SD_cs))
  {
    D_println("Card failed, or not present..");
    error_flag = error_flag | 0b00000100;
  }
  else
  {
    SD_card_valid = 1;
    delay(500);
    D_println("SD Card Initialized.");
    char sz[15];
    uint8_t filename_date_good=0;
    if(gps1.date.isValid() && gps1.date.year() > 2000)
    {
      D_println("GPS1 Date Valid.");
      sprintf(sz, "_%02d_%02d", gps1.date.month(), gps1.date.day());
      filename_date_good = 1;
    }
    else if(gps2.date.isValid() && gps2.date.year() > 2000)
    {
      D_println("GPS2 Date Valid.");
      sprintf(sz, "_%02d_%02d", gps2.date.month(), gps2.date.day());
      filename_date_good = 1;
    }

    if(filename_date_good)
    {
      D_println("Writing to dated SD file: " + String(sz));
      const char fname2[] = ".csv";
      strcat(file_name, sz); strcat(file_name, fname2);
    }
    else
    {
      D_println("Didn't get good GPS date, writing to general SD file.");
      const char fname2[] = ".csv";
      strcat(file_name, fname2);
    }

    dataFile = SD.open(file_name, FILE_WRITE);
    dataFile.println("Newline on Reboot");
    dataFile.close();
    D_println("Wrote Newline Reboot Line to SD file.");
  } 

  // Set counter back to zero explicitly
  spi_data.counter = 0;
  
  if(!ParseAndPackTimer.begin(flight_loop, n_ms_main*1000))
    D_println("Failed To Start Flight Loop.");

  if(!SDCardWriteTimer.begin(write_to_sd_card, n_ms_SD*1000))
    D_println("Failed To Start SD Write Loop.");

  D_println("Finished Device Setup.");
}

void loop()
{
  // Just using interrupts
}