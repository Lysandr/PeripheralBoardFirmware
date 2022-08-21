/* Padraig Lysandrou April 2022
 * "Everything Else Board" or "Bitchboard" Firmware
 * Devices:
 *  GPS1, GPS2,
 *  IMU,
 *  ADC-16,
 *  Relay-16,
 *  SD Card,
 *  SPI to Fc
 * 
 * TODO:
 * What's wrong with IMU? (low priority)
 * MCP startup needs fixing
 * Write packet stuffer to FC
 * Send packet to SD card
 * Write the packet parser and test with FC (harder)
 * Write the relay time commander (hold relay on for X ms)
 * https://github.com/tonton81/SPI_MSTransfer_T4
 * 
 * Transmit: [gps1][gps2][adc][relay state][imu]
 * Receive:  [relay time commands]
 */
#include "main.h"

// Cycle time, debug flag, and number of cycles for the main loop
unsigned long start = 0;
unsigned int  n_cycles_main = 50;
uint32_t      error_flag = 0;
bool          init_relay_states = 1;
bool          SD_card_valid = 1;
bool          time_to_write = 1;
uint32_t      cycle_counter = 0;

void setup()
{
  // Fill up the SPI Buffers with zeros
  memset(&spi_data,    0, sizeof(spi_data_t));
  memset(&spi_command, 0, sizeof(spi_command_t));
  
  // Debug Serial Port and Configure GPS Serial
  Serial.begin(115200);
  GPS1.begin(GPSBaud);
  GPS2.begin(GPSBaud);
  Serial.println("GPS Serial Ports Opened.");

  // Make sure the I2C device is responding. (do we need to use MCP_ADDR?)
  // Might need to make a TwoWire object here?
  if (!RelayDriver.begin_I2C()) {
    Serial.println("MCP23017 Error.");
    error_flag = 1;
  }
  else{
    delay(5);
    // Configure All RelayDriver pins as outputs
    for (uint8_t i = 0; i < 16; i++) RelayDriver.pinMode(i, OUTPUT);
    Serial.println("MCP23017 I2C Opened and Pinmodes Configured.");
    // Make sure the valves are not being actuated.
    for (uint8_t i = 0; i < 16; i++) RelayDriver.digitalWrite(i, 1);
  }
  
  // Make sure Flight Computer is Chatting over SPI
  // NEW SPI_MSTransfer library does not return anything
  // fc_spi.begin();
  Serial.println("FC SPI Line Opened.");
  // if(debug_mode) {fc_spi.onTransfer(myCB_debug);}
  // else{ fc_spi.onTransfer(myCB);}
  Serial.println("FC SPI Line Opened.");

  // Configure the IMU with the following (TBD!)
  if(imu.begin() < 0)
  {
    Serial.println("IMU SPI Failed.");
    error_flag = 1;
  }
  else{
    // Configure IMU settings
    imu.setGyroRange(IMU380::GYRO_RANGE_250DPS);
    imu.setFilter(IMU380::BUTTERWORTH_20HZ);
    imu.setDataRate(IMU380::ODR_50HZ);
    imu.setDataReady(false, false);
    Serial.println("IMU Configured.");
  }

  // TODO: Write Calibration Routine
  // imu.setBias(-0.0744499980, -0.0623099982, 0.2286699940, 0.078577361, -0.003357263, -0.005844155);
  // imu.setScale(1.007145919, 0.999380455, 0.998160113);

  // Configure the MUX common analog read input, ADC enable, and enable it (must be low)
  pinMode(adc_common_input, INPUT);
  pinMode(adc_enable, OUTPUT);
  digitalWrite(adc_enable, LOW);

  // Set up the SD Card (Should be SPI2 by default)
  if (!SD.begin(SD_cs)){
    Serial.println("Card failed, or not present..");
    SD_card_valid = 0;
    error_flag = 1;
  }
  else{
    Serial.println("SD Card Initialized.");
    char sz[15];
    uint8_t filename_date_good=0;
    if(gps1.date.isValid()){
      sprintf(sz, "%02d_%02d_%02d", gps1.date.month(), gps1.date.day(), gps1.date.year());
      filename_date_good = 1;
    }
    else if(gps2.date.isValid()){
      sprintf(sz, "%02d_%02d_%02d", gps2.date.month(), gps2.date.day(), gps2.date.year());
      filename_date_good = 1;
    }

    if(filename_date_good){
      char s[80];
      const char fname1[] = "bitchboard_telemetry_"; 
      const char fname2[] = ".csv";
      strcat(s, fname1); strcat(s, sz); strcat(s, fname2);
      dataFile = SD.open(s, FILE_WRITE);
    }
    else
      dataFile = SD.open("bitchboard_telemetry.csv", FILE_WRITE);
  } 

  Serial.println("Finished Device Setup.");
  digitalWrite(13, HIGH);
}


////////////////////////////////////////////////////////////////////////////////////////
// Objective: Rx/Tx updates to/from FC at 50Hz.
// Send: GPS1, GPS2, IMU, ADC states, Relay States
// Receive: Relay State Commands
// Other: Store Telemetry on SD Card at a lower rate.
void flight_loop(){

  /* Read the flight computer buffer. If there is a fresh command packet, reset the timers
   * only update the timers in the case we get an updated packet from FC 
   * this should be every 250ms or so.
   */
  if(_new_fcrx){
    populate_fc_rx_data();
    for (uint8_t i = 0; i < N_COUNT; i++){
      _relay_timers[i]   = spi_command.relay_times_desired_ms[i];
      _relay_commands[i] = spi_command.relay_states_desired[i];
    }
  }

  /* Now activate relays based on those commands. 
   * _relay_commands is a little redundant but offers extra masking 
   * ability for any flight code that wants to act as an override
   */
  for (uint8_t i = 0; i < N_COUNT; i++){
    if(_relay_timers[i] > 0 && _relay_commands[i] == 1){
      // Low means relays are active.
      RelayDriver.digitalWrite(i, LOW);
      spi_data.relay_states[i] = HIGH;
      _relay_timers[i] -= n_cycles_main;
    }
    else{
      // High means relays aren't activated.
      RelayDriver.digitalWrite(i, HIGH);
      spi_data.relay_states[i] = LOW;
      _relay_timers[i] = 0; 
    }
  }

  /* 
   * Pull imu data, and populate telem struct
   */
  imu.readSensor();
  _gyro_status  = imu.getStatus();
  _imu_omega[0] = imu.getGyroX();
  _imu_omega[1] = imu.getGyroY();
  _imu_omega[2] = imu.getGyroZ();
  _imu_accel[0] = imu.getAccelX();
  _imu_accel[1] = imu.getAccelY();
  _imu_accel[2] = imu.getAccelZ();
  for (int i = 0; i < 3; i++) {
    spi_data.imu_w[i] = _imu_omega[i];
    spi_data.imu_a[i] = _imu_accel[i];
  }
  spi_data.imu_temp = imu.getTemp();
  spi_data.flags[0] = error_flag;
  spi_data.flags[1] = (uint32_t)_gyro_status;

  /* Now get ADC data (does this need a delay? 16ms)
   * Configure the mux address lines via channl(i)
   * The Teensy ADC is measuring 5V at the top end.
  */
  for (int i = 0; i < N_COUNT; i++) {
    adc_mux.channel(i); delay(1);
    spi_data.adc[i] = analogRead(adc_common_input);
  }

  // Populate the telemetry struct with GPS1 and GPS2 data
  gps_populate_telem_struct();
  // tx_data_to_fc(spi_data);

  // Store the data on the SD card possibly at lower rate?
  if (SD_card_valid && time_to_write) {
    print_data_to_sd();
    dataFile.flush();
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// DEBUG THE FLIGHT ROUTINE, RUN SLOWER
void debug_loop(){
  // Make the main loop much slower
  n_cycles_main = 1000;
  time_to_write = 1;

  /*
   * PRINT OUT A BUNCH OF SHIT
  */
  Serial.println(F("----------------------------------------------------------------------------------------"));
  Serial.println(F("nsats-hdop-latitude----longitude--age--[date----------------]---alt----spd--crse-fchksum"));
  gps_debug_to_console(gps1);
  gps_debug_to_console(gps2);

  // Read and Print off IMU Packet
  imu.readSensor();
  Serial.print("temperature: "); Serial.print(imu.getTemp());
  Serial.print(" SN: "); Serial.print(imu.getSerialNumber());
  Serial.print(" Status: "); Serial.print(imu.getStatus());
  Serial.print(" wx: ");  Serial.print(imu.getGyroX());
  Serial.print(" wy: "); Serial.print(imu.getGyroY());
  Serial.print(" wz: "); Serial.print(imu.getGyroZ());
  Serial.print(" ax: "); Serial.print(imu.getAccelX());
  Serial.print(" ay: "); Serial.print(imu.getAccelY());
  Serial.print(" az: "); Serial.print(imu.getAccelZ());
  Serial.println();

  // Check all mux input lines and print them off
  Serial.print("All ADC Lines From Mux: ");
  for (int i = 0; i < N_COUNT; i++) {
    adc_mux.channel(i); delay(5);
    Serial.print((analogRead(adc_common_input)*5.0)/1023.0); Serial.print(" / ");
  }
  Serial.println();

  /* RUN THE MAIN LOOP, BUT SET UP SOME INITIAL CONDITIONS ETC
   * 
   */

  // for (uint8_t i = 0; i < N_COUNT; i++){
  //   RelayDriver.digitalWrite(i, HIGH);
  // }

  unsigned long start_flight = millis();
  if(cycle_counter%5 == 0) _relay_timers[5] = 2000;
  _relay_commands[5] = 1;
  

  flight_loop();
  Serial.print("Flight Loop takes this long to run (ms): ");
  Serial.println(millis() - start_flight);

  digitalWrite(13, HIGH);
}


void loop()
{
  // Populate Ring Buffer from FC
  // _new_fcrx = fc_spi.events();

  // Execution Loop
  if (millis() - start >= n_cycles_main )
  {
    // Reset Timer
    start = millis();
    cycle_counter++;
    spi_data.counter = cycle_counter;

    // Write to SD card every other cycle.
    if(cycle_counter%2 == 0) time_to_write = 1;

    // Do the thing.
    if (debug_mode){
      time_to_write = 1;
      debug_loop();
    }
    else{
      /* 
       * Give Each GPS time to update a packet if a new solution is found
       * They update at ~171ms interval anyway. Updates dont happen when Age is 0 btw.
      */
      flight_loop();
      smartDelayGPS1(10);
      smartDelayGPS2(10);
    }    
  }

  // Keep the buffers fed
  smartDelayGPS1(0);
  smartDelayGPS2(0);
}