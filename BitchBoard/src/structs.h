#ifndef STRUCTS_H
#define STRUCTS_H
// Recall Devices: GPS1, GPS2, IMU, ADC-16, Relay-16, SD Card, SPI to Fc

enum SPI_TELEM_MODE: uint8_t {
  SPI_XFER_INVALID = 0x00,
  SPI_XFER_COMMAND = 0x69,
  SPI_XFER_TELEM = 0x42
};

// See more data on this here: https://blog.veles.rs/sending-struct-via-spi-between-arduino-nano-and-arduino-mega-2560/
// Transmitted to FC
typedef struct __attribute__((packed)) spi_data_t
{
    uint32_t    flags[2];
    uint32_t    num_sats[2];  // number of GPS satellites in view 2x
    double      hdop[2];      // horizontal diliution of precision 2x
    double      lat[2];
    double      lng[2];
    uint32_t    age[2];
    char        date1[10];
    char        time1[10];
    char        date2[8];
    char        time2[8];
    double      altitude_m[2];
    double      speed_mps[2];
    uint32_t    failedChecksum[2];
    float       imu_w[3];
    float       imu_a[3];
    float       imu_temp;
    uint32_t         relay_states[16];
    double      adc[16];
    uint32_t    counter;
    int32_t     checksum;
} b_s;

// Received From FC
typedef struct __attribute__((packed)) spi_command_t
{
    uint32_t relay_states_desired[16];
    uint32_t relay_times_desired_ms[16];
    int32_t checksum;
} b_r;

typedef union __attribute__((packed)) customMessageUnion {
  b_s message;
  unsigned char bytes[sizeof(b_s)];
} b_u;

typedef union __attribute__((packed)) customMessageUnionFC {
  b_r message;
  unsigned char bytes[sizeof(b_r)];
} b_ufc;


#endif
