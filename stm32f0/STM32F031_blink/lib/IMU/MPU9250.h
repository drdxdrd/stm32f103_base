/*
 * MPU9150.h
 *
 *  Created on: 10 февр. 2016 г.
 *      Author: vladimir
 */

#ifndef MPU9150_H_
#define MPU9150_H_

void init_i2c(void);


// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-9150 Register Map and Descriptions, Revision 4.0, RM-MPU-9150A-00, 9/12/2012 for registers not listed in
// above document; the MPU6050 and MPU 9150 are virtually identical but the latter has an on-board magnetic sensor
//

#define AK8963_ADDRESS   0x0C<<1

#define I2C_IMU 0x68


#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E




//extern float SelfTest[6];


// Using the GY-9150 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// mbed uses the eight-bit device address, so shift seven-bit addresses left by one!


// #define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

/*
extern float ax, ay, az;
extern float gx, gy, gz;
extern float mx, my, mz; // variables to hold latest sensor data values
extern float pitch, yaw, roll;
extern float q1, q2, q3, q4;
extern float deltat;                             // integration interval for both filter schemes
extern float aRes, gRes;      // scale resolutions per LSB for the sensors
extern float magCalibration[3];
extern float magbias[3];  // Factory mag calibration and mag bias
extern float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
extern float mRes; // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
//extern float BMP280temper;
extern int32_t lTemper100;
extern float BMP280pressure;
*/


/*
    void getGres(void);
    void getAres(void);
    void readAccelData(void);
    void readGyroData(void);
    void readMagData(void);
    void calibrateAK8963(void);
    void initAK8963(void);
    int16_t readTempData(void);
    void resetMPU9250(void) ;
    void initMPU9250(void);
    void calibrateMPU9250(float * dest1, float * dest2);
    void MPU9250SelfTest(float * destination);
    float invSqrt(float x);
    void MadgwickQuaternionUpdate(void);
    void MahonyQuaternionUpdate(void);
    void Mahony_no_mag_Update(void);
    void quater2euler(void);
    void BMP280test(void);
*/

class cImu
{
  private:
    float q1, q2, q3, q4;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz; // variables to hold latest sensor data values


    float aRes, gRes;      // scale resolutions per LSB for the sensors
    float magCalibration[3];
    float mRes;          // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
    uint8_t Ascale;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    uint8_t Mscale; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
    uint8_t Mmode;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

    float SelfTest[6];

    void getGres(void);
    void getAres(void);
    float invSqrt(float x);

    void i2cByteWrite(uint8_t reg, uint8_t cmd);    
    void i2cBlockRead(uint reg, uint8_t * tx_pointer, uint8_t buf_lengh);
    uint8_t i2cByteRead(uint8_t reg);

  public:

    float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
    float deltat;                             // integration interval for both filter schemes
    float pitch, yaw, roll;

    cImu();
    void readAccelData(void);
    void readGyroData(void);
    int16_t readTempData(void);
    void resetMPU9250(void);
    void initMPU9250(void);
    void calibrateMPU9250(float * dest1, float * dest2);
    void MPU9250SelfTest(float * destination);
    void calibrateAK8963(void);
    void quater2euler(void);
    void Mahony_no_mag_Update(void);
};

#endif /* MPU9150_H_ */
