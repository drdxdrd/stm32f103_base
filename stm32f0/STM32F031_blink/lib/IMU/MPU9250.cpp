#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <system.h>
#include <MPU9250.h>
#include "math.h"


#define PI 3.14159265

//#define I2C_IMU 0x68

void init_i2c(void)
{
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOB);
  rcc_set_i2c_clock_hsi(I2C1);

	i2c_reset(I2C1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
	i2c_peripheral_disable(I2C1);
	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, 0);
	/* HSI is at 8Mhz */
	i2c_set_speed(I2C1, i2c_speed_fm_400k, 8);
	//configure No-Stretch CR1 (only relevant in slave mode)
	i2c_enable_stretching(I2C1);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C1);
	i2c_peripheral_enable(I2C1);  

}


cImu::cImu()
{
  q1=1;
  q2=0;
  q3=0;
  q4=0;

  mRes = 10.*1229./4096.;
  Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
  Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
}


void cImu::getGres(void)
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0*PI/180.0f/32768.0;
    	  //gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0*PI/180.0f/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0*PI/180.0f/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0*PI/180.0f/32768.0;
          break;
  }
}


void cImu::getAres(void)
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void cImu::i2cBlockRead(uint reg, uint8_t * tx_pointer, uint8_t buf_lengh)
{
  uint8_t cmd = reg; 
  i2c_transfer7(I2C1, MPU9250_ADDRESS, &cmd, 1, tx_pointer, buf_lengh);  
}

void cImu::i2cByteWrite(uint8_t reg, uint8_t cmd)
{
  uint8_t command[2];
  command[0]=reg;
  command[1]=cmd;
  uint8_t dummy;
  i2c_transfer7(I2C1, MPU9250_ADDRESS, command, 2, &dummy, 0);  
} 

uint8_t cImu::i2cByteRead(uint8_t reg)
{
  uint8_t out;
  i2c_transfer7(I2C1, MPU9250_ADDRESS, &reg, 1, &out, 1);  
  return out;
}


void cImu::readAccelData(void)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  i2cBlockRead(ACCEL_XOUT_H, rawData, 6);
  //i2cReadyFlag.wait();
  ax = (float)((int16_t)(((uint16_t)rawData[0] << 8) | rawData[1])) ;  // Turn the MSB and LSB into a signed 16-bit value
  ay = (float)((int16_t)(((uint16_t)rawData[2] << 8) | rawData[3])) ;
  az = (float)((int16_t)(((uint16_t)rawData[4] << 8) | rawData[5])) ;
  // Now we'll calculate the accleration value into actual g's
//  ax *= aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
//  ay *= aRes; // - accelBias[1];
//  az *= aRes; // - accelBias[2];

  float norm;
  norm = ax * ax + ay * ay + az * az;
  norm = invSqrt(norm);
  ax *= norm;
  ay *= norm;
  az *= norm;
}

void cImu::readGyroData(void)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  i2cBlockRead(GYRO_XOUT_H, rawData, 6);
  //i2cReadyFlag.wait();
  gx = (float)((int16_t)(((uint16_t)rawData[0] << 8) | rawData[1])) ;  // Turn the MSB and LSB into a signed 16-bit value
  gy = (float)((int16_t)(((uint16_t)rawData[2] << 8) | rawData[3])) ;
  gz = (float)((int16_t)(((uint16_t)rawData[4] << 8) | rawData[5])) ;
  gx *= gRes;
  gy *= gRes;
  gz *= gRes;
}


int16_t cImu::readTempData(void)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  i2cBlockRead(TEMP_OUT_H, rawData, 2);
  return ((int16_t)((uint16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a 16-bit value
}

void cImu::resetMPU9250(void)
{
  // reset device
  i2cByteWrite( PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
}



void cImu::initMPU9250(void)
{
  // Initialize MPU9150 device
  // wake up device
  i2cByteWrite( PWR_MGMT_1, 0x00);	 // Clear OS:sleep mode bit (6), enable all sensors
  delay(100);// Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

  // get stable time source
  i2cByteWrite( PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  i2cByteWrite( CONFIG, 0x03);
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  i2cByteWrite( SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c;
  c = i2cByteRead( GYRO_CONFIG);
  i2cByteWrite( GYRO_CONFIG, c & ~0xE0);  // Clear self-test bits [7:5]
  i2cByteWrite( GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  Gscale = GFS_250DPS;
  i2cByteWrite( GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
  getGres();

 // Set accelerometer configuration
  c = i2cByteRead( ACCEL_CONFIG);
  i2cByteWrite( ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  i2cByteWrite( ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  Ascale = AFS_2G;
  i2cByteWrite( ACCEL_CONFIG, c | Ascale << 3);  // Set full scale range for the accelerometer
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting


  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = i2cByteRead( ACCEL_CONFIG2);
  i2cByteWrite( ACCEL_CONFIG2, c & ~0x0F);  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  i2cByteWrite( ACCEL_CONFIG2, c | ~0x03);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  i2cByteWrite( INT_PIN_CFG, 0x22);
  i2cByteWrite( INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void cImu::calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
// reset device, reset all registers, clear gyro and accelerometer bias registers
  i2cByteWrite( PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  i2cByteWrite( PWR_MGMT_1, 0x01);
  i2cByteWrite( PWR_MGMT_2, 0x00);
  delay(200);

// Configure device for bias calculation
  i2cByteWrite( INT_ENABLE, 0x00);  // Disable all interrupts
  i2cByteWrite( FIFO_EN,0x00 );  // Disable FIFO
  i2cByteWrite(PWR_MGMT_1, 0x00);  // Turn on internal clock source
  i2cByteWrite( I2C_MST_CTRL, 0x00); // Disable I2C master
  i2cByteWrite( USER_CTRL, 0x00); // Disable FIFO and I2C master modes
  i2cByteWrite( USER_CTRL, 0x0C); // Reset FIFO and DMP
  delay(15);

// Configure MPU9150 gyro and accelerometer for bias calculation
  i2cByteWrite( CONFIG, 0x01);  // Set low-pass filter to 188 Hz
  i2cByteWrite( SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  i2cByteWrite( GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  i2cByteWrite( ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  i2cByteWrite( USER_CTRL, 0x40);  // Enable FIFO
  i2cByteWrite( FIFO_EN, 0x78);  // Enable gyro and accelerometer sensors for FIFO (max size 1024 bytes in MPU9150)
  delay(40);  // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  i2cByteWrite( FIFO_EN, 0x00);   // Disable gyro and accelerometer sensors for FIFO
  i2cBlockRead(FIFO_COUNTH, data, 2);  // read FIFO sample count
  //i2cReadyFlag.wait();
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    i2cBlockRead(FIFO_R_W, data, 12);  // read data for averaging
    //i2cReadyFlag.wait();
    accel_temp[0] = (int16_t) (((uint16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((uint16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((uint16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((uint16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((uint16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((uint16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }

  accel_bias[0] /= packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= packet_count;
  accel_bias[2] /= packet_count;
  gyro_bias[0]  /= packet_count;
  gyro_bias[1]  /= packet_count;
  gyro_bias[2]  /= packet_count;

  if(accel_bias[2] > 0L) accel_bias[2] -= (int32_t) accelsensitivity;  // Remove gravity from the z-axis accelerometer bias calculation
  else                   accel_bias[2] += (int32_t) accelsensitivity;

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] >> 10); // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] >>2); // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] >> 10);
  data[3] = (-gyro_bias[1] >> 2);
  data[4] = (-gyro_bias[2] >> 10);
  data[5] = (-gyro_bias[2] >> 2);

  dest1[0] = (float) gyro_bias[0]/gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/gyrosensitivity;




// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  i2cBlockRead(XA_OFFSET_H, data, 2);
  accel_bias_reg[0] = (int16_t) ((uint16_t)data[0] << 8) | data[1];
  i2cBlockRead(YA_OFFSET_H, data, 2);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  i2cBlockRead(ZA_OFFSET_H, data, 2);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++)
  {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  


  data[0] = (accel_bias_reg[0] >> 8);
  data[1] = (accel_bias_reg[0]);
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8);
  data[3] = (accel_bias_reg[1]);
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8);
  data[5] = (accel_bias_reg[2]);
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/accelsensitivity;
   dest2[1] = (float)accel_bias[1]/accelsensitivity;
   dest2[2] = (float)accel_bias[2]/accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void cImu::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4] = {0, 0, 0, 0};
   uint8_t selfTest[6];
   float factoryTrim[6];
   // Configure the accelerometer for self-test
   i2cByteWrite( ACCEL_CONFIG, 0xF0);  // Enable self test on all three axes and set accelerometer range to +/- 8 g
   i2cByteWrite( GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(250);  // Delay a while to let the device execute the self-test

   rawData[0] = i2cByteRead( SELF_TEST_X_ACCEL);
   rawData[1] = i2cByteRead( SELF_TEST_Y_ACCEL); // Y-axis self-test results
   rawData[2] = i2cByteRead( SELF_TEST_Z_ACCEL); // Z-axis self-test results
   rawData[3] = i2cByteRead( SELF_TEST_A);       // Mixed-axis self-test results
   
   //uint8_t reg = SELF_TEST_X_ACCEL;
   //i2c_transfer7(I2C1, MPU9250_ADDRESS, &reg, 0, rawData, 4);  


   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer

   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));             // FT[Zg] factory trim calculation


 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }

}

float cImu::invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void cImu::quater2euler(void)
{
  yaw   = atan2(2 * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
  pitch = -asin(2 * (q2 * q4 - q1 * q3));
  roll  = atan2(2 * (q1 * q2 + q3 * q4), q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);

  pitch *= 180 / PI;
  yaw   *= 180 / PI;
  roll  *= 180 / PI;
}

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f


 void cImu::Mahony_no_mag_Update(void)
 {
     float norm;
     float vx, vy, vz;
     float ex, ey, ez;
     static float eInt1=0;  // vector to hold integral error for Mahony method
     static float eInt2=0;
     static float eInt3=0;

     // Auxiliary variables to avoid repeated arithmetic
     float q1q1 = q1 * q1;
     float q1q2 = q1 * q2;
     float q1q3 = q1 * q3;
     float q2q2 = q2 * q2;
     float q2q4 = q2 * q4;
     float q3q3 = q3 * q3;
     float q3q4 = q3 * q4;
     float q4q4 = q4 * q4;

     // Estimated direction of gravity
     vx = 2.0f * (q2q4 - q1q3);
     vy = 2.0f * (q1q2 + q3q4);
     vz = q1q1 - q2q2 - q3q3 + q4q4;

     // Error is cross product between estimated direction and measured direction of gravity
     ex = ay * vz - az * vy;
     ey = az * vx - ax * vz;
     ez = ax * vy - ay * vx;

     // accumulate integral error
     eInt1 += ex;
     eInt2 += ey;
     eInt3 += ez;

     // Apply feedback terms
     gx += Kp * ex + Ki * eInt1;
     gy += Kp * ey + Ki * eInt2;
     gz += Kp * ez + Ki * eInt3;

     // Integrate rate of change of quaternion
     q1 += (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
     q2 += (q1 * gx + q3 * gz - q4 * gy) * (0.5f * deltat);
     q3 += (q1 * gy - q2 * gz + q4 * gx) * (0.5f * deltat);
     q4 += (q1 * gz + q2 * gy - q3 * gx) * (0.5f * deltat);

     // Normalise quaternion
     norm = q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4;    // normalise quaternion
     norm = invSqrt(norm);
     q1 = q1 * norm;
     q2 = q2 * norm;
     q3 = q3 * norm;
     q4 = q4 * norm;
}
