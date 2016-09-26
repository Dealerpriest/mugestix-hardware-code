#include <Arduino.h>
#include "MPU9250.h"
#include "helpers.h"

//These libraries also needs to be included from the sketch file (.ino) in order to add their search path to the compiler
#include "i2c_t3.h"

MPU9250::MPU9250(uint8_t sclpin, uint8_t sdapin, int8_t ad0pin, bool ad0ishigh, uint8_t interruptpin, uint8_t mscale, uint8_t gscale, uint8_t ascale, uint8_t mmode){
  Mscale = mscale;
  Gscale = gscale;
  Ascale = ascale;
  Mmode = mmode;
  interruptPin = interruptpin;
  _sclPin = sclpin;
  _sdaPin = sdapin;
  _AD0Pin = ad0pin;
  _AD0isHigh = ad0ishigh;
  setAxisAlignment();
}

void MPU9250::init(uint8_t calibrationSet){
  if(_AD0Pin >= 0){
    pinMode(_AD0Pin, OUTPUT);
    digitalWrite(_AD0Pin, _AD0isHigh);
  }
  if(!_AD0isHigh){
    _accGyroAddress = MPU9250_ADDRESS;
  }else{
    _accGyroAddress = MPU9250_ALTERNATIVE_ADDRESS;
  }
  _calibrationSet = calibrationSet;
  getMres();
  getGres();
  getAres();
  i2c_init(_sclPin, _sdaPin);


}

void MPU9250::i2c_init(uint8_t sclPin, uint8_t sdaPin){
#ifdef __MK20DX256__
  if(sclPin == 29 && sdaPin == 30){
    wajr = &Wire1;
    wajr->begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  }else
#endif
  if(sclPin == 16 && sdaPin == 17){
    wajr = &Wire;
    wajr->begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  }else{
    Serial.println("Heeey! The MPU9250 library doesn't recognize that i2c pin configuration. Add it to the i2c_init function!");
  }
}

void MPU9250::whoAmI(){
  Serial.println("MPU9250 9-axis motion sensor...");
  byte c = readByte(_accGyroAddress, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
}

bool MPU9250::isConnected(){
  byte c = readByte(_accGyroAddress, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  return c == 0x71;
}

void MPU9250::initMag()
{
  // First extract the factory calibration for each magnetometer axis
  
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  _magFactoryCalibration[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  _magFactoryCalibration[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  _magFactoryCalibration[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);

  Serial.print("magFactoryCalibration values: ");
  Serial.print(_magFactoryCalibration[0]); Serial.print('\t');
  Serial.print(_magFactoryCalibration[1]); Serial.print('\t');
  Serial.print(_magFactoryCalibration[2]); Serial.print('\t');
  Serial.println();


  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void MPU9250::initAccGyro()
{

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  
 // wake up device
  writeByte(_accGyroAddress, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(_accGyroAddress, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  // writeByte(_accGyroAddress, MAIN_CONFIG, 0x03); 

  //Setting dlpf_cfg ([2:0]) to 3 instead. That is a bandwidth of 184Hz and a Freq of 1kHz for the gyroscope (and temp sensor). 
  writeByte(_accGyroAddress, MAIN_CONFIG, 0x01);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // writeByte(_accGyroAddress, SMPLRT_DIV, 0x01);  // Use a 500Hz rate; If digital filters samples at 1kHz that is (see config registers)
  // writeByte(_accGyroAddress, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
  // determined inset in MAIN_CONFIG above

  //Try to not limit the output rate instead! Why lower it to half since this is lowering rate without low pass filtering i.e no smoothing?
  writeByte(_accGyroAddress, SMPLRT_DIV, 0x00);  // Use a 1kHz rate; If digital filters samples at 1kHz that is (see config registers)
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(_accGyroAddress, GYRO_CONFIG);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(_accGyroAddress, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0] 
  writeByte(_accGyroAddress, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(_accGyroAddress, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
 // Set accelerometer full-scale range configuration
  c = readByte(_accGyroAddress, ACCEL_CONFIG);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(_accGyroAddress, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(_accGyroAddress, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(_accGyroAddress, ACCEL_CONFIG2);
  writeByte(_accGyroAddress, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  // writeByte(_accGyroAddress, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(_accGyroAddress, ACCEL_CONFIG2, c | 0x01); // Set accelerometer rate to 1 kHz and bandwidth to 460 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(_accGyroAddress, INT_PIN_CFG, 0x22);    
   writeByte(_accGyroAddress, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);

  //Set start values for filteredGyroRaw
  readGyroRaw();
  for(int i = 0; i < 3; ++i){
    filteredGyroRaw[i] = gyroRaw[i];
  }
}

void MPU9250::getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void MPU9250::getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9250::getAres() {
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

//TODO: Make these functions cascade changes to the mpu registers for proper scaling.

void MPU9250::setMscale(uint8_t mscale){ //Doesn't change the mpu registers. Don't use these functions until that is taken care of!
  Mscale = mscale;
}

void MPU9250::setGscale(uint8_t gscale){
  Gscale = gscale;

}

void MPU9250::setAscale(uint8_t ascale){
  Ascale = ascale;
}

void MPU9250::setMmode(uint8_t mmode){
  Mmode = mmode;
}

void MPU9250::setAxisAlignment(int a_x, int a_y, int a_z, int g_x, int g_y, int g_z, int m_x, int m_y, int m_z){
  accelAxisAlignment[0] = a_x;
  accelAxisAlignment[1] = a_y;
  accelAxisAlignment[2] = a_z;

  gyroAxisAlignment[0] = g_x;
  gyroAxisAlignment[1] = g_y;
  gyroAxisAlignment[2] = g_z;

  magAxisAlignment[0] = m_x;
  magAxisAlignment[1] = m_y;
  magAxisAlignment[2] = m_z;
}

bool MPU9250::newSensorDataAvailable()
{
  return readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
}

void MPU9250::readAll(){
  //We want no interrupts inbetween these. They shloud be as syncronized as possible.
  noInterrupts();
  readAccelRaw();
  readGyroRaw();
  readMagnRaw();
  interrupts();
  
  readAccel();
  readGyro();
  readMagn();
  // if(!(accelNoMotion && gyroNoMotion && magnetomNoMotion)){
  //   isMoving = true;
  // }else{
  //   isMoving = false;
  // }
  // if(!isMoving){
  //   for(int i = 0; i < 3; ++i){
  //     gyroBias[i] = filteredGyroRaw[i];

  //   }
  //   // Serial.printf("setting new gyrobias\n");
  // }
}

void MPU9250::readAccelRaw(){
  int16_t reading[3];
  uint8_t rawData[6];  // x/y/z accel register data stored here
  float   tmpValues[3];
  readBytes(_accGyroAddress, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  reading[0] = (((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  tmpValues[0] = (float)reading[0] * aRes;
  reading[1] = (((int16_t)rawData[2] << 8) | rawData[3]) ; 
  tmpValues[1] = (float)reading[1] * aRes;
  reading[2] = (((int16_t)rawData[4] << 8) | rawData[5]) ; 
  tmpValues[2] = (float)reading[2] * aRes;

  for (int i = 0; i < 3; ++i)
  {
    previousAccelRaw[i] = accelRaw[i];
    accelRaw[i] = tmpValues[abs(accelAxisAlignment[i])-1];
    if(accelAxisAlignment[i] < 0){ // Turn this axis around
      accelRaw[i] = -accelRaw[i];
    }
  }
}

// Accelerometer calibration data
                            // 1          2           3              4              5           6            7
// const float ACCEL_X_MIN[7] = {-1.02,     -1,         -1.01,         -1.00,         -1,         -1,          -0.9651913};
// const float ACCEL_X_MAX[7] = { 1.03,      1,          1.02,          1.03,          1,          1,           0.9751351};
// const float ACCEL_Y_MIN[7] = {-1.01,     -1,         -1.01,         -1.03,         -1,         -1,          -0.9720794};
// const float ACCEL_Y_MAX[7] = { 1.01,      1,          1.0,           0.98,          1,          1,           0.9933636};
// const float ACCEL_Z_MIN[7] = {-1.01,     -1,         -1.05,         -1.10,         -1,         -1,          -0.9746387};
// const float ACCEL_Z_MAX[7] = { 1.03,      1,          1.0,           0.99,          1,          1,           0.9767945};


void MPU9250::readAccel()
{
  // readAccelRaw();
  updateAccelSlope();
  // calculateAccelBias();
  // Compensate accelerometer error

  // float x_offset = ((ACCEL_X_MIN[_calibrationSet] + ACCEL_X_MAX[_calibrationSet]) / 2.0f);
  // float y_offset = ((ACCEL_Y_MIN[_calibrationSet] + ACCEL_Y_MAX[_calibrationSet]) / 2.0f);
  // float z_offset = ((ACCEL_Z_MIN[_calibrationSet] + ACCEL_Z_MAX[_calibrationSet]) / 2.0f);
  // float x_scale = (1.0 / (ACCEL_X_MAX[_calibrationSet] - x_offset));
  // float y_scale = (1.0 / (ACCEL_Y_MAX[_calibrationSet] - y_offset));
  // float z_scale = (1.0 / (ACCEL_Z_MAX[_calibrationSet] - z_offset));
  // accel[0] = (accelRaw[0] - x_offset) * x_scale;
  // accel[1] = (accelRaw[1] - y_offset) * y_scale;
  // accel[2] = (accelRaw[2] - z_offset) * z_scale;

  accel[0] = (accelRaw[0]);
  accel[1] = (accelRaw[1]);
  accel[2] = (accelRaw[2]);
}

void MPU9250::updateAccelSlope(){
  float slope = 0;
  for(int i = 0; i < 3; ++i){
    slope += abs(accelRaw[i] - previousAccelRaw[i]);
  }
  if(slope < 0.0000001f){//Comparing same sample
    return;
  }
  float newSum = accelRawSlopeSum[accelRawSlopeSumIndex] + slope;

  accelRawSlopeSumIndex++;
  accelRawSlopeSumIndex %= accelWindowSize;

  windowedAccelRawSlope = (newSum - accelRawSlopeSum[accelRawSlopeSumIndex]);
  windowedAccelRawSlope /= accelWindowSize;

  accelRawSlopeSum[accelRawSlopeSumIndex] = newSum;

  if(windowedAccelRawSlope < 0.01f){
    if(!accelNoMotion){
      Serial.printf("\t\taccelNOMotion\n");
    }
    accelNoMotion = true;
    accelSlowMotion = false;
  }else if(windowedAccelRawSlope < 0.03f){
    if(!accelSlowMotion){
      Serial.printf("\t\taccelSlowMotion\n");
    }
    accelSlowMotion = true;
    accelNoMotion = false;
  }else{
    if(accelNoMotion || accelSlowMotion){
      Serial.printf("\t\taccelMotion active\n");
    }
    accelNoMotion = false;
    accelSlowMotion = false;
  }

  // Serial.printf("windowedAccelRawSlope: %f\n", windowedAccelRawSlope);
}

void MPU9250::readGyroRaw()
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  int16_t reading[3];
  float   tmpValues[3];
  readBytes(_accGyroAddress, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  reading[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  tmpValues[0] = (float)reading[0] * gRes;
  reading[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  tmpValues[1] = (float)reading[1] * gRes;
  reading[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  tmpValues[2] = (float)reading[2] * gRes;

  for (int i = 0; i < 3; ++i)
  {
    previousGyroRaw[i] = gyroRaw[i];
    gyroRaw[i] = tmpValues[abs(gyroAxisAlignment[i])-1];
    if(gyroAxisAlignment[i] < 0){ // Turn this axis around
      gyroRaw[i] = -gyroRaw[i];
    }
  }
}

void MPU9250::updateFilteredGyroRaw(){
  for(int i = 0; i< 3; ++i){
    filteredGyroRaw[i] = (1.0 - gyroRawFilterC)*filteredGyroRaw[i] + gyroRawFilterC*gyroRaw[i];
  }
}

void MPU9250::readGyro()
{
  // readGyroRaw();
  updateFilteredGyroRaw();
  // updateGyroSlope();
  calculateGyroBias();
  // gyroRaw[0] -= gyroBias[0];
  // gyroRaw[1] -= gyroBias[1];
  // gyroRaw[2] -= gyroBias[2];

  gyro[0] = gyroRaw[0] - gyroBias[0];
  gyro[1] = gyroRaw[1] - gyroBias[1];
  gyro[2] = gyroRaw[2] - gyroBias[2];

  gyro[0] = TO_RAD(gyro[0]);
  gyro[1] = TO_RAD(gyro[1]);
  gyro[2] = TO_RAD(gyro[2]);
}

void MPU9250::updateGyroSlope(){
  float slope = 0;
  for(int i = 0; i < 3; ++i){
    slope += abs(gyroRaw[i] - previousGyroRaw[i]);
  }
  if(slope < 0.0000001f){//Comparing same sample
    return;
  }
  float newSum = gyroRawSlopeSum[gyroRawSlopeSumIndex] + slope;

  gyroRawSlopeSumIndex++;
  gyroRawSlopeSumIndex %= gyroWindowSize;

  windowedGyroRawSlope = newSum - gyroRawSlopeSum[gyroRawSlopeSumIndex];
  windowedGyroRawSlope /= gyroWindowSize;

  gyroRawSlopeSum[gyroRawSlopeSumIndex] = newSum;

  if(windowedGyroRawSlope < 0.37f){
    // if(!gyroNoMotion){
    //   Serial.printf("\t\tgyroNoMotion triggered \n");
    // }
    gyroNoMotion = true;
  }else{
    gyroNoMotion = false;
  }

  // Serial.printf("windowedGyroRawSlope: %f\n", windowedGyroRawSlope);
}

// float newMagnetomRaw[3] = {0};
// const float magnetomRawC = 0.5;
void MPU9250::readMagnRaw()
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  int16_t reading[3];
  float   tmpValues[3];
  uint8_t c = readByte(AK8963_ADDRESS, AK8963_ST1);
  if(c & 0x01) { // wait for magnetometer data ready bit to be set
    magDataOverrun = c & 0x02; // Check dataoverrun status register bit
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      reading[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      tmpValues[0] = (float)reading[0] * mRes*_magFactoryCalibration[0];
      reading[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      tmpValues[1] = (float)reading[1] * mRes*_magFactoryCalibration[1];
      reading[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
      tmpValues[2] = (float)reading[2] * mRes*_magFactoryCalibration[2];


      //Apply axisAlignment (and filter)
      for (int i = 0; i < 3; ++i)
      {
        previousMagnetomRaw[i] = magnetomRaw[i];
        magnetomRaw[i] = tmpValues[abs(magAxisAlignment[i])-1];
        if(magAxisAlignment[i] < 0){ // Turn this axis around
          magnetomRaw[i] = -magnetomRaw[i];
        }

        // magnetomRaw[i] = newMagnetomRaw[i];
        // magnetomRaw[i] = (1.0-magnetomRawC) * magnetomRaw[i] + magnetomRawC * newMagnetomRaw[i];
      }
    }else{
      Serial.println("magnetometer overflow");
    }
  }
}

void MPU9250::updateFilteredMagnetomRaw(){
  for(int i = 0; i < 3; ++i){
    previousFilteredMagnetomRaw[i] = filteredMagnetomRaw[i];
    filteredMagnetomRaw[i] = (1.0-magnetomRawFilterC) * filteredMagnetomRaw[i] + magnetomRawFilterC * magnetomRaw[i];
  }
}

//Compare calibration at different battery levels (node 5)
// Low level:
// const float magn_ellipsoid_center[3] = {-124.579, 227.323, 273.862};
// const float magn_ellipsoid_transform[3][3] = {{0.980209, 0.00700484, -0.00120823}, {0.00700484, 0.982384, -0.0168318}, {-0.00120823, -0.0168318, 0.980246}};
// Fully charged:
// const float magn_ellipsoid_center[3] = {-126.149, 221.195, 268.192};
// const float magn_ellipsoid_transform[3][3] = {{0.981727, 0.00596070, -0.00240183}, {0.00596070, 0.982099, -0.0153305}, {-0.00240183, -0.0153305, 0.983411}};
// Different placing, fully charged:
// const float magn_ellipsoid_center[3] = {-127.238, 231.330, 275.658};
// const float magn_ellipsoid_transform[3][3] = {{0.981154, 0.0101629, -0.000481403}, {0.0101629, 0.974322, -0.0190969}, {-0.000481403, -0.0190969, 0.981437}};
// Giving sensor time to warm up:
// const float magn_ellipsoid_center[3] = {-127.161, 234.267, 276.691};
// const float magn_ellipsoid_transform[3][3] = {{0.982393, 0.00596086, -0.00204437}, {0.00596086, 0.977090, -0.0189226}, {-0.00204437, -0.0189226, 0.981347}};
// With 8Mhz update rate:
// const float magn_ellipsoid_center[3] = {-125.701, 235.063, 274.796};
// const float magn_ellipsoid_transform[3][3] = {{0.979081, 0.00746180, -0.00178819}, {0.00746180, 0.978915, -0.0178605}, {-0.00178819, -0.0178605, 0.981274}};
// With filter 0.5
// const float magn_ellipsoid_center[3] = {-125.877, 232.295, 275.163};
// const float magn_ellipsoid_transform[3][3] = {{0.978041, 0.00754466, -0.00266392}, {0.00754466, 0.976280, -0.0164312}, {-0.00266392, -0.0164312, 0.985435}};

// Manual magcal:
// -127.649994, 233.65, 278.299



const bool useExtendedMagCalibration = true;
const float magn_ellipsoid_center[7][3] = {
  {-16.8863, 399.016, 142.907},
  {80.2388, 481.809, 289.632},
  {-148.317, 280.627, -161.916},
  {77.9503, 500.102, 434.564},

  {-125.918, 234.557, 274.702},
  {41.7497, 236.698, -274.512},
  {165.606, 505.876, 326.720}
};
const float magn_ellipsoid_transform[7][3][3] = {
  {{0.994513, 0.00502762, -0.00143121}, {0.00502762, 0.971615, -0.0174256}, {-0.00143121, -0.0174256, 0.984863}},
  {{0.981769, 0.0117962, 0.00202186}, {0.0117962, 0.991297, 0.00452319}, {0.00202186, 0.00452319, 0.968011}},
  {{0.955293, 0.00458540, -0.0103671}, {0.00458540, 0.949781, 0.0187842}, {-0.0103671, 0.0187842, 0.991284}},
  {{0.887482, 0.000966477, 0.0222864}, {0.000966477, 0.930653, -0.0126360}, {0.0222864, -0.0126360, 0.993352}},
  
  {{0.981210, 0.00817758, -0.00151712}, {0.00817758, 0.980527, -0.0176514}, {-0.00151712, -0.0176514, 0.978808}},
  {{0.991386, 0.00849474, 0.00630473}, {0.00849474, 0.989039, -0.000717012}, {0.00630473, -0.000717012, 0.983676}},
  {{0.885493, 0.00104034, 0.0238128}, {0.00104034, 0.881532, 0.000594332}, {0.0238128, 0.000594332, 0.995042}}
};

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

bool  magMinSetOnAxis[3] = {false, false, false};
bool  magMaxSetOnAxis[3] = {false, false, false};
bool  magBiasSetOnAxis[3] = {false, false, false};
void MPU9250::readMagn()
{
  // readMagnRaw();
  // calculateMagBias();

  
  

  //apply calibration data and save to output array (magnetom)
  if(useExtendedMagCalibration){
    float magnetom_tmp[3];
    for (int i = 0; i < 3; i++){
      previousMagnetom[i] = magnetom[i];
      // if(magBiasSetOnAxis[i]){ // we might getter min/max values during run-time. If so, use them!
      //   magnetom_tmp[i] = magnetomRaw[i] - magBias[i];
      // }else{
        magnetom_tmp[i] = magnetomRaw[i] - magn_ellipsoid_center[_calibrationSet][i];
      // }
    }

    // //Gunnar version with autoupdated bias
    // applyMagBias();
    // for (int i = 0; i < 3; i++){
    //   magnetom_tmp[i] = magnetom[i];
    // }


    matrix_Vector_Multiply(magn_ellipsoid_transform[_calibrationSet], magnetom_tmp, magnetom);
  }else{
    calculateMagBias();
    applyMagBias();
  }

  // updateMagnetomSlope();

  //Low passed update of field strength
  // float filterConstant = 0.999;
  // magFieldStrength = filterConstant * magFieldStrength + (1 - filterConstant) * sqrt(magnetom[0]*magnetom[0] + magnetom[1]*magnetom[1] + magnetom[2]*magnetom[2]);

  
}

void MPU9250::updateMagnetomSlope(){
  updateFilteredMagnetomRaw();
  // if(abs(filteredMagnetomRaw[0] - previousFilteredMagnetomRaw[0]) < 0.00000001f){//comparing same sample
  //   return;
  // }
  // float slope = TO_DEG(vec_angle(filteredMagnetomRaw, previousFilteredMagnetomRaw));
  float slope = 0;
  for(int i = 0; i < 3; ++i){
    slope += abs(filteredMagnetomRaw[i] - previousFilteredMagnetomRaw[i]);
  }
  if(slope < 0.00000001f){//comparing same sample
    return;
  }
  
  float newSum = magnetomRawSlopeSum[magnetomRawSlopeSumIndex] + slope;

  magnetomRawSlopeSumIndex++;
  magnetomRawSlopeSumIndex %= magnetomWindowSize;

  windowedMagnetomRawSlope = newSum - magnetomRawSlopeSum[magnetomRawSlopeSumIndex];
  // windowedMagnetomRawSlope /= magnetomWindowSize;

  magnetomRawSlopeSum[magnetomRawSlopeSumIndex] = newSum;

  if(windowedMagnetomRawSlope < 31.0f){
    // if(!magnetomNoMotion){
    //   Serial.printf("\t\tmagnetomNoMotion triggered \n");
    // }
    magnetomNoMotion = true;
  }else{
    magnetomNoMotion = false;
  }

  // Serial.printf("magnetom slope: %f,    ", slope);
  // Serial.printf("windowedMagnetomRawSlope: %f\n", windowedMagnetomRawSlope);
}

//Accel run-time calibration
    // float previousAccelRaw[3] = {0};
    // float filteredAccelLength = 0;
    // float previousFilteredAccelLength = 0;
    // float accelLengthFilter = 0.5;
    // float accelLength = 0;
    // float filteredAccelLengthDelta = 0;
    // float accelLengthDeltaFilter = 0.01;

// bool MPU9250::calculateAccelBias(){
//   filteredAccelLength = (1.0 - accelLengthFilter)* filteredAccelLength + accelLengthFilter * vec_length(accelRaw);
//   Serial.print("filteredAccelLength is: ");
//   Serial.print(filteredAccelLength);

//   float delta = abs(filteredAccelLength - previousFilteredAccelLength);
//   filteredAccelLengthDelta = (1.0 - accelLengthDeltaFilter)*filteredAccelLengthDelta + accelLengthDeltaFilter*delta;
//   Serial.print("\tfilteredAccelLengthDelta is: ");
//   Serial.println(filteredAccelLengthDelta*100);
//   if(filteredAccelLengthDelta*100 < 0.30){
//     Serial.println("accelVector ready for bias");
//   }

//   // for(int i = 0; i < 3; ++i){
    
//   // }

//   previousFilteredAccelLength = filteredAccelLength;
// }

//TODO: Prova att inte ha abs på delta. Då får jag slopesamplingar med tecken. Kanske noiset tar ut varann om jag summerar dom. Sen abs på summan och kolla mot threshold
int nrOfGyroSamplesBelowThreshold = 0;
const int gyroSampleThreshold = 30000;
const float slopeThreshold = 0.75;
float gyroSlope[3];
// const float gyroDeltaFilter = 0.005;
// const float gyroRawFilter = 0.001;
const float gyroRawFilter = 0.1f;
bool MPU9250::calculateGyroBias()
{
  float deltaGyroRaw[3] = {0,0,0};
  for(int i = 0; i< 3; ++i){
    deltaGyroRaw[i] = abs(gyroRaw[i] - previousGyroRaw[i]);
    previousGyroRaw[i] = gyroRaw[i];
  }
  //Check if we're comparing same sample
  if(deltaGyroRaw[0] + deltaGyroRaw[1] + deltaGyroRaw[2] < 0.0000001f){
    return false;
  }
  float biggestAxis = max(deltaGyroRaw[0], max(deltaGyroRaw[1], deltaGyroRaw[2]));
  // Serial.printf("deltaGyroRaw: %f, %f, %f \n", deltaGyroRaw[0], deltaGyroRaw[1], deltaGyroRaw[2]);
  if(biggestAxis < slopeThreshold){
    nrOfGyroSamplesBelowThreshold++;
    // Serial.printf("incrementing gyrothresholdcounter\n");
  }else{
    // Serial.printf("incrementing of gyrothresholdcounter stopped at n: %i \n", nrOfGyroSamplesBelowThreshold);
    nrOfGyroSamplesBelowThreshold = 0;
  }

  if(nrOfGyroSamplesBelowThreshold > gyroSampleThreshold){
    // Serial.print("Setting new gyroBias "); Serial.print(millis());
    // for(int i = 0; i<3; ++i){
    //   gyroBias[i] = filteredGyroRaw[i];
    //   // Serial.printf("\t gyroBias[%i] = %f", i, gyroBias[i]);
    // }

    // Serial.printf("gyro no motion triggered\n");
    isMoving = false;
    
    // Serial.println();
  }else{
    isMoving = true;
  }

  // float deltaGyroRaw = 0;
  // for(int i = 0; i< 3; ++i){
  //   filteredGyroRaw[i] = (1.0 - gyroRawFilter)*filteredGyroRaw[i] + gyroRawFilter*gyroRaw[i];
  //   deltaGyroRaw += abs(previousGyroRaw[i] - gyroRaw[i]);
  //   previousGyroRaw[i] = gyroRaw[i];
  // }
  // filteredTotalDeltaGyroRaw = (1.0 - gyroDeltaFilter)*filteredTotalDeltaGyroRaw + gyroDeltaFilter*deltaGyroRaw;
  // Serial.print("filteredTotalDeltaGyroRaw is: ");
  // Serial.print(filteredTotalDeltaGyroRaw);
  // Serial.print("\tfilteredGyroRaw is: ");
  // Serial.print(filteredGyroRaw[0]);
  // Serial.print(", ");
  // Serial.print(filteredGyroRaw[1]);
  // Serial.print(", ");
  // Serial.println(filteredGyroRaw[2]);
  // if(filteredTotalDeltaGyroRaw < 0.15){
  //   Serial.print("Setting new gyroBias "); Serial.print(millis());
  //   for(int i = 0; i< 3; ++i){
  //     Serial.print(",   ");
  //     Serial.print(filteredGyroRaw[i]);
  //     gyroBias[i] = filteredGyroRaw[i];
  //   }
  //   Serial.println();
  //   return true;
  // }
  // return false;
}

float filteredMagFieldStrength = 650.0;
const float magFieldStrengthFilterC = 0.0006;
float currentMagFieldStrength;
float magScale[3];
bool MPU9250::calculateMagBias()
{
  currentMagFieldStrength = vec_length(magnetom);
  filteredMagFieldStrength = (1.0 - magFieldStrengthFilterC) * filteredMagFieldStrength + magFieldStrengthFilterC * currentMagFieldStrength;
  // Serial.printf("filteredFieldStrength is %f \n", filteredMagFieldStrength);
  float biggestAxisValue = 0;
  int prominentAxis = 0;
  int otherAxis[2] = {1,2};

  bool result = false;
  for (int axis = 0; axis < 3; ++axis)
  {
    //Finding prominent axis
    if(abs(magnetom[axis]) > biggestAxisValue){
      biggestAxisValue = abs(magnetom[axis]);
      prominentAxis = axis;
    }
  }
  //Find the other axis's
  otherAxis[0] = (prominentAxis+1)%3;
  otherAxis[1] = (prominentAxis+2)%3;


  //Are we clos to the axis?
  // if(abs(magnetom[otherAxis[0]]) < 50 && abs(magnetom[otherAxis[1]]) < 50)
  {
    //Update max and min if increased
    if(magnetomRaw[prominentAxis] > magMax[prominentAxis])
    {
      magMax[prominentAxis] = magnetomRaw[prominentAxis];
      magMaxSetOnAxis[prominentAxis] = true;
      result = true;
    }
    else if(magnetomRaw[prominentAxis] < magMin[prominentAxis])
    {
      magMin[prominentAxis] = magnetomRaw[prominentAxis];
      magMinSetOnAxis[prominentAxis] = true;
      result = true;
    }
  }

  //almost impossible that this would yield a negative value, so I'm skipping abs().
  float distance = (magMax[prominentAxis] - magMin[prominentAxis])/2;

  

  if(distance > filteredMagFieldStrength && result && magMaxSetOnAxis[prominentAxis] && magMinSetOnAxis[prominentAxis])
  {
    // magScale[axis] = 1.0/distance;
    // Serial.printf("axis %i, magscale = %f \t", axis, magScale[axis]);
    magBias[prominentAxis] = (magMax[prominentAxis] + magMin[prominentAxis])/2;
    magBiasSetOnAxis[prominentAxis] = true;
    // Serial.printf("setting magBias[%i] to %f, precalibrated bias is %f \n", prominentAxis, magBias[prominentAxis], magn_ellipsoid_center[_calibrationSet][prominentAxis]);
  }




  // if (currentMagFieldStrength < filteredMagFieldStrength*1.2)//Is the max and min bigger than vector length? Constrain the max and min to strength of field
  // {

  //   float scaleFactor = 0.05;
  //   if(magnetom[prominentAxis] > 0){
  //     magMax[prominentAxis] -= scaleFactor;
  //     Serial.printf("lowering magMax[%i] to %f \t", prominentAxis, magMax[prominentAxis]);
  //   }else{
  //     magMin[prominentAxis] += scaleFactor;
  //     Serial.printf("raising magMin[%i] to %f \t", prominentAxis, magMin[prominentAxis]);
  //   }
  //   //recalculate magScale
  //   // distance = (magMax[axis] - magMin[axis])/2;
  //   result = true;
    
  // }

  // Serial.println();

  // digitalWrite(13, result);
  
  return result;
}

void MPU9250::applyMagBias(){
  for(int axis = 0; axis < 3; ++axis){
    magnetom[axis] = magnetomRaw[axis] - magBias[axis];
    // magnetom[axis] = magnetom[axis]*magScale[axis];
  }
}

bool MPU9250::isCurrentlyMoving(){
  return isMoving;
}

void MPU9250::printMagAccelAngle(){
  float angle = vec_angle(accel, magnetom);
  Serial.printf("magAccelAngle : %f \n", TO_DEG(angle));
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::accGyroSelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(_accGyroAddress, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(_accGyroAddress, MAIN_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(_accGyroAddress, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(_accGyroAddress, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(_accGyroAddress, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(_accGyroAddress, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(_accGyroAddress, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(_accGyroAddress, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(_accGyroAddress, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(_accGyroAddress, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(_accGyroAddress, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(_accGyroAddress, ACCEL_CONFIG, 0x00);  
   writeByte(_accGyroAddress, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(_accGyroAddress, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(_accGyroAddress, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(_accGyroAddress, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(_accGyroAddress, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(_accGyroAddress, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(_accGyroAddress, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }
   
}

        void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  noInterrupts();
  wajr->beginTransmission(address);  // Initialize the Tx buffer
  wajr->write(subAddress);           // Put slave register address in Tx buffer
  wajr->write(data);                 // Put data in Tx buffer
  wajr->endTransmission();           // Send the Tx buffer
  interrupts();
}

       uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data  
  noInterrupts(); 
  wajr->beginTransmission(address);         // Initialize the Tx buffer
  wajr->write(subAddress);                   // Put slave register address in Tx buffer
  wajr->endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
  wajr->requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = wajr->read();                      // Fill Rx buffer with result
  interrupts();
  return data;                             // Return data read from slave register
}

        void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  noInterrupts();
  wajr->beginTransmission(address);   // Initialize the Tx buffer
  wajr->write(subAddress);            // Put slave register address in Tx buffer
  wajr->endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//  wajr->endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
//        wajr->requestFrom(address, count);  // Read bytes from slave register address 
        wajr->requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (wajr->available()) {
        dest[i++] = wajr->read(); }         // Put read results in the Rx buffer
  interrupts();
}