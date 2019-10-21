/*
 * Author: Fawadul Haq
 * Last Modified: 10/21/19
 * 
 * This code is written for use with the GY521 Accelerometer/Gyroscope chip. 
 * It tests for an acceleration (in units of g's) greater than a specified threshold.
 * 
 * Code adapted from Dejan of https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 */

#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
const int Sens16g = 0x18;  // Set AFS_SEL bits to this for +/- 16g sensitivity
float accSens = 16384.0;  //  The LSB/g sensitivity of the +/- 16g setting
float gyroSens = 131.0; // The LSB/deg/s sensitivity for +/- 250 deg/sec

float AccX, AccY, AccZ;   // Raw values
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Derived values
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; // Calibration parameters (natural error)
float AccErrorXOrig, AccErrorYOrig, AccErrorZOrig; 
float stdAccX, stdAccY, stdGyroX, stdGyroY, stdGyroZ;

int errorN = 100;         // The number of samples to take to find error
float errorMargin = 1;    // The number of standard deviations off of the avg error 
                          // the reading has to be in order to be considered real


float elapsedTime, currentTime, previousTime;    // Used to calculate the degree
float roll, pitch, yaw;

// LED I/O
const int RED = 9;
const int YELLOW = 10;
const int GREEN = 11;


float alertThreshold = 1.4;

float prevGyroX = 0.0; // Compare the last reading with the current one
float prevGyroY = 0.0;
float prevGyroZ = 0.0;

void setup() {
  Serial.begin(9600);

  // LED I/O
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  digitalWrite(RED, 0);

  // 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);             // Initialize communication with the 6B register of the chip
  Wire.write(0x00);             // Reset values in 0x6B register
  Wire.endTransmission(true);   

//  // Change sensitivity of accelerometer to +/- 16g
//  Wire.beginTransmission(MPU);
//  Wire.write(0x1C);             // Begin comm. with ACCEL_CONFIG register
//  Wire.write(Sens16g); 
//  Wire.endTransmission(true);
  
  // Keep sensitivity of gyroscope at default +/- 250deg/s

  calculate_IMU_error();
  delay(20);
}

void loop() {

  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  AccX = (Wire.read() << 8 | Wire.read()); // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()); // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()); // Z-axis value

  Serial.print("AccX: "); Serial.println(AccX / accSens);
  Serial.print("AccY: "); Serial.println(AccY / accSens);
  Serial.print("AccZ: "); Serial.println(AccZ / accSens);
  Serial.println();

  
  // Check for error margin (in gs)
  //For a sensitivity of +/-2g, we need to divide the raw values by accSens, according to the datasheet
  if(AccX / accSens > alertThreshold || AccX / accSens < -alertThreshold)
    alert();
  if(AccY / accSens > alertThreshold || AccY / accSens < -alertThreshold)
    alert();
  if(AccZ / accSens > alertThreshold || AccZ / accSens < -alertThreshold)
    alert();

  delay(1000);  

  
}

// Finds the natural errors of the accelerometer and gyroscope
// Chip should be still in the beginning
void calculate_IMU_error(){

  int c = 0;
  float samplesX[errorN];
  float samplesY[errorN];
  float samplesZ[errorN];
  
  while(c < errorN) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    // Extract raw values
    AccX = (Wire.read() << 8 | Wire.read()); 
    AccY = (Wire.read() << 8 | Wire.read());
    AccZ = (Wire.read() << 8 | Wire.read()); 

    // Error storage for std deviation (in LSB/g)
    samplesX[c] = AccX;
    samplesY[c] = AccY;

    // Sum all readings (LSB/g)
    AccErrorXOrig = AccX + AccErrorXOrig; // Raw error values
    AccErrorYOrig = AccY + AccErrorYOrig;
    AccErrorZOrig = AccZ + AccErrorZOrig;

    // Adjust for sensitivity (in Gs)
    AccX /= accSens;
    AccY /= accSens;
    AccZ /= accSens;
    
    // Calculate Acceleration Angle Errors
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    
    c++;
  }

  //Divide the sum by the number of samples to get the error value
  AccErrorXOrig = AccErrorXOrig/errorN;
  AccErrorYOrig = AccErrorYOrig/errorN;
  AccErrorZOrig = AccErrorZOrig/errorN;
  AccErrorX = AccErrorX / errorN;
  AccErrorY = AccErrorY / errorN;
  
  stdAccX = calculate_IMU_std(samplesX, AccErrorXOrig);
  stdAccY = calculate_IMU_std(samplesY, AccErrorYOrig);

  Serial.print("stdAccX: "); Serial.println(stdAccX);
  Serial.print("stdAccY: "); Serial.println(stdAccY);
  
  c = 0;
 
  // Reading gyro values
  while (c < errorN) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / gyroSens);
    GyroErrorY = GyroErrorY + (GyroY / gyroSens);
    GyroErrorZ = GyroErrorZ + (GyroZ / gyroSens);

    samplesX[c] = GyroErrorX;
    samplesY[c] = GyroErrorY;
    samplesZ[c] = GyroErrorZ;
    
    c++;
  }

  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / errorN;
  GyroErrorY = GyroErrorY / errorN;
  GyroErrorZ = GyroErrorZ / errorN;

  stdGyroX = calculate_IMU_std(samplesX, GyroErrorX);
  stdGyroY = calculate_IMU_std(samplesY, GyroErrorY);
  stdGyroZ = calculate_IMU_std(samplesZ, GyroErrorZ);

  Serial.print("stdGyroX: "); Serial.println(stdGyroX);
  Serial.print("stdGyroY: "); Serial.println(stdGyroY);
  Serial.print("stdGyroZ: "); Serial.println(stdGyroZ);
  
  
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorXRaw: ");
  Serial.println(AccErrorXOrig);
  Serial.print("AccErrorYRaw: ");
  Serial.println(AccErrorYOrig);
  Serial.print("AccErrorZRaw: ");
  Serial.println(AccErrorZOrig);
  Serial.print("GyroErrorXRaw: ");
  Serial.println(GyroErrorX * gyroSens);
  Serial.print("GyroErrorYRaw: ");
  Serial.println(GyroErrorY * gyroSens);
  Serial.print("GyroErrorZRaw: ");
  Serial.println(GyroErrorZ * gyroSens);
  
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  
}

float calculate_IMU_std(float samples[], float mean){
  float ans;

  float sum = 0;
  for(int i = 0; i < errorN; i++){
    sum = pow(samples[i] - mean, 2) + sum;
  }

  ans = sqrt(sum/errorN);
  return ans;

}

// LED Alert
void alert(){

  Serial.println("THIS IS AN ALERT");
  digitalWrite(RED, 1);
  delay(2000);
  digitalWrite(RED, 0);
 
}
