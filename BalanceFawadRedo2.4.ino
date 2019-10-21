/*
 * Author: Fawadul Haq
 * Last Modified: 10/21/19
 * 
 * This code is written for use with the GY521 Accelerometer/Gyroscope chip. 
 * It tests for the AP sway 
 * 
 * Code adapted from Dejan of https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 */

#include <Wire.h>
#include <MPU6050.h>

const int MPU = 0x68; // MPU6050 I2C address
const int Sens16g = 0x18;  // Set AFS_SEL bits to this for +/- 16g sensitivity
float accSens = 2048.0;  //  The LSB/g sensitivity of the +/- 16g setting
float gyroSens = 131.0; // The LSB/deg/s sensitivity for +/- 250 deg/sec

float AccX, AccY, AccZ;   // Raw values
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Derived values
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; // Calibration parameters (natural error)
float AccErrorXOrig, AccErrorYOrig, AccErrorZOrig; 
float stdAccX, stdAccY, stdGyroX, stdGyroY, stdGyroZ;

float maxPGyroX, maxPGyroY, maxPGyroZ, maxNGyroX, maxNGyroY, maxNGyroZ = 0.0;

int errorN = 100;         // The number of samples to take to find error
float errorMargin = 3.0;  // The number of standard deviations off of the avg error 
                          // the reading has to be in order to be considered real


float elapsedTime, currentTime, previousTime;                   // Used to calculate the degree

MPU6050 myMPU;  // MPU Library class for the chip

void setup() {
  Serial.begin(19200);
  delay(200);
  Serial.println("Setup...");

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);             // Initialize communication with the 6B register of the chip
  Wire.write(0x00);             // Reset values in 0x6B register
  Wire.endTransmission(true);   

  // IC2Dev Library
  myMPU = MPU6050(MPU);

  // Keep sensitivity of accelerometer at +/- 2g
  
  // Keep sensitivity of gyroscope at default +/- 250deg/s
  
  delay(200);

  calculate_IMU_error();
}

void loop() {

  Serial.println("in main loop... \n");

  myMPU.resetSensors(); // Start testing from zero

  float prevGyroX = 0.0; // Compare the last reading with the current one
  float prevGyroY = 0.0;
  float prevGyroZ = 0.0;
  
  float beginTest = millis();

  int testNum = 1;
  Serial.print("Test "); Serial.println(testNum);

  int count = 1;
  while(millis() - beginTest < 20000){  // Test is 20 seconds long

    // Re-initialization
    maxPGyroX, maxPGyroY, maxPGyroZ, maxNGyroX, maxNGyroY, maxNGyroZ = 0.0;
    
    // Reading Gyro input (deg/s)
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    if(count % 1000 == 0){ // output every 1000 iterations
      Serial.print("GyroX: "); Serial.println(GyroX);
      Serial.print("GyroY: "); Serial.println(GyroY);
      Serial.print("GyroZ: "); Serial.println(GyroZ);
      Serial.print("prevGyroX: "); Serial.println(prevGyroX);
      Serial.print("prevGyroY: "); Serial.println(prevGyroY);
      Serial.print("prevGyroZ: "); Serial.println(prevGyroZ);
      Serial.println();
    }
    

    // Checking for peaks
    // X-Axis
    if( abs(GyroX - prevGyroX) > abs(errorMargin*stdGyroX) ){
      // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by senconds (s) to get the angle in degrees
      gyroAngleX = gyroAngleX + (GyroX * elapsedTime) / gyroSens; // deg/s * s = deg

      if(count % 500 == 0){
        Serial.println("ChangedX");
      }
      
      // Positive peak
      if(gyroAngleX > 0 && gyroAngleX > maxPGyroX) maxPGyroX = gyroAngleX;
      // Negative Peak
      if(gyroAngleX < 0 && gyroAngleX < maxNGyroX) maxNGyroX = gyroAngleX;

    } 
    prevGyroX = GyroX;

    // Y-axis
    if( abs(GyroY - prevGyroY) > abs(errorMargin*stdGyroY) ){
      gyroAngleY = gyroAngleY + (GyroY * elapsedTime) / gyroSens;

      
      if(count % 500 == 0){
        Serial.println("ChangedY");
      }
      
      if(gyroAngleY > 0 && gyroAngleY > maxPGyroY) maxPGyroY = gyroAngleY;

      if(gyroAngleY < 0 && gyroAngleY < maxNGyroY) maxNGyroY = gyroAngleY;
    
    }
    prevGyroY = GyroY;
    
    // Z-axis
    if( abs(GyroZ - prevGyroZ) > abs(errorMargin*stdGyroZ) ){
      gyroAngleZ = gyroAngleZ + (GyroZ * elapsedTime) / gyroSens;

      if(count % 500 == 0){
        Serial.println("ChangedZ");
      }
      
      if(gyroAngleZ > 0 && gyroAngleZ > maxPGyroZ) maxPGyroZ = gyroAngleZ;

      if(gyroAngleZ < 0 && gyroAngleZ < maxNGyroZ) maxNGyroZ = gyroAngleZ; 

    }
    prevGyroZ = GyroZ;

    count++;
  } // end of Test

  // Plug into eq score equation
  Serial.println("Axis: Max Neg Ang, Max Pos Angle"); 
  Serial.print("X: "); Serial.print(maxPGyroX); Serial.print(" , "); Serial.println(maxNGyroX);
  Serial.print("Y: "); Serial.print(maxPGyroY); Serial.print(" , "); Serial.println(maxNGyroY);
  Serial.print("Z: "); Serial.print(maxPGyroZ); Serial.print(" , "); Serial.println(maxNGyroZ);
  Serial.println();

  
  float X_eq = eq_score(maxPGyroX, maxNGyroX);
  float Y_eq = eq_score(maxPGyroY, maxNGyroY);
  float Z_eq = eq_score(maxPGyroZ, maxNGyroZ);

  Serial.print("X_eq: "); Serial.println(X_eq);
  Serial.print("Y_eq: "); Serial.println(Y_eq);
  Serial.print("Z_eq: "); Serial.println(Z_eq);
  Serial.println();

  testNum++;
} // end of main loop

// Finds the natural drift of the gyroscope
// Chip should be still in the beginning to calibrate
void calculate_IMU_error(){

  // Serial.println("calculating error...");

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  float prevGyroX = Wire.read() << 8 | Wire.read();
  float prevGyroY = Wire.read() << 8 | Wire.read();
  float prevGyroZ = Wire.read() << 8 | Wire.read();

  float samplesX[errorN];
  float samplesY[errorN];
  float samplesZ[errorN];

  // Reading gyro values
  int c = 0;
  while (c < errorN) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX - prevGyroX);
    GyroErrorY = GyroErrorY + (GyroY - prevGyroY);
    GyroErrorZ = GyroErrorZ + (GyroZ - prevGyroZ);

    samplesX[c] = GyroX - prevGyroX;
    samplesY[c] = GyroY - prevGyroY;
    samplesZ[c] = GyroZ - prevGyroZ;

    prevGyroX = GyroX;
    prevGyroY = GyroY;
    prevGyroZ = GyroZ;
    
    c++;
  }
  // Serial.println("Done with mean error");


  //Divide the sum by errorN to get the mean error
  GyroErrorX = GyroErrorX / float(errorN);
  GyroErrorY = GyroErrorY / float(errorN);
  GyroErrorZ = GyroErrorZ / float(errorN);

  int sizeOfSamples = sizeof(samplesX)/sizeof(samplesX[0]);
  stdGyroX = calculate_std(samplesX, GyroErrorX, sizeOfSamples); 
  stdGyroY = calculate_std(samplesY, GyroErrorY, sizeOfSamples);
  stdGyroZ = calculate_std(samplesZ, GyroErrorZ, sizeOfSamples);

  Serial.print("stdGyroX: "); Serial.println(stdGyroX);
  Serial.print("stdGyroY: "); Serial.println(stdGyroY);
  Serial.print("stdGyroZ: "); Serial.println(stdGyroZ);
  
  
  // Print the error values on the Serial Monitor
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
//  Serial.print("GyroErrorY: ");
//  Serial.println(GyroErrorY);
//  Serial.print("GyroErrorZ: ");
//  Serial.println(GyroErrorZ);
//
//  Serial.print("GyroErrorXDeg: ");
//  Serial.println(GyroErrorX / gyroSens);
//  Serial.print("GyroErrorYDeg: ");
//  Serial.println(GyroErrorY / gyroSens);
//  Serial.print("GyroErrorZDeg: ");
//  Serial.println(GyroErrorZ / gyroSens);
  
}

float calculate_std(float samples[], float mean, int size){

  float ans;

  float sum = 0;
  for(int i = 0; i < size; i++){
    sum = pow(samples[i] - mean, 2) + sum;
  }

  ans = sqrt(sum/size);
  return ans;

}

float eq_score(float maxAng, float minAng){

  return (12.5 - (maxAng - minAng))*100/12.5;
}
