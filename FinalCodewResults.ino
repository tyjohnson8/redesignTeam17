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
#include <MPU6050.h>

//////////// INITIALIZATION ////////////////
const int BODY = 0x68; // MPU6050 I2C address for BODY
const int HELMET = 0x69; // HELMET
// AD0 Pin is connected to 3.3 V directly

// MPU6050
float accSens = 16384.0;  //  The LSB/g sensitivity of the +/- 2g setting
float gyroSens = 131.0; // The LSB/deg/s sensitivity for +/- 250 deg/sec
// Keep sensitivity of accelerometer at +/- 2g
// Keep sensitivity of gyroscope at default +/- 250deg/s
MPU6050 bodyMPU;  // IC2Dev Library
MPU6050 helmMPU;

// Struct to contain errors
struct errorVals {
  float AccErrorXOrig;
  float AccErrorYOrig;
  float AccErrorZOrig;

  float GyroErrorX;
  float GyroErrorY;
  float GyroErrorZ;

  float stdAccX;
  float stdAccY;
  float stdAccZ;

  float stdGyroX;
  float stdGyroY;
  float stdGyroZ;
};

////////////////////////////////////////////

/////////////// I/O ////////////////////////
// LED //
const int RED = 9;
const int YELLOW = 10;
const int GREEN = 11;

// BALANCE
const int BALANCE = 2;    // Start balance test

// HEARING
const int HEARING = 12;   // Start hearing test
const int buzzerPinLeft = 8;   
const int buzzerPinRight = 4;
const int buttonLeft = 7;
const int buttonRight = 6;

////////////////////////////////////////////

////////////////// DATA ///////////////////

// MPU6050
float AccX, AccY, AccZ;   // Raw values


float elapsedTime, currentTime, previousTime;    // Used to calculate the degree
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Derived values
float gyroAngleX2, gyroAngleY2, gyroAngleZ2; // for second MPU


// Hearing
int errorLeft= 0;
int errorRight = 0;
int buttonCorrectLeft = 0;    
int buttonCorrectRight = 0;
int errorCorrectTotal = 0;    // errorRight + errorLeft

///////////////////////////////////////////

///////////// ERROR CALCULATION ///////////
int errorN = 100;         // The number of samples to take to find error
float errorMargin = 3;    // The number of standard deviations off of the avg error 
                          // the reading has to be in order to be considered real

// MPU1
// Calibration parameters (natural error)
errorVals bodyErrors;

// MPU2
errorVals helmErrors;

////////////////////////////////////////////


///////////// CONDITIONALS /////////////////
const float alertThreshold = 1.4;   // The number in Gs for which to send an alert (mTBI risk)
const int hearingThreshold = 2;     // The number of errors that define a failure of the hearing test
const float balanceThreshold = 69.0;    // the Equilibrium Score limit that defines a failure of the hearing test
const float EQ_thresh = 30;
const int hearing_test_delay = 500;

boolean hearing_result; // Store the pass/fails of each test
boolean balance_result;

////////////////////////////////////////////


void setup() {
  Serial.begin(9600);

  // LED I/O
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  //

  // Hearing
  pinMode(HEARING, INPUT);
  pinMode(buttonLeft, INPUT);
  pinMode(buttonRight, INPUT);  
  pinMode(buzzerPinLeft, OUTPUT);
  pinMode(buzzerPinRight, OUTPUT);
  digitalWrite(buzzerPinRight, LOW);
  digitalWrite(buzzerPinLeft, LOW);
  //
  
  // MPU6050
  Wire.begin();
  Wire.beginTransmission(BODY);
  Wire.write(0x6B);             // Initialize communication with the 6B register of the chip
  Wire.write(0x00);             // Reset values in 0x6B register
  Wire.endTransmission(true);  

  Wire.begin();
  Wire.beginTransmission(HELMET);
  Wire.write(0x6B);             
  Wire.write(0x00);             
  Wire.endTransmission(true);
  
  bodyMPU = MPU6050(BODY);      
  helmMPU = MPU6050(HELMET);
  //

  // Calculate error for each chip
  bodyErrors = calculate_error(BODY);
  helmErrors = calculate_error(HELMET);
  
} // END of SETUP




void loop() {

  // Continuously check for impact from HELMET MPU
  impact(HELMET);
  // If g-force beyond alertThreshold, alert LEDS
  // Standby until either hearing or balance test button is pressed

  // mTBI TESTS
  while(true){
    // If user starts with hearing, perform test and wait until balance is finished
    if(digitalRead(HEARING)){
      hearing_result = hearing(HEARING, buzzerPinLeft, buzzerPinRight, buttonLeft, buttonRight);
      while(!digitalRead(BALANCE)){}
      balance_result = balance(BODY, bodyMPU, bodyErrors);
      break;
    }

    // if user starts with balance, then vice versa
    if(digitalRead(BALANCE)){
      balance_result = balance(BODY, bodyMPU, bodyErrors);
      while(!digitalRead(HEARING)){}
      hearing_result = hearing(HEARING, buzzerPinLeft, buzzerPinRight, buttonLeft, buttonRight);
      break;
    }
  }

  delay(2000);

  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  // Evaluate verdict by combining both results
  int sum = balance_result + hearing_result;
  if(sum){
    if(sum == 1){
      digitalWrite(YELLOW, HIGH);
    } else{
      digitalWrite(GREEN, HIGH);
    }
  } else{
    digitalWrite(RED, HIGH);
  }
  delay(5000);
  
  digitalWrite(RED, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(GREEN, LOW);

} // END of MAIN


/////////// IMPACT ///////////////

void impact(int HELMET){

  digitalWrite(RED, LOW);
  
  while(true){

    // === Read accelerometer data === //
    Wire.beginTransmission(HELMET);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(HELMET, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    AccX = (Wire.read() << 8 | Wire.read()); // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()); // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()); // Z-axis value

    // Check for error margin (in gs)
    //For a sensitivity of +/-2g, we need to divide the raw values by accSens, according to the datasheet
    if(AccX / accSens > alertThreshold || AccX / accSens < -alertThreshold)
    {
      digitalWrite(RED, HIGH); return; // Alert and return
    }
    if(AccY / accSens > alertThreshold || AccY / accSens < -alertThreshold)
    {
      digitalWrite(RED, HIGH); return;
    }
    if(AccZ / accSens > alertThreshold || AccZ / accSens < -alertThreshold)
    {
      digitalWrite(RED, HIGH); return;
    }
      
  }
  
}

//////////////////////////////////


///////// HEARING ////////////////
/*
 * Returns whether the person passed the hearing test or not.
 */
boolean hearing(int buttonPinStart, int buzzerPinLeft, int buzzerPinRight, int buttonLeft, int buttonRight){
  // Initialization
//  int buttonLeftCt = 0; // shouldn't need these
//  int buttonRightCt = 0;
  digitalWrite(RED, LOW); // reset the red light
  digitalWrite(GREEN, LOW);
  
  int buzzerStateLeft = 0;
  int buzzerStateRight = 0;
  int buttonNew = 0;
  int buttonOld = 0;
  int testCount = 1;
  int beginTime;
  int lastTime;

  while(testCount < 9){ // end after some tests
    /*
     * randomizing left and right buzzers for the test
     * selects a random number, divides it by two, then checks for a remainder
     * if even, remainder is zero, and it plays right buzzer
     * if odd, remainder is 1 (not equal to zero), and it plays left buzzer
     */

    int x = random(1, 100);     
    int evenOrOdd = x % 2;

    // RIGHT = EVEN 
     if (evenOrOdd==0)  
      {
      digitalWrite(buzzerPinRight, HIGH);
      digitalWrite(buzzerPinLeft, LOW);
      delay(500);
      digitalWrite(buzzerPinRight, LOW);
      buzzerStateRight = 1; // buzzer state 1 means the buzzer did ring
      }

      // LEFT = ODD = LOUD
      else 
       {                                    
       digitalWrite(buzzerPinLeft, HIGH); //left rings
       digitalWrite(buzzerPinRight, LOW); //right stays silent
       delay(500);
       digitalWrite(buzzerPinLeft, LOW);  //left goes quiet after half a second
       buzzerStateLeft = 1;               //increments a counter for the amt of times the left buzzer has rang
       }

      /* 
      // a while loop that waits 10 seconds and constantly
      // checks if the button is pressed even once (change a variable to high)
      // then after the time period, check that variable whether it is high or low
      */
      
     int buttonPressedLeft = 0;
     int buttonPressedRight = 0;
     beginTime = millis();
     int elapsedTime = 0;

     while(elapsedTime < hearing_test_delay ) // 10 seconds = 10000 milliseconds
    {
     lastTime = millis();
     elapsedTime = lastTime - beginTime;  
     
     if(digitalRead(buttonLeft)==HIGH)
     {
      buttonPressedLeft = 1;
     }
    //test implemented - if the left button isn't pressed and the right button IS pressed, the counter for whether the right 
    // button is pressed should be incremented
     if(digitalRead(buttonRight)==HIGH)
     {
      buttonPressedRight = 1;
     }
    }
      
    // correctly pressed buttons
    if (evenOrOdd == buttonPressedLeft)
      { 
        if (buttonPressedLeft ==1)
        buttonCorrectLeft = buttonCorrectLeft +1;
      

        else if(buttonPressedRight==1)
        {
          buttonCorrectRight = buttonCorrectRight + 1;
        }

        else if (buttonPressedRight ==0)
        {
          errorCorrectTotal ++;
        }
      }
    else
      {
        errorCorrectTotal = errorCorrectTotal + 1;
      }

      
    testCount++;
  }

  // Recording
  Serial.println(errorCorrectTotal); 

  // Evaluate Results
  if(errorCorrectTotal > hearingThreshold){ // how many errors fail the test
    digitalWrite(RED, HIGH);
    return false;
  } else{ 
    digitalWrite(GREEN, HIGH);
    return true; 
    }
  
}


//////////////////////////////////

///////////// BALANCE ///////////////

boolean balance(int MPU1Addr, MPU6050 myMPU1, errorVals MPU1errors){

  digitalWrite(RED, LOW); // reset the red light
  digitalWrite(GREEN, LOW);

  float prevGyroX = 0.0; // Compare the last reading with the current one
  float prevGyroY = 0.0;
  float prevGyroZ = 0.0;
  float GyroX, GyroY, GyroZ;
  int count = 0;

  // Re-initialization
  float maxPGyroX, maxPGyroY, maxPGyroZ, maxNGyroX, maxNGyroY, maxNGyroZ = 0.0;   // Used in calculation of Equilibrium score
  Serial.print("Max P Angle X: "); Serial.println(maxPGyroX);
  Serial.print("Max N Angle X: "); Serial.println(maxNGyroX);
  Serial.print("Max P Angle Y: "); Serial.println(maxPGyroY);
  Serial.print("Max N Angle Y: "); Serial.println(maxNGyroY);
  Serial.print("Max P Angle Z: "); Serial.println(maxPGyroZ);
  Serial.print("Max N Angle Z: "); Serial.println(maxNGyroZ);

  float beginTest = millis();

  while(millis() - beginTest < 20000){  // Test is 20 seconds long
  
    // Reading Gyro input (deg/s)
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;
    Wire.beginTransmission(MPU1Addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1Addr, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    static float firstGyroX = GyroX;
    static float firstGyroY = GyroY;
    static float firstGyroZ = GyroZ;

    GyroX = GyroX - firstGyroX;
    GyroY = GyroY - firstGyroY;
    GyroZ = GyroZ - firstGyroZ;

    // BODY SWAY ///////////////////////
    // X-Axis
    if( abs(GyroX - prevGyroX) > abs(MPU1errors.GyroErrorX + errorMargin*MPU1errors.stdGyroX) ){
      // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by senconds (s) to get the angle in degrees
      gyroAngleX = gyroAngleX + (GyroX * elapsedTime) / gyroSens; // deg/s * s = deg

      // Positive peak
      if(gyroAngleX > 0 && gyroAngleX > maxPGyroX) {maxPGyroX = gyroAngleX;}
      // Negative Peak
      if(gyroAngleX < 0 && gyroAngleX < maxNGyroX) {maxNGyroX = gyroAngleX;}

    }
    prevGyroX = GyroX;

    // Y-axis
    if( abs(GyroY - prevGyroY) > abs(MPU1errors.GyroErrorY + errorMargin*MPU1errors.stdGyroY) ){
      gyroAngleY = gyroAngleY + (GyroY * elapsedTime) / gyroSens;
      
      if(gyroAngleY > 0 && gyroAngleY > maxPGyroY) {maxPGyroY = gyroAngleY;}

      if(gyroAngleY < 0 && gyroAngleY < maxNGyroY) {maxNGyroY = gyroAngleY;}
    
    }
    prevGyroY = GyroY;
    
    // Z-axis
    if( abs(GyroZ - prevGyroZ) > abs(MPU1errors.GyroErrorZ + errorMargin*MPU1errors.stdGyroZ) ){
      gyroAngleZ = gyroAngleZ + (GyroZ * elapsedTime) / gyroSens;
      
      if(gyroAngleZ > 0 && gyroAngleZ > maxPGyroZ) {maxPGyroZ = gyroAngleZ;}

      if(gyroAngleZ < 0 && gyroAngleZ < maxNGyroZ) {maxNGyroZ = gyroAngleZ; }


    }
    prevGyroZ = GyroZ;

    if(count % 1000 || count == 1){
      // Record
      Serial.print(gyroAngleX); Serial.print(",");
      Serial.print(gyroAngleY); Serial.print(",");
      Serial.println(gyroAngleZ); 
    }

    count++;

  } // end of Test

  Serial.print(maxPGyroX); Serial.print(",");
  Serial.print(maxNGyroX); Serial.print(",");
  Serial.print(maxPGyroY); Serial.print(",");
  Serial.print(maxNGyroY); Serial.print(",");
  Serial.print(maxPGyroZ); Serial.print(",");
  Serial.println(maxNGyroZ); 
  
  float X_eq = eq_score(maxPGyroX, maxNGyroX);
  float Y_eq = eq_score(maxPGyroY, maxNGyroY);
  float Z_eq = eq_score(maxPGyroZ, maxNGyroZ);

  // Record
  Serial.print(X_eq); Serial.print(",");
  Serial.print(Y_eq); Serial.print(",");
  Serial.println(Z_eq); 

  // Evaluate Results
  if(X_eq < balanceThreshold || Y_eq < balanceThreshold || Z_eq < balanceThreshold){
    digitalWrite(RED, HIGH);
    return false; 
  } else{ 
    digitalWrite(GREEN, HIGH);
    return true; 
    }


  // HEAD COMPARISON //
    // To be done
  //
  
}


//////////////////////////////////////


// Finds the natural drift of the gyroscope
// Chip should be still in the beginning to calibrate
errorVals calculate_error(int MPUAddr){

  // Initialization
  int c = 0;
  int sizeOfSamples;
  double samplesX[errorN]; // Hold error values
  double samplesY[errorN];
  double samplesZ[errorN];
  double AccErrorXOrig;
  double AccErrorYOrig;
  double AccErrorZOrig;
  

  // ACCELERATION //
  while(c < errorN){
    Wire.beginTransmission(MPUAddr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPUAddr, 6, true);

    // Extract raw values
    AccX = (Wire.read() << 8 | Wire.read()); 
    AccY = (Wire.read() << 8 | Wire.read());
    AccZ = (Wire.read() << 8 | Wire.read()); 

    // Error storage for std deviation (in LSB/g)
    samplesX[c] = AccX;
    samplesY[c] = AccY;
    samplesZ[c] = AccZ;

    // Sum all readings (LSB/g)
    AccErrorXOrig = AccX + AccErrorXOrig; // Raw error values
    AccErrorYOrig = AccY + AccErrorYOrig;
    AccErrorZOrig = AccZ + AccErrorZOrig;

    c++;
  }

  // Divide the sum by the number of samples to get the mean error
  AccErrorXOrig = AccErrorXOrig/errorN; // Raw values
  AccErrorYOrig = AccErrorYOrig/errorN;
  AccErrorZOrig = AccErrorZOrig/errorN;

  // Calculate Standard Deviation
  sizeOfSamples = sizeof(samplesX)/sizeof(samplesX[0]);
  double stdAccX = calculate_std(samplesX, AccErrorXOrig, sizeOfSamples);
  double stdAccY = calculate_std(samplesY, AccErrorYOrig, sizeOfSamples);
  double stdAccZ = calculate_std(samplesZ, AccErrorZOrig, sizeOfSamples);


  // GYROSCOPE //
  double GyroErrorX = 0;
  double GyroErrorY = 0;
  double GyroErrorZ = 0;
  float GyroX, GyroY, GyroZ;
  
  Wire.beginTransmission(MPUAddr);  // Read previous values to get a drift recording
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUAddr, 6, true);
  float prevGyroX = Wire.read() << 8 | Wire.read();
  float prevGyroY = Wire.read() << 8 | Wire.read();
  float prevGyroZ = Wire.read() << 8 | Wire.read();

  c = 0;
  while (c < errorN) {
    Wire.beginTransmission(MPUAddr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPUAddr, 6, true);
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

  // Divide the sum by errorN to get the mean error
  GyroErrorX = GyroErrorX / errorN;
  GyroErrorY = GyroErrorY / errorN;
  GyroErrorZ = GyroErrorZ / errorN;

  Serial.print(GyroErrorX); Serial.print(",");
  Serial.print(GyroErrorY); Serial.print(",");
  Serial.println(GyroErrorZ);

  delay(500);

  // Calculate Standard Deviation
  sizeOfSamples = sizeof(samplesX)/sizeof(samplesX[0]);
  double stdGyroX = calculate_std(samplesX, GyroErrorX, sizeOfSamples); 
  double stdGyroY = calculate_std(samplesY, GyroErrorY, sizeOfSamples);
  double stdGyroZ = calculate_std(samplesZ, GyroErrorZ, sizeOfSamples);  

  errorVals resErrors = {
    AccErrorXOrig,
    AccErrorYOrig,
    AccErrorZOrig,
  
    GyroErrorX,
    GyroErrorY,
    GyroErrorZ,
  
    stdAccX,
    stdAccY,
    stdAccZ,
  
    stdGyroX,
    stdGyroY,
    stdGyroZ
  };

  return resErrors;
}

double calculate_std(double samples[], double mean, int len){

  double ans;
  unsigned long sum = 0;
  for(int i = 0; i < len; i++){
    sum = pow(samples[i] - mean, 2) + sum;
  }

  ans = sqrt(sum/len);

  return ans;

}

// Based on the Sensory Organization Test
float eq_score(float maxAng, float minAng){

  return (EQ_thresh - (maxAng - minAng))*100/EQ_thresh;
}

// LED Switch 
void turnAllOff(){

  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(YELLOW, LOW);
 
}
