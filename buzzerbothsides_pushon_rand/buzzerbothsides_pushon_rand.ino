int buzzerStateLeft = 0;
int buzzerStateRight = 0;
int buzzerPinLeft = 8;
int buzzerPinRight = 4;
int buttonPin = 12;
int buttonNew = 0;
int buttonOld = 0;
int buttonLeft = 7;
int buttonStateLeft = 0;
int buttonRight = 2;
int buttonStateRight = 0;

int error = 0;
int run;
int testCount = 1;
void setup() {
  // put your setup code here, to run once:
run = 0; //the code starts stopped (nothing is running)
 
pinMode(buttonLeft, INPUT);
pinMode(buttonRight, INPUT);  
pinMode(buzzerPinLeft, OUTPUT);
pinMode(buzzerPinRight, OUTPUT);
pinMode(buttonPin, INPUT);
//pinMode(redButton, INPUT_PULLUP); //Read player button (w/o ext. pull up resistor)
digitalWrite(buzzerPinRight, LOW);
digitalWrite(buzzerPinLeft, LOW);
Serial.begin(9600);
}

void loop() {
  
  
    
    // put your main code here, to run repeatedly:
  
    //PUSH TO START ENTIRE HEARING TEST
  buttonNew = digitalRead(buttonPin);
  
  if(digitalRead(buttonPin)==LOW) //off
  {
    if (run==0){
      run = 255;
    }

    else {
      run = 0;
    }
  }
  
  while (run>0){ 
  
  
  //randomizing for left and right
  if (testCount==5)
     { //checking testCount for 5 tests 
                     // if less than five it will continue
     Serial.print("done");        
  
     exit(0);
     }
  
  else 
    {
     Serial.println(testCount);
    }
  
  delay (10000);
  
  int x = random(1, 100);
  int evenOrOdd = x % 2;
  
  Serial.println(x);
  
  // read button, if pressed, play buzzer. if not, stay silent
  // if the previous value of the button is a 0 and the new value is a 1
  // then you want to switch the state of the buzzer
  // if it was already off ---> on
  // if it was already on ---> off
  
  
  if (evenOrOdd==0) //RIGHT = EVEN
  
  {
    digitalWrite(buzzerPinRight, HIGH);
    digitalWrite(buzzerPinLeft, LOW);
    delay(500);
    digitalWrite(buzzerPinRight, LOW);
  
      
      buzzerStateRight = 1; // buzzer state 1 means ringing
      
    }
    else {   // LEFT = ODD
     digitalWrite(buzzerPinLeft, HIGH);
     digitalWrite(buzzerPinRight, LOW);
     delay(500);
     digitalWrite(buzzerPinLeft, LOW);
     buzzerStateLeft = 1;

// make a while loop that waits 10 seconds and constantly
// checks if the button is pressed even once (change a variable to high)
// then after the time period, check that variable whether it is high or low
// millis() = beginTime
// while(elapsedTime < 10000 milliseconds){
  // millis() = lastTime;
  // int elapsedTime = lastTime - beginTime;
// }
  
    delay(10000);
    }
    if(digitalRead(buttonLeft)==LOW && digitalRead(buttonRight)== HIGH)
   { // actual state of the button
    error = error+1;
  }
  
    else {//if the button IS hit (state HIGH) then add one to the counter
   buttonStateLeft = buttonStateLeft+1; // counter for the amt of times left is hit
    }
  
   Serial.print(buttonStateLeft);
   Serial.println(error);
  
   buttonOld = buttonNew;
  
  testCount = testCount+1;
  
  }
  
} // end of main loop
 
