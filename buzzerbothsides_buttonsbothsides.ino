/* this code is intended for the hearing portion of Team 17's Redesign Project
// the code begins by a press of a button
// beeping from piezobuzzers is randomized for left and right ears and the helmet user will 
// indicate whether they hear the buzzer by pressing either a left button or a right button
// this code will count successes and error and indicate a pass rate with a set of LED lights
*/


//initializing variables

int buzzerStateLeft = 0;
int buzzerStateRight = 0;
int buzzerPinLeft = 8;
int buzzerPinRight = 4;
int buttonPinStart = 12;
int buttonNew = 0;
int buttonOld = 0;
int buttonLeft = 7;
int buttonLeftCt = 0;
int buttonRight = 6;
int buttonRightCt = 0;
int errorLeft= 0;
int errorRight = 0;
int run;
int testCount = 1;
int beginTime;
int lastTime;
;



void setup() {
  // put your setup code here, to run once:
run = 0; //the code starts stopped (nothing is running)

pinMode(buttonLeft, INPUT);
pinMode(buttonRight, INPUT);  
pinMode(buzzerPinLeft, OUTPUT);
pinMode(buzzerPinRight, OUTPUT);
pinMode(buttonPinStart, INPUT);
digitalWrite(buzzerPinRight, LOW);
digitalWrite(buzzerPinLeft, LOW);
Serial.begin(9600);
}

void loop() 
{
  // put your main code here, to run repeatedly
  
  //PUSH BUTTON TO START ENTIRE HEARING TEST
  buttonNew = digitalRead(buttonPinStart);
  
  if(digitalRead(buttonPinStart)==LOW) //off
  {
    if (run==0){
      run = 255;
    }

    else {
      run = 0;
    }
  }

  /* 
  // checking testCount for 5 tests 
  // if less than five it will continue
  */
  while (run>0)
  { 
 
  if (testCount==6)
     { 
     Serial.print("done");        
     delay(5000);
     exit(0);
     }
  
  else 
    {
     Serial.print("test number=");
     Serial.println(testCount);
    }

  delay (1000);

  /*
   * randomizing left and right buzzers for the test
   * selects a random number, divides it by two, then checks for a remainder
   * if even, remainder is zero, and it plays right buzzer
   * if odd, remainder is 1 (not equal to zero), and it plays left buzzer
   */
  int x = random(1, 100);     
  int evenOrOdd = x % 2;

  //printing random numbers to check
  
  Serial.print("Random Number =");
  Serial.println(x);

  
  // RIGHT = EVEN 
   if (evenOrOdd==0)  
    {
    digitalWrite(buzzerPinRight, HIGH);
    digitalWrite(buzzerPinLeft, LOW);
    delay(500);
    digitalWrite(buzzerPinRight, LOW);
    buzzerStateRight = 1; // buzzer state 1 means the buzzer did ring
    }

    
    // LEFT = ODD
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
   
   while(elapsedTime < 10000 ) // 10 seconds = 10000 milliseconds
    {
     lastTime = millis();
     elapsedTime = lastTime - beginTime;  
     
     if(digitalRead(buttonLeft)==HIGH)
     {
      buttonPressedLeft = 1;
     }
    
     if(digitalRead(buttonRight)==HIGH)
     {
      buttonPressedRight = 1;
     }
    }

/*Recording the state of the buttons for tallying purposes
 * 
 * 
 */

    // state of the left button
    if(buttonPressedLeft==1)      
     { 
      buttonLeftCt = buttonLeftCt + 1;      // counter for the amt of times the LEFT button is hit
                                            // if the button IS hit (state HIGH) then add one to the counter
     }                                      
        
    else                       
      {                              
       errorLeft = errorLeft + 1;            // LEFT button was NOT hit so the player got it wrong
      }








    // state of the right button
    if (buttonPressedRight==1) 
     {
      buttonRightCt = buttonRightCt + 1;       // counter for the amt of time the RIGHT button is hit
                                               // if the button was hit (state HIGH) then add one to counter
     }
    else
      {
       errorRight = errorRight + 1; // RIGHT button was NOT hit so the player got it wrong
      }
      
   //Printing values and labels (for testing purposes)
   Serial.print ("button left count = ");
   Serial.println(buttonLeftCt);
   Serial.print ("button right count = ");
   Serial.println(buttonRightCt);
   delay (5000);
   Serial.print ("error for the left ear=");
   Serial.println(errorLeft);
   Serial.print ("error for the right ear=");
   Serial.println(errorRight);
   delay (5000);
   testCount = testCount+1;
  // buttonOld = buttonNew;
  
  
  }
  
}

// end of main loop
 
