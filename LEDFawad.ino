

// LED I/O
const int RED = 9;
const int YELLOW = 10;
const int GREEN = 11;


void setup() {
  // put your setup code here, to run once:
pinMode(RED, OUTPUT);
pinMode(GREEN, OUTPUT);
pinMode(YELLOW, OUTPUT);
}

void loop() {

  digitalWrite(RED, 1);
  digitalWrite(YELLOW, 1);
  digitalWrite(GREEN, 1);
  
//  digitalWrite(GREEN, 1);
//  delay(1000);
//  digitalWrite(RED, 1);
//  delay(1000);
//  digitalWrite(YELLOW, 1);
//  delay(1000);
//  digitalWrite(YELLOW, 0);
//  delay(1000);
//  digitalWrite(RED, 0);
//  delay(1000);
//  digitalWrite(GREEN, 0);
//  delay(1000);
//  digitalWrite(GREEN, 1);
//  delay(1000);
  
}
