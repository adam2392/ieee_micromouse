<<<<<<< HEAD
/* IEEE Micromouse Main Driver File:

By: Adam Li, Eric Lui, Matt Fach

Dependencies:
- 

Pins Used:
0, 1, 2, 3, 4, 5, 6, 16, 17, 18, 19, 20, 21, 22, 23

*/
//Define Driving Direction
#define BACKWARD 0
#define FORWARD 1

//LED Pin Definitions for debugging
#define led1 26
#define led2 27
#define led3 28

//Pin definitions for motor
const int R_fwd = 6, R_bkw = 5;  //R_fwd = 1 and R_bkw = 0 -> go forward on rightside
const int L_fwd = 4, L_bkw = 3;  //L_fwd = 1 and L_bkw = 0 -> go forward on leftside

//Sensors
#define LF_Emitter 23        //left front emitter
#define LF_Receiver 17      //left front receiver
#define RF_Emitter 20        //right front emitter
#define RF_Receiver 16      //right front receiver
#define SIDE_HIGH_POWER 21  //turn on Right and Left Emitters
#define SIDE_LOW_POWER 22  //turn off Right and Left Emitters
#define R_Receiver 19    //Right sensor receiver
#define L_Receiver 18  //Left sensors receiver

// Encoders 
#define LenchA 8 
#define LenchB 7
#define RenchA 10
#define RenchB 9


//Speaker
#define speakerPin 2

// READ SENSORS
int readLeft(){
  return analogRead(L_Receiver);
}
int readRight(){
  return analogRead(R_Receiver);
}
int readLeft_Front(){
  return analogRead(LF_Receiver);
}
int readRight_Front(){
  return analogRead(RF_Receiver);
}
// WRITE SENSORS
void writeSides(){
  digitalWrite(SIDE_HIGH_POWER, HIGH);
}
void writeLeftDiag(){
  digitalWrite(SIDE_LOW_POWER, HIGH);
}
void writeLeftFront(){
  digitalWrite(LF_Emitter, HIGH);
}
void writeRightFront(){
  digitalWrite(RF_Emitter, HIGH);
}

// DRIVING Functions for left and right motor ***** Probably needs testing to set pwmvalue
//turn left motor on or off
void leftForward(int pwmvalue){
  //digital write for testing/debugging
  analogWrite(L_fwd, pwmvalue);
  analogWrite(L_bkw, LOW);
}
void rightForward(int pwmvalue){
  //digital write for testing/debugging
  analogWrite(R_fwd, pwmvalue);
  analogWrite(R_bkw, LOW);
}
void leftBackward(int pwmvalue){
  analogWrite(L_fwd, LOW);
  analogWrite(L_bkw, pwmvalue);
}
void rightBackward(int pwmvalue){
  analogWrite(R_fwd, LOW);
  analogWrite(R_bkw, pwmvalue);  
}
 
void readEnc(){
   //rightenc = analogRead(RenchA); 
   Serial.print("Right EncA: ");
   Serial.println(digitalRead(RenchA)); 
   
   //rightencb = analogRead(RenchB); 
   Serial.print("Right EncB: ");
   Serial.println(digitalRead(RenchB)); 
   Serial.println(" "); 
   
   //leftenca = analogRead(LenchA); 
   Serial.print("Left EncA: ");
   Serial.println(digitalRead(LenchA));
  
   //leftencb = analogRead(LenchB);  
   Serial.print("Left EncB: ");
   Serial.println(digitalRead(LenchB));
}
void about_face(){
  
  rightForward(20); 
  leftBackward(30); 
  delay(1000);
  rightForward(0); 
  leftForward(0); 
  
}

 int rightenca = 0; 
 int rightencb = 0; 
 int leftenca = 0; 
 int leftencb = 0;
 
 
// the setup routine runs once when you press reset:
void setup()
{
   
  pinMode(RenchA, INPUT); 
  pinMode(RenchB, INPUT);
  pinMode(LenchA, INPUT); 
  pinMode(LenchB, INPUT); 
  
  //initialize motor pins to off
  pinMode(R_fwd, OUTPUT);  //initialize rightmotor forward as output
  pinMode(R_bkw, OUTPUT);
  pinMode(L_fwd, OUTPUT);
  pinMode(L_bkw, OUTPUT);
    
  Serial.begin(9600);     //initialize for serial output 
  
  //power for the receivers *leave as is DO NOT CHANGE*
  pinMode(A14, OUTPUT);  
  analogWrite(A14, 255);
  
  //initialize debug LEDs
  pinMode(led1, OUTPUT);  //setup led1 for output
  pinMode(led2, OUTPUT);  //setup led2 for output
  pinMode(led3, OUTPUT);  //setup led3 for output
  
  //setup sensor pins for output and input
  pinMode(LF_Emitter, OUTPUT);  //initializes leftfront emitter to output stuff
  pinMode(RF_Emitter, OUTPUT);
  pinMode(LF_Receiver, INPUT);  //initialize leftfront receiver to receive input from stuff
  pinMode(RF_Receiver, INPUT);
  pinMode(SIDE_HIGH_POWER, OUTPUT);
  pinMode(L_Receiver, INPUT);
  pinMode(R_Receiver, INPUT);
  
  //setup the speaker pin
  pinMode(speakerPin, OUTPUT);
  
  rightForward(0); 
  leftForward(0); 
}

// the loop routine runs over and over again forever:
void loop()
{
  writeSides();  //turn on side emitters to receive distance signal


  //drive_straight_PID();
  //leftForward(30); 
  //rightForward(30); 
  //leftBackward(30);
  //rightBackward(30); 
  //Serial.print("IR left diag: ");
  //Serial.println(readLeft());
  //Serial.print("IR right diag: "); 
  //Serial.println(readRight()); 
  //drive_test(); 
  //drive_straight_PID(); 
  readEnc(); 
  //about_face(); 
  delay(100);
}
void drive_test(){
  int leftDistance = readLeft(); 
  
  if(leftDistance > 80) {
    leftBackward(30); 
    rightBackward(30); 
  }  
  else if( leftDistance < 70 ) {
    rightForward(30); 
    leftForward(30);  
  }
  
}
/**** PID TEST ****/
void drive_straight_PID(void){
  int offset = -40;
  static int previous_error = 0;
  static int previous_time = 0;
  static int last_big = 0;
  int error; //current error values
  int biggest;
  int current_time; //current time
  double total;
  int leftDiagSensor, rightDiagSensor;
  double kp = 0.5, kd = 0.5;
  leftDiagSensor = readLeft();
  rightDiagSensor = readRight();
  //debug print out sensor readings
  //Serial.print("IR left diag: ");
  //Serial.print(leftDiagSensor);
  //Serial.print(" IR right diag: ");
  //Serial.print(rightDiagSensor);
  
  if(!previous_time)
  {
    previous_time = millis();
    return;
  }
  leftDiagSensor = readLeft();
  rightDiagSensor = readRight();
  if( 1 )//temporarily for walls on both sides only |x|
  {
    error = rightDiagSensor - leftDiagSensor + offset;
  }
  total = error *kp;
  previous_time = current_time;
  
  //analogWrite(R_fwd, HIGH - total);
  //analogWrite(L_fwd, HIGH + total);
  //what the PID will do (because motor functions are not done)
  if(total > 25)
    total=0.5*total; 
  if(total > 50 )
    total = 0; 
  if(total<-50)
    total=0;
  Serial.print("total error: "); 
  Serial.println(total); 
  rightForward(15+total); 
  leftForward(25-total); 
  if( error == 0 ){
    Serial.print(" Mouse is straight: ");
    Serial.println(error);
     
  }
  if( error > 0 ){
    Serial.print(" Mouse is veering right: ");
    Serial.println(error);
    
  }
  if( error < 0 ){
    Serial.print(" Mouse is veering left: ");
    Serial.println(error);
  }
}//end drive_straight_PID
=======
<<<<<<< HEAD
/*‪#‎include‬ <LedDisplay.h>
//LCD Pins
‪#‎define‬ enable1 11 // the display's chip enable pin
#define reset1 24 // the display's reset pin
#define dataPin 12 // connects to the display's data in
#define registerSelect 1 // the display's register select pin
#define clockPin 15 // the display's clock pin
#define displayLength 8 // number of characters in the display
// create am instance of the LED display library:
LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin,
enable1, reset1, displayLength);
#define brightness 15 // screen brightness
*/
//LEDs
#define led1 26
#define led2 27
#define led3 28
const int R_fwd = 5, R_bkw = 6, L_fwd = 3, L_bkw = 4;
//Sensors
#define LF_Emitter 23
#define LF_Receiver 17
#define RF_Emitter 20
#define RF_Receiver 16
#define SIDE_HIGH_POWER 21
#define SIDE_LOW_POWER 22
#define R_Receiver 19
#define L_Receiver 18
//Speaker
#define speakerPin 2
// READ SENSORS
int readLeft(){
return analogRead(L_Receiver);
}
int readRight(){
return analogRead(R_Receiver);
}
int readLeft_Front(){
return analogRead(LF_Receiver);
}
int readRight_Front(){
return analogRead(RF_Receiver);
}
// WRITE SENSORS
void writeDiag(){
digitalWrite(SIDE_HIGH_POWER, HIGH);
}
void writeLeftDiag(){
digitalWrite(SIDE_LOW_POWER, HIGH);
}
void writeLeftFront(){
digitalWrite(LF_Emitter, HIGH);
}
void writeRightFront(){
digitalWrite(RF_Emitter, HIGH);
}
// the setup routine runs once when you press reset:
void setup()
{
digitalWrite(R_fwd,LOW);
digitalWrite(R_bkw,LOW);
digitalWrite(L_fwd,LOW);
digitalWrite(R_bkw,LOW);
Serial.begin(9600);
pinMode(A14, OUTPUT);
analogWrite(A14, 255);
/*
// initialize the display library:
myDisplay.begin();
// set the brightness of the display:
myDisplay.setBrightness(brightness);*/
// initialize the digital pin as an output.
pinMode(led1, OUTPUT);
pinMode(led2, OUTPUT);
pinMode(led3, OUTPUT);
pinMode(LF_Emitter, OUTPUT);
pinMode(RF_Emitter, OUTPUT);
pinMode(SIDE_HIGH_POWER, OUTPUT);
pinMode(speakerPin, OUTPUT);
}
// the loop routine runs over and over again forever:
void loop()
{
/*
// set the cursor to 0:
myDisplay.home();
// print the millis:
myDisplay.print("ms:");
myDisplay.print(millis());
*/
writeDiag();
//Serial.print("IR left front: ");
//Serial.println(readLeft_Front());
/*writeRightFront();
writeDiag();
Serial.print("IR right: ");
Serial.println(readRight());
Serial.print("IR left: ");
Serial.println(readLeft());
Serial.print("IR left diag: ");
Serial.print(readLeft());
Serial.print(" IR right diag: ");
Serial.println(readRight());
*/
drive_straight_PID();
delay(100);
}
/**** PID TEST ****/
void drive_straight_PID(void){
int offset = 556;
static int previous_error = 0;
static int previous_time = 0;
static int last_big = 0;
int error; //current error values
int biggest;
int current_time; //current time
double total;
int leftDiagSensor, rightDiagSensor;
double kp = 0.5, kd = 0.5;
leftDiagSensor = readLeft();
rightDiagSensor = readRight();
//debug print out sensor readings
//Serial.print("IR left diag: ");
//Serial.print(leftDiagSensor);
//Serial.print(" IR right diag: ");
//Serial.print(rightDiagSensor);
if(!previous_time)
{
previous_time = millis();
return;
}
leftDiagSensor = readLeft();
rightDiagSensor = readRight();
if( 1 )//temporarily for walls on both sides only |x|
{
error = rightDiagSensor - leftDiagSensor + offset;
}
total = error *kp;
previous_time = current_time;
//analogWrite(R_fwd, HIGH - total);
//analogWrite(L_fwd, HIGH + total);
//what the PID will do (because motor functions are not done)
if( error == 0 ){
Serial.print(" Mouse is straight: ");
Serial.println(error);
}
if( error > 0 ){
Serial.print(" Mouse is veering right: ");
Serial.println(error);
}
if( error < 0 ){
Serial.print(" Mouse is veering left: ");
Serial.println(error);
}
}//end drive_straight_PID
=======
/* IEEE Micromouse Main Driver File:

By: Adam Li, Eric Lui, Matt Fach

Dependencies:
- 

Pins Used:
0, 1, 2, 3, 4, 5, 6, 16, 17, 18, 19, 20, 21, 22, 23

*/
//Define Driving Direction
#define BACKWARD 0
#define FORWARD 1

//LED Pin Definitions for debugging
#define led1 26
#define led2 27
#define led3 28

//Pin definitions for motor
const int R_fwd = 6, R_bkw = 5;  //R_fwd = 1 and R_bkw = 0 -> go forward on rightside
const int L_fwd = 4, L_bkw = 3;  //L_fwd = 1 and L_bkw = 0 -> go forward on leftside

//Sensors
#define LF_Emitter 23        //left front emitter
#define LF_Receiver 17      //left front receiver
#define RF_Emitter 20        //right front emitter
#define RF_Receiver 16      //right front receiver
#define SIDE_HIGH_POWER 21  //turn on Right and Left Emitters
#define SIDE_LOW_POWER 22  //turn off Right and Left Emitters
#define R_Receiver 19    //Right sensor receiver
#define L_Receiver 18  //Left sensors receiver

// Encoders 
#define LenchA 8 
#define LenchB 7
#define RenchA 10
#define RenchB 9


//Speaker
#define speakerPin 2

// READ SENSORS
int readLeft(){
  return analogRead(L_Receiver);
}
int readRight(){
  return analogRead(R_Receiver);
}
int readLeft_Front(){
  return analogRead(LF_Receiver);
}
int readRight_Front(){
  return analogRead(RF_Receiver);
}
// WRITE SENSORS
void writeSides(){
  digitalWrite(SIDE_HIGH_POWER, HIGH);
}
void writeLeftDiag(){
  digitalWrite(SIDE_LOW_POWER, HIGH);
}
void writeLeftFront(){
  digitalWrite(LF_Emitter, HIGH);
}
void writeRightFront(){
  digitalWrite(RF_Emitter, HIGH);
}

// DRIVING Functions for left and right motor ***** Probably needs testing to set pwmvalue
//turn left motor on or off
void leftForward(int pwmvalue){
  //digital write for testing/debugging
  analogWrite(L_fwd, pwmvalue);
  analogWrite(L_bkw, LOW);
}
void rightForward(int pwmvalue){
  //digital write for testing/debugging
  analogWrite(R_fwd, pwmvalue);
  analogWrite(R_bkw, LOW);
}
void leftBackward(int pwmvalue){
  analogWrite(L_fwd, LOW);
  analogWrite(L_bkw, pwmvalue);
}
void rightBackward(int pwmvalue){
  analogWrite(R_fwd, LOW);
  analogWrite(R_bkw, pwmvalue);  
}
 
void readEnc(){
   //rightenc = analogRead(RenchA); 
   Serial.print("Right EncA: ");
   Serial.println(digitalRead(RenchA)); 
   
   //rightencb = analogRead(RenchB); 
   Serial.print("Right EncB: ");
   Serial.println(digitalRead(RenchB)); 
   Serial.println(" "); 
   
   //leftenca = analogRead(LenchA); 
   Serial.print("Left EncA: ");
   Serial.println(digitalRead(LenchA));
  
   //leftencb = analogRead(LenchB);  
   Serial.print("Left EncB: ");
   Serial.println(digitalRead(LenchB));
}
void about_face(){
  
  rightForward(20); 
  leftBackward(30); 
  delay(1000);
  rightForward(0); 
  leftForward(0); 
  
}

 int rightenca = 0; 
 int rightencb = 0; 
 int leftenca = 0; 
 int leftencb = 0;
 
 
// the setup routine runs once when you press reset:
void setup()
{
   
  pinMode(RenchA, INPUT); 
  pinMode(RenchB, INPUT);
  pinMode(LenchA, INPUT); 
  pinMode(LenchB, INPUT); 
  
  //initialize motor pins to off
  pinMode(R_fwd, OUTPUT);  //initialize rightmotor forward as output
  pinMode(R_bkw, OUTPUT);
  pinMode(L_fwd, OUTPUT);
  pinMode(L_bkw, OUTPUT);
    
  Serial.begin(9600);     //initialize for serial output 
  
  //power for the receivers *leave as is DO NOT CHANGE*
  pinMode(A14, OUTPUT);  
  analogWrite(A14, 255);
  
  //initialize debug LEDs
  pinMode(led1, OUTPUT);  //setup led1 for output
  pinMode(led2, OUTPUT);  //setup led2 for output
  pinMode(led3, OUTPUT);  //setup led3 for output
  
  //setup sensor pins for output and input
  pinMode(LF_Emitter, OUTPUT);  //initializes leftfront emitter to output stuff
  pinMode(RF_Emitter, OUTPUT);
  pinMode(LF_Receiver, INPUT);  //initialize leftfront receiver to receive input from stuff
  pinMode(RF_Receiver, INPUT);
  pinMode(SIDE_HIGH_POWER, OUTPUT);
  pinMode(L_Receiver, INPUT);
  pinMode(R_Receiver, INPUT);
  
  //setup the speaker pin
  pinMode(speakerPin, OUTPUT);
  
  rightForward(0); 
  leftForward(0); 
}

// the loop routine runs over and over again forever:
void loop()
{
  writeSides();  //turn on side emitters to receive distance signal


  //drive_straight_PID();
  //leftForward(30); 
  //rightForward(30); 
  //leftBackward(30);
  //rightBackward(30); 
  //Serial.print("IR left diag: ");
  //Serial.println(readLeft());
  //Serial.print("IR right diag: "); 
  //Serial.println(readRight()); 
  //drive_test(); 
  //drive_straight_PID(); 
  readEnc(); 
  //about_face(); 
  delay(100);
}
void drive_test(){
  int leftDistance = readLeft(); 
  
  if(leftDistance > 80) {
    leftBackward(30); 
    rightBackward(30); 
  }  
  else if( leftDistance < 70 ) {
    rightForward(30); 
    leftForward(30);  
  }
  
}
/**** PID TEST ****/
void drive_straight_PID(void){
  int offset = -40;
  static int previous_error = 0;
  static int previous_time = 0;
  static int last_big = 0;
  int error; //current error values
  int biggest;
  int current_time; //current time
  double total;
  int leftDiagSensor, rightDiagSensor;
  double kp = 0.5, kd = 0.5;
  leftDiagSensor = readLeft();
  rightDiagSensor = readRight();
  //debug print out sensor readings
  //Serial.print("IR left diag: ");
  //Serial.print(leftDiagSensor);
  //Serial.print(" IR right diag: ");
  //Serial.print(rightDiagSensor);
  
  if(!previous_time)
  {
    previous_time = millis();
    return;
  }
  leftDiagSensor = readLeft();
  rightDiagSensor = readRight();
  if( 1 )//temporarily for walls on both sides only |x|
  {
    error = rightDiagSensor - leftDiagSensor + offset;
  }
  total = error *kp;
  previous_time = current_time;
  
  //analogWrite(R_fwd, HIGH - total);
  //analogWrite(L_fwd, HIGH + total);
  //what the PID will do (because motor functions are not done)
  if(total > 25)
    total=0.5*total; 
  if(total > 50 )
    total = 0; 
  if(total<-50)
    total=0;
  Serial.print("total error: "); 
  Serial.println(total); 
  rightForward(15+total); 
  leftForward(25-total); 
  if( error == 0 ){
    Serial.print(" Mouse is straight: ");
    Serial.println(error);
     
  }
  if( error > 0 ){
    Serial.print(" Mouse is veering right: ");
    Serial.println(error);
    
  }
  if( error < 0 ){
    Serial.print(" Mouse is veering left: ");
    Serial.println(error);
  }
}//end drive_straight_PID
>>>>>>> 4f46ba97072ef8117c3082b5719ac0909a76772f
>>>>>>> 7451016ecfbeb6269893ee99138351585d8f2c4a
