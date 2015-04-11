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