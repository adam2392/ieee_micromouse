/* IEEE Micromouse Main Driver File:
 
 By: Adam Li, Eric Lui, Matt Fach
 
 Dependencies:
 - 
 
 Pins Used:
 0, 1, 2, 3, 4, 5, 6, 16, 17, 18, 19, 20, 21, 22, 23
 
 */
#include <avr/io.h>
#include "Maze.c"
#include "Maze.h"
#include "Stack.c"

#include <avr/interrupt.h>
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

#define ABOUT_FACE_COUNT 3600

#define TURN_RIGHT_COUNT 1900
#define TURN_LEFT_COUNT 1900

#define ONECELL 6000

int ran = 0; 
// TODO: find the correct values for these variables by 
// placing the mouse int he the middle of a maze path
// and measuring the sensor readings 
int hasLeftWall = 50; 
int hasRightWall = 50; 

int errorP = 0;
int errorD = 0; 
int oldErrorP = 0; 
int newOffset = -80; 
int rightBaseSpeed = 35; 
int leftBaseSpeed = 31;

int P = 0.25; 
int D = 0.3; 

int rightenca = 0; 
int rightencb = 0; 
int leftenca = 0; 
int leftencb = 0;
int time = 0; 
int n = 0; 
int m = 0; 
boolean keep_moving; 

int leftSense = 0; 
int rightSense = 0; 
volatile int R_encoder_val = 0;  // declare encoder interrupt values
volatile int L_encoder_val = 0;


const int numReadings = 500;

int readings[numReadings];      // the readings from the analog input
int smoothingIndex = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

int leftSensor = 0; 
int rightSensor = 0; 
int LFSensor = 0; 
int RFSensor =0; 

/**** maze solving variables ****/
struct Maze * my_maze; //maze that keeps track of flood fill values and walls
struct Stack * my_stack; //stack used for flood fill
struct Node * temp; //node used for in-between start->goal, goal->start transition
short found_dest; //flag if goal is reached
short direction; //direction that the mouse is facing
short x, y; //current coordinates of the mouse in the maze
short goal_x, goal_y; //goal coordinates once found



// the setup routine runs once when you press reset:
void setup()
{

  Serial.begin(9600);     //initialize for serial output
  pinMode(RenchA, INPUT); 
  pinMode(RenchB, INPUT);
  pinMode(LenchA, INPUT); 
  pinMode(LenchB, INPUT); 

  attachInterrupt(LenchA, left_interrupt, CHANGE);  // may need to adjust encoder operation
  attachInterrupt(LenchB, left_interrupt, CHANGE);
  attachInterrupt(RenchA, right_interrupt, CHANGE);
  attachInterrupt(RenchB, right_interrupt, CHANGE);  // comment out interrupts not used




  //initialize motor pins to off
  pinMode(R_fwd, OUTPUT);  //initialize rightmotor forward as output
  pinMode(R_bkw, OUTPUT);
  pinMode(L_fwd, OUTPUT);
  pinMode(L_bkw, OUTPUT);



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
  ran = 0; 

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  //rightForward(0); 
  //leftForward(0); 

  /**maze solving setup**/
  my_maze = new_Maze(); //initialize new maze
  my_stack = new_Stack(); //initialize new stack
  x = START_X;
  y = START_Y;
  direction = NORTH;
  found_dest = FALSE;


}


// the loop routine runs over and over again forever:
void loop()
{
  
  
  
  readSensor(); 
  //printSensors(); 
  Serial.print("move once-----------------");
 // move_single_cell(); 
  pid();
  delay(10);
  //turn_right(); 
  //delay(1000); 
  //about_face(); 
  //turn_left(); 
  //delay(1000); 
  //turn_right(); 
  //delay(1000); 


  /*
   
   // subtract the last reading:
   total= total - readings[smoothingIndex];         
   // read from the sensor:  
   readings[smoothingIndex] = readLeft(); ; 
   // add the reading to the total:
   total= total + readings[smoothingIndex];       
   // advance to the next position in the array:  
   smoothingIndex =smoothingIndex + 1;                    
   
   // if we're at the end of the array...
   if (smoothingIndex >= numReadings)              
   // ...wrap around to the beginning: 
   smoothingIndex = 0;                           
   
   // calculate the average:
   average = total / numReadings;         
   // send it to the computer as ASCII digits
   Serial.println(average);   
   delay(5);        // delay in between reads for stability 
   */

/*
  //* trip from start to goal 
  while(!found_dest){
    readSensor(); 
    visit_node(my_maze, my_stack, x, y, FALSE);
    change_dir(my_maze, &x, &y, &direction); //find the best direction to face based on flood values and turn to that direction
    move_single_cell(); //move one cell in the new direction

    check_goal_reached(&x, &y, &found_dest);

  }

  goal_x = x;
  goal_y = y;

  // Read walls of Center Cells 
  visit_node(my_maze, my_stack, x, y, FALSE);
  set_center_walls(x, y);
  delay(200);
  reflood_from_goal();
  found_dest = FALSE;
  about_face();
  
  ///* trip from GOAL to START 
  while(!found_dest){
    readSensor(); 
    visit_node(my_maze,my_stack,x,y,TRUE);
    change_dir(my_maze,&x,&y,&direction);
    move_single_cell();
    check_start_reached(&x,&y,&found_dest);
  }

*/
}

void readSensor(void) {

  int curt = micros();//record the current time

  writeLeftFront();//this is not sudo code, this is macro I defined somewhere else

  while((micros()-curt)<60);//use up time until 60us from where we record curt

  LFSensor =readLeftFront(); 

  stopLeftFront();//turn off emitter right after receive   r done ADC converting

  //do linear regression here for left front sensor if you plan to linearize your sensor

  while((micros()-curt)<140);//140-60=80us,wait 80us until reflection is gone

  writeRightFront();

  while((micros()-curt)<200);//200-140=60us, turn on emitter for 60us

  RFSensor=readRightFront();

  stopRightFront(); 

  //do linear regression here for right front sensor if you plan to linearize your sensor

  while((micros()-curt)<280);//280-200=80us

  writeSides();//turn on side emitters for side sensors

    while((micros()-curt)<340);//340-280=60us

  leftSensor  = readLeft(); 

  rightSensor = readRight(); 

  stopSides(); 

  //do linear regression here for side sensors if you plan to linearize your sensors

}

void printSensors(){
  Serial.print("Time: "); 
  Serial.println(micros());
  Serial.print("IR left diag: ");
  Serial.println(leftSensor);
  Serial.print("IR right diag: "); 
  Serial.println(rightSensor); 
  Serial.print("IR left front: ");
  Serial.println(LFSensor);
  Serial.print("IR right front: "); 
  Serial.println(RFSensor);
}
void readDistance(){
  Serial.print("Right encoder: "); 
  Serial.println(R_encoder_val); 
  Serial.print("Left encoder: "); 
  Serial.println(L_encoder_val);  

}
void left_interrupt()
{

  ++L_encoder_val;

  //Serial.print("interrupt working"); 

  /*
  if( digitalRead(LenchA) == LOW ) 
   Serial.print("A LOW"); 
   else
   Serial.print("A HIGH"); 
   if( digitalRead(LenchB) == LOW )
   Serial.print("B LOW"); 
   else
   Serial.print("B HIGH"); 
   */
}

void right_interrupt()
{
  //Serial.print("interrupt"); 

  ++R_encoder_val;

}



// READ SENSORS
int readLeft(){
  return analogRead(L_Receiver);
}
int readRight(){
  return analogRead(R_Receiver);
}
int readLeftFront(){
  return analogRead(LF_Receiver);
}
int readRightFront(){
  return analogRead(RF_Receiver);
}


short check_left_wall() {
  if (readLeft() > LEFT_WALL_SENSED)//need to adjust value
    return TRUE;

  return FALSE;
}

short check_right_wall() {
  if (readRight() > RIGHT_WALL_SENSED)//need to adjust value  
    return TRUE;

  return FALSE;
}

short check_front_wall() {
  if (readLeftFront() > FRONT_WALL_SENSED && readRightFront() > FRONT_WALL_SENSED)
    return TRUE;

  return FALSE;
}

// WRITE SENSORS
void writeSides(){
  digitalWrite(SIDE_HIGH_POWER, HIGH);
}
void writeSidesLow(){
  digitalWrite(SIDE_LOW_POWER, HIGH);
}
void writeLeftFront(){
  digitalWrite(LF_Emitter, HIGH);
}
void writeRightFront(){
  digitalWrite(RF_Emitter, HIGH);
}
void stopLeftFront(){
  digitalWrite(LF_Emitter, LOW);  
}
void stopRightFront(){

  digitalWrite(RF_Emitter, LOW );  
}
void stopSides(){
  digitalWrite(SIDE_HIGH_POWER, LOW );  
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

void writeEmitters(){

  writeSides(); 
  writeLeftFront(); 
  writeRightFront(); 

}

void readSensors(){
  Serial.print("Time: "); 
  Serial.println(time++);
  Serial.print("IR left diag: ");
  Serial.println(readLeft());
  Serial.print("IR right diag: "); 
  Serial.println(readRight()); 
  Serial.print("IR left front: ");
  Serial.println(readLeftFront());
  Serial.print("IR right front: "); 
  Serial.println(readRightFront());
}
void readEnc(){

  //rightenc = analogRead(RenchA); 
  Serial.print("Time: "); 
  Serial.println(time++); 

  Serial.print("Right EncA: ");
  Serial.println(digitalRead(RenchA));

  //rightencb = analogRead(RenchB); 
  Serial.print("Right  EncB: ");
  Serial.println(digitalRead(RenchB)); 
  Serial.println(" "); 

  //leftenca = analogRead(LenchA); 
  Serial.print("Left EncA: ");
  Serial.println(digitalRead(LenchA));

  //leftencb = analogRead(LenchB);  
  Serial.print("Left EncB: ");
  Serial.println(digitalRead(LenchB));
  Serial.println(" ");
  Serial.println(" ");
  /*
   Serial.print("Time: "); 
   Serial.println(time++);
   Serial.print("Right Encoder: "); 
   Serial.println(R_encoder_val); 
   Serial.print("Left Encoder: "); 
   Serial.println(L_encoder_val);
   */

}


void about_face()  // because, why not?
{
  delay(2000); 

  int value = R_encoder_val;

  analogWrite(L_fwd, LOW);
  analogWrite(L_bkw, 70);
  analogWrite(R_fwd, 70);
  analogWrite(R_bkw, LOW);



  while(R_encoder_val - value < ABOUT_FACE_COUNT);  // *********increase value to turn more***********

  analogWrite(L_bkw, LOW);
  analogWrite(R_fwd, LOW);
  R_encoder_val = 0;
  L_encoder_val = 0;


}


void turn_left() // point turn
{
  int encoder_number = L_encoder_val;

  analogWrite(R_fwd, LOW);
  analogWrite(L_fwd, LOW);
  analogWrite(R_bkw, LOW);
  analogWrite(L_bkw, LOW);

  delay(1000);  // decrease delay if mouse pauses too much, increase it if the mouse tries to turn
  // before slowing down enough (same thing in turn_right)

  analogWrite(R_fwd, 70);

  analogWrite(L_bkw, 70); 

  //delay(400);
  while(L_encoder_val - encoder_number < TURN_LEFT_COUNT );  // tune this value for complete turn ************* ///////////////////

  analogWrite(R_fwd, LOW);
  analogWrite(L_bkw, LOW);
  R_encoder_val = 0;
  L_encoder_val = 0;
}

void turn_right()  // point turn
{
  int encoder_number = L_encoder_val;

  analogWrite(R_fwd, LOW);
  analogWrite(L_fwd, LOW);
  analogWrite(R_bkw, LOW);
  analogWrite(L_bkw, LOW);

  delay(1000);

  analogWrite(L_fwd, 70);

  analogWrite(R_bkw, 70);

  while(L_encoder_val - encoder_number < TURN_RIGHT_COUNT);
  //delay(400);  // tune this value for complete turn ******* ///////////////////

  analogWrite(L_fwd, LOW);
  analogWrite(R_bkw, LOW);
  R_encoder_val = 0;
  L_encoder_val = 0;
}


void move_single_cell() {

  //analogWrite(R_fwd, LOW);
  //analogWrite(L_fwd, LOW);
  //analogWrite(R_bkw, LOW);
  //analogWrite(L_bkw, LOW );
  L_encoder_val = 0;
  keep_moving = true;
  while(keep_moving){
    //Serial.println("move one working"); 
    //Serial.println(L_encoder_val); 
    //rightForward(70); 
    //leftForward(70);
    readSensor();
    pid();  
    if (L_encoder_val >= ONECELL)  {

      keep_moving = false;
      R_encoder_val = 0;
      L_encoder_val = 0;
      
      analogWrite(R_fwd, LOW);
      analogWrite(L_fwd, LOW);
      analogWrite(R_bkw, LOW);
      analogWrite(L_bkw, LOW);
      //Serial.println("one cell moved. should stop now");
    }
  } 


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


void pid( void ) {
 
  readSensor(); 
  int totalError; 
  rightSense = rightSensor; 
  leftSense = leftSensor; 
  if( leftSense > hasLeftWall && rightSense > hasRightWall ) {
    //Serial.println("in the middle"); 
    errorP = rightSense - leftSense - newOffset;  
    errorD = errorP - oldErrorP;  
  } 
  else if( leftSense > hasLeftWall ){
    //Serial.println("only left wall"); 
    errorP = .1 * ( 80 - leftSense ); 
    errorD = errorP - oldErrorP; 
  }
  else if( rightSense > hasRightWall ){
   // Serial.println("only right wall"); 
    errorP = .1 * (rightSense - 80 ); 
    errorD = errorP - oldErrorP;  
  }
  //Serial.print("Error P : "); 
  //Serial.println(errorP); 
  
  totalError = P * errorP + D * errorD; 
  oldErrorP = errorP; 
  //Serial.print("P: "); 
  //Serial.println(P); 
  //Serial.print("Total Error: ");
  //Serial.println(totalError); 
  int rightSpeed = rightBaseSpeed + totalError; 
  int leftSpeed = leftBaseSpeed - totalError; 
  
  
  if( rightSpeed > 50 || rightSpeed < -50 )
    rightSpeed = rightSpeed/2 -5 ; 
  if( leftSpeed > 50 || leftSpeed < -50 )
    leftSpeed = leftSpeed/2 - 5 ; 
  
  if(RFSensor > 500 || LFSensor > 500 ){
    
    rightSpeed = 0; 
    leftSpeed = 0;  
  }
  /*
  Serial.print("Right speed: "); 
  Serial.println(rightSpeed); 
  Serial.print("Left speed: "); 
  Serial.println(leftSpeed); 
  */
  rightForward( rightSpeed ); 
  leftForward(leftSpeed ); 
  //Serial.println("working"); 
  
  
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
  //Serial.print("total error: "); 
  //Serial.println(total); 
  rightForward(15+total); 
  leftForward(25-total); 

  if( error == 0 ){
    //Serial.print(" Mouse is straight: ");
    //Serial.println(error);

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

/** Function: change_dir
 * Parameters: this_maze - the maze with flood values
 * x,y - current mouse coordinates
 * dir - current direction mouse is facing.
 * Description: makes the mouse face new direction. updates the new coordinates of the mouse.
 */
void change_dir ( Maze * this_maze, short * x, short * y, short * dir){

  Node * this_node;
  short next_dir;//new direction to face

  this_node = this_maze->map[(*x)][(*y)];
  next_dir = get_smallest_neighbor_dir(this_node, *dir);

  /* update the appropriate location value x or y */
  if (next_dir == NORTH) 
    (*x) = (*x) - 1;
  else if (next_dir == EAST) 
    (*y) = (*y) + 1;
  else if (next_dir == SOUTH) 
    (*x) = (*x) + 1;
  else if (next_dir == WEST) 
    (*y) = (*y) - 1;

  // Turn the actual mouse 
  if (*dir == NORTH) {
    if (next_dir == WEST)
      turn_left();
    else if (next_dir == EAST)
      turn_right();
    else if (next_dir == SOUTH)
      about_face();
  }

  else if (*dir == EAST) {
    if (next_dir == NORTH)
      turn_left();
    else if (next_dir == SOUTH)
      turn_right();
    else if (next_dir == WEST)
      about_face();
  }

  else if (*dir == SOUTH) {
    if (next_dir == EAST)
      turn_left();
    else if (next_dir == WEST)
      turn_right();
    else if (next_dir == NORTH)
      about_face();
  }

  else if (*dir == WEST) {
    if (next_dir == SOUTH)
      turn_left();
    else if (next_dir == NORTH)
      turn_right();
    else if (next_dir == EAST)
      about_face();
  }

  /* update the direction */
  (*dir) = next_dir;


}//end change_dir

/** Function: visit_node
 * Parameters: this_maze - maze with flood values
 * this_stack - stack for flood fill
 * x,y - coordinates to be visited
 * flag - whether to update goal cells or not
 * Description: visits the cell, checks for walls, and updates flood values
 */
void visit_node(Maze * this_maze, Stack * this_stack, short x, short y, short flag) {

  Node * this_node;
  short wall_on_left, wall_on_front, wall_on_right;
  short northwall, eastwall, southwall, westwall;

  this_node = this_maze->map[x][y];
  wall_on_left = wall_on_front = wall_on_right = FALSE;
  northwall = eastwall = southwall = westwall = FALSE;

  // read walls and set
  // read left
  wall_on_left = check_left_wall();
  wall_on_front = check_front_wall();
  wall_on_right = check_right_wall();  


  if (direction == NORTH) {

    if (wall_on_left) {
      set_wall(this_node, WEST);
      westwall = TRUE; 
    }
    if (wall_on_front) {
      set_wall(this_node, NORTH);
      northwall = TRUE;
    }
    if (wall_on_right) {
      set_wall(this_node, EAST); 
      eastwall = TRUE;
    }  
  }
  else if (direction == EAST){

    if (wall_on_left) {
      set_wall(this_node, NORTH);
      northwall = TRUE; 
    }
    if (wall_on_front) {
      set_wall(this_node, EAST);
      eastwall = TRUE;
    }
    if (wall_on_right) {
      set_wall(this_node, SOUTH);
      southwall = TRUE;
    }
  }
  else if (direction == SOUTH) {

    if (wall_on_left) {
      set_wall(this_node, EAST);
      eastwall = TRUE; 
    }
    if (wall_on_front) {
      set_wall(this_node, SOUTH);
      southwall = TRUE;
    }
    if (wall_on_right) {
      set_wall(this_node, WEST); 
      westwall = TRUE;
    }
  }
  else {

    if (wall_on_left) {
      set_wall(this_node, SOUTH);
      southwall = TRUE; 
    }
    if (wall_on_front) {
      set_wall(this_node, WEST);
      westwall = TRUE;
    }
    if (wall_on_right) {
      set_wall(this_node, NORTH); 
      northwall = TRUE;
    }
  }

  if (northwall) {
    if (this_node->row != 0)
      push (this_stack, MAP[ROW-1][COL]);
    set_wall(this_node, NORTH);
  }
  if (eastwall) {
    if (this_node->column != SIZE-1)
      push (this_stack, MAP[ROW][COL+1]);
    set_wall(this_node, EAST);
  }
  if (southwall) {
    if (this_node->row != SIZE-1)
      push (this_stack, MAP[ROW+1][COL]);
    set_wall(this_node, SOUTH);
  }
  if (westwall) {
    if (this_node->column != 0)
      push (this_stack, MAP[ROW][COL-1]);
    set_wall(this_node, WEST);
  }

  /* push this node itself, as it was updated */
  push(this_stack, this_node);

  /* pop until the stack is empty, and call flood_fill on that node */
  while (!is_empty_Stack(this_stack)) {
    pop(this_stack, &this_node);
    /* NOTE: the flag parameter determines wheter to update goal cells or not */
    flood_fill(this_node, this_stack, flag);
  }

  set_visited (this_node);
}//end visit_node

/** Function: check_goal_reached
 * Parameters: x,y - coordinate to be checked
 * found_goal - flag if goal cell was found or not
 * Description: updates flag for whether goal cell was reached
 */
void check_goal_reached (short * x, short * y, short * found_goal) {

  if (*x == SIZE / 2 || *x == SIZE / 2 - 1) {
    if (*y == SIZE / 2 || *y == SIZE / 2 - 1) {
      *(found_goal) = TRUE;
    }
  }
}
/* update flag for whether goal cell was reached */
void check_start_reached (short * x, short * y, short * found_start) {

  if (*x == START_X && *y == START_Y) {
    *(found_start) = TRUE;
    //printf("Start Coorinates Reached: %d, %d\n", *x, *y);
  }
}


/**Function: set_center_walls
   Description: fills in the wall values of the centers based on where
                you discovered the goal from.
*/
void set_center_walls(short entered_x, short entered_y) {

  // 8, 8 : NORTH or WEST
  if (entered_x = SIZE/2 && entered_y == SIZE/2) {

    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     EAST);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], WEST);
  }

  // 8, 7 : NORTH or EAST
  if (entered_x = SIZE/2 && entered_y == SIZE/2 - 1) {

    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     EAST);
    set_wall(my_maze->map[SIZE/2][SIZE/2], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2], EAST);
  }

  // 7, 7 : SOUTH or EAST
  if (entered_x = SIZE/2 - 1 && entered_y == SIZE/2 - 1) {

    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     EAST);
    set_wall(my_maze->map[SIZE/2][SIZE/2], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2], EAST);
  }


  // 7, 8 : SOUTH or WEST
  if (entered_x = SIZE/2 - 1 && entered_y == SIZE/2) {

    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2][SIZE/2], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2], EAST);
  }

}

/** Function: reflood_from_goal
    Description: Resets all the flood values so that the goal is now the start without modifying wall values.
*/
void reflood_from_goal() {
 
  for (int i = 0; i < SIZE; i++) 
      for (int j = 0; j < SIZE; j++)
        my_maze->map[i][j]->floodval = LARGEVAL;
      
    /* set the start value to zero */
    set_value(my_maze->map[START_X][START_Y], 0);

    /* push the neighbors of start cell to stack 
       then pop everything until all cells updated*/
    push_open_neighbors(my_maze->map[START_X][START_Y], my_stack);
    while(!is_empty_Stack(my_stack)) {
      pop(my_stack, &temp);
      if (!(temp->row == 15 && temp->column == 0))
        flood_fill(temp, my_stack, TRUE);
    }
  
}

