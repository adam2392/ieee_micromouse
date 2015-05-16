/* IEEE Micromouse Main Driver File:
*/
#include <avr/io.h>

#include "Maze.c"
#include "Maze.h"
#include "Stack.c"
#include "Sensor.h"
#include "Drive.h"
#include <LedDisplay.h>
#include <avr/interrupt.h>
//Define Driving Direction
#define BACKWARD 0
#define FORWARD 1

//LED Pin Definitions for debugging
#define led1 26
#define led2 27
#define led3 28

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
#define speaker 2

//LCD Display
#define dataPin 12      //connect to displays data input
#define registerSelect 1 //the display's register select pin
#define clockPin 15  //the display's clock pin
#define  enable 11      //The display's chip enable pin
#define  reset 24       //the display's reset pin
#define  displayLength 4   //number of characters in the display

LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin, enable, reset, displayLength);
int brightness = 15;        // screen brightness

// Directions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//constants for encoder to turn right/left/180
#define TURN_RIGHT_COUNT 1900      //1900 for fake maze
#define TURN_LEFT_COUNT 1850      //1850 for fake maze +/- 50
#define TURN_AROUND_COUNT 3000    //3000 for fake maze
#define ONECELL 7100              //7200 for fake maze

//constants for wall sensing left/right/front
#define hasLeftWall 190 //190 for fake maze
#define hasRightWall 220  //220 for fake maze; 120 for real maze
#define hasFrontWall 300  //300 for fake maze; 

#define PIDSTOP 600    //600 for fake maze;
#define LEFTPID 200    //200 for fake maze
#define RIGHTPID 350  //350 for fake maze
//maybe define new ones for wall sensing for the floodfill


short frontwallflag; //global flag that is set when PID finds a front wall
short noWall;  //global flag to detect scenario with no walls

int ran = 0; 
int errorP = 0;
int errorD = 0; 
int oldErrorP = 0; 
int newOffset = 15;     // increase to go right
int rightBaseSpeed = 35; //35 old values
int leftBaseSpeed = 32;//32

//PD values
double P = 0.25; 
double D = 0.3; 

volatile int R_encoder_val = 0;  // declare encoder interrupt values
volatile int L_encoder_val = 0;

const int numReadings = 500;

int readings[numReadings];      // the readings from the analog input
int smoothingIndex = 0;         // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

// the setup routine runs once when you press reset:
/**** maze solving variables ****/
struct Maze * my_maze; //maze that keeps track of flood fill values and walls
struct Stack * my_stack; //stack used for flood fill
struct Node * temp; //node used for in-between start->goal, goal->start transition
short found_dest; //flag if goal is reached
short direction; //direction that the mouse is facing
short x, y; //current coordinates of the mouse in the maze

short goal_x, goal_y; //goal coordinates once found


// Sensor initializations
Sensor sensor = Sensor(LF_Emitter, LF_Receiver, RF_Emitter, 
        RF_Receiver, SIDE_HIGH_POWER, SIDE_LOW_POWER,
        R_Receiver, L_Receiver);
int leftSensor = 0; 
int rightSensor = 0; 
int LFSensor = 0; 
int RFSensor = 0; 
int oldLeftSensor = 0;
int oldRightSensor = 0;

//to start the mouse with a hand wave
short leftSeen = false; 
short rightSeen = false; 

// Drive initializations
//Pin definitions for motor
const int R_fwd = 6, R_bkw = 5;  //R_fwd = 1 and R_bkw = 0 -> go forward on rightside
const int L_fwd = 4, L_bkw = 3;  //L_fwd = 1 and L_bkw = 0 -> go forward on leftside

Drive drive = Drive(R_fwd, R_bkw, L_fwd, L_bkw); 

// Setup: the setup routine runs once when you press reset:
void setup()
{
  Serial.begin(9600);     //initialize for serial output
 
  myDisplay.begin();                     // initialize the display library:
  myDisplay.setBrightness(brightness);  // initialize the display library:
  
  // initialize left and right encoders
  pinMode(RenchA, INPUT); 
  pinMode(RenchB, INPUT);
  pinMode(LenchA, INPUT); 
  pinMode(LenchB, INPUT); 

  //power for the receivers *leave as is DO NOT CHANGE*
  pinMode(A14, OUTPUT);  
  analogWrite(A14, 255);

  //setup sensor pins for output and input
  pinMode(LF_Emitter, OUTPUT);  //initializes leftfront emitter to output stuff
  pinMode(RF_Emitter, OUTPUT);
  pinMode(LF_Receiver, INPUT);  //initialize leftfront receiver to receive input from stuff
  pinMode(RF_Receiver, INPUT);
  pinMode(SIDE_HIGH_POWER, OUTPUT);
  pinMode(L_Receiver, INPUT);
  pinMode(R_Receiver, INPUT);

  attachInterrupt(LenchA, left_interrupt, CHANGE);  // may need to adjust encoder operation
  attachInterrupt(LenchB, left_interrupt, CHANGE);
  attachInterrupt(RenchA, right_interrupt, CHANGE);
  attachInterrupt(RenchB, right_interrupt, CHANGE);  // comment out interrupts not used

  //initialize motor pins to off
  pinMode(R_fwd, OUTPUT);  //initialize rightmotor forward as output
  pinMode(R_bkw, OUTPUT);
  pinMode(L_fwd, OUTPUT);
  pinMode(L_bkw, OUTPUT);

  //initialize debug LEDs
  pinMode(led1, OUTPUT);  //setup led1 for output
  pinMode(led2, OUTPUT);  //setup led2 for output
  pinMode(led3, OUTPUT);  //setup led3 for output

  //setup the speaker pin
  pinMode(speaker, OUTPUT);
  ran = 0; 

  /// what are these for?
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
  
  frontwallflag = 0;
  noWall = 0;
}

// the loop routine runs over and over again forever:
void loop()
{  
  myDisplay.home();          // set the cursor to 0:
  
  
  //swipe
  while(!leftSeen || !rightSeen )
  {
    Serial.println(leftSensor); 
    readSensor(); 
    if( leftSensor > 600 )
      leftSeen = true;
    if( rightSensor > 600 ) 
      rightSeen = true; 
  }
  delay(400);   
  
  while(1){
    int temp = 0;
    while(!temp)
    {
      move_single_cell();
      delay(1000);
      temp = check_front_wall();
    }
    turn_left();
    delay(1000);
  }
  



//  while the center destination hasn't been reached
//  while(!found_dest){
//    Serial.println("start"); 
//     readSensor();          // read sensors
//     
//     visit_next_node(my_maze,my_stack,x,y,TRUE); //visit node in front to detect LEFT/RIGHTwalls
//     visit_node(my_maze,my_stack,x,y,TRUE); //visit current node to detect FRONT walls
//     delay(100);
//     
//     change_dir(my_maze,&x,&y,&direction); //turns the mouse to face the direction of best path. updates location (x,y)
//     delay(100); 
//     move_single_cell();   // move a single cell forward in direction chosen by change_dir
//     
//
//     check_goal_reached(&x, &y, &found_dest); //neccesary to know if mouse should stop, will set found_dest flag
//     
//     lcd_display_location(); //Debugging: display current location of mouse
//     print_map_serial(my_maze); //Debugging: prints out flood values of the maze to Serial port
//     Serial.println("end"); 
//  }

  ////redo floodfill

  ////speed runs?
}

/*Function: pid
  Description: Essentially helps control mouse ONLY when moving forward in the desired direction
  Uses other functions to turn mouse in that direction first!
*/
void pid( void ) {
  readSensor();
  int totalError=0;   // the total error for the wall
  
  short leftWall = check_left_wall();
  short rightWall = check_right_wall();
  
  if ((RFSensor > PIDSTOP && LFSensor > PIDSTOP)) //prevents mouse from crashing into wall due to errors.
  {                                        //if this is called, its a BAD sign....
    motorbreak();
    frontwallflag = 1;
    return;
  }
  
  // compare sensor readings with constants for the wall -> if true it is in the middle
  if(leftWall && rightWall) {
//    Serial.println("in the middle"); 
    errorP = rightSensor - leftSensor - newOffset;  //how far away from middle we are
    errorD = errorP - oldErrorP;                  //change in error  
  }
  // sensor only reads in left wall 
  else if(leftWall){
//    Serial.println("only left wall"); 
    errorP = .15 * (LEFTPID - leftSensor);      // old: 200; the error away from center with only a left wall
    errorD = errorP - oldErrorP;           // change in error
  }
  // sensor only reads in right wall
  else if(rightWall){
//    Serial.println("only right wall"); 
    errorP = .15 * (rightSensor - RIGHTPID);      //old: 350 the error away from center with only a right wall
    errorD = errorP - oldErrorP;           // change in error
  }
  // no walls detected except for maybe in front
  else { 
    int value = R_encoder_val;
    drive.rightForward(40);
    drive.leftForward(40);
    
    return; 
     
    noWall = 1;
    
    //make it go 1 cell
    while(R_encoder_val - value < ONECELL);
    
    analogWrite(R_fwd, HIGH);
    analogWrite(L_fwd, HIGH);
    analogWrite(R_bkw, HIGH);
    analogWrite(L_bkw, HIGH);
  }
  totalError = P * errorP + D * errorD; // total error = P + D controller type
  oldErrorP = errorP;                   // re initialize oldError 
  
  //initialize motor speeds based on error, and what we set as the base speed
  int rightSpeed = rightBaseSpeed + totalError; 
  int leftSpeed = leftBaseSpeed - totalError;
//  Serial.println(rightBaseSpeed);
 

  //Lucas: This prevents jerking
//  if( rightSpeed > 50 || rightSpeed < -50 )
//    rightSpeed = rightSpeed/2 -5 ; 
//  if( leftSpeed > 50 || leftSpeed < -50 )
//    leftSpeed = leftSpeed/2 - 5 ; 
//  Serial.println(rightSpeed);
  
  // move right/left motor forward
  drive.rightForward(rightSpeed); 
  drive.leftForward(leftSpeed); 
}

/*Function: readSensor
  Description: Read in left-front, right-front and the right/left sensors
  and record the distances into the vars:
  1. LFSensor
  2. RFSensor
  3. rightSensor
  4. leftSensor
*/
void readSensor(void) {
  int curt = micros();//record the current time
  
  oldLeftSensor = leftSensor;
  oldRightSensor = rightSensor;
  
  // read in the left front sensor and then stop emitter
  sensor.writeLeftFront();//this is not sudo code, this is macro I defined somewhere else
  while((micros()-curt)<60); //use up time until 60us from where we record curt to read another sensor 
  LFSensor = sensor.readLeftFront(); //read the leftFront sensor
  sensor.stopLeftFront();//turn off emitter right after receive   r done ADC converting
  
  // read in right front sensor and then stop emitter
  while((micros()-curt)<140);//140-60=80us,wait 80us until reflection is gone
  sensor.writeRightFront();
  while((micros()-curt)<200);//200-140=60us, turn on emitter for 60us
  RFSensor = sensor.readRightFront();
  sensor.stopRightFront(); 
  
  //do linear regression here for right front sensor if you plan to linearize your sensor
  
  // read in side sensors and then stop emitters
  while((micros()-curt)<280);//280-200=80us
  sensor.writeSides();//turn on side emitters for side sensors
  while((micros()-curt)<340);//340-280=60us 
  leftSensor  = sensor.readLeft(); 
  rightSensor = sensor.readRight(); 
  
  //apply moving average of 2
  leftSensor = (leftSensor + oldLeftSensor)/2;
  rightSensor = (rightSensor + oldRightSensor)/2;
  
  sensor.stopSides(); 
}

/* Sensor readings for checking if there is a wall on left/right/front */
short check_left_wall() {
  if ((leftSensor > hasLeftWall)){
    return TRUE;
  }
  return FALSE;
}
short check_right_wall() {
  if ((rightSensor > hasRightWall)){//need to adjust value  
    
    return TRUE;
  }
  return FALSE;
}
short check_front_wall() {
  if ((RFSensor > hasFrontWall && LFSensor > hasFrontWall) ){ //&& !(RFSensor < 200 || LFSensor < 200)

    test_tone(); 
    return TRUE;
  }
  return FALSE;
}

// Shouldn't touch this.... These are encoder functions that are hardware implemented
// to increase encoder value with every turn of the wheel
void left_interrupt()
{
  ++L_encoder_val;
}
void right_interrupt()
{
  ++R_encoder_val;  
}

/* Function: turn_left
Description: Should make the mouse turn left 90 degrees without moving forward
 ** or make it turn left and move forward 1 cell? (doubtful because it could be error prone)
*/
void turn_left()
{
  Serial.println("turning left!");
  int encoder_number = L_encoder_val;  // get the left encoder value at the beginning of when we want to turn
  
  //turn off all motors
  analogWrite(R_fwd, LOW);
  analogWrite(L_fwd, LOW);
  analogWrite(R_bkw, LOW);
  analogWrite(L_bkw, LOW);
  
  delay(100);  // decrease delay if mouse pauses too much, increase it if the mouse tries to turn
  	       // before slowing down enough (same thing in turn_right)  
  // make right side go forward, left side go backwards -> turn left
  analogWrite(R_fwd, 100); 
  analogWrite(L_bkw, 100); 
  
  // don't stop the analogWrite until mouse has completed a left turn
  while(L_encoder_val - encoder_number < TURN_LEFT_COUNT );  // tune this value for complete turn ************* ///////////////////
  
  //break motors
  drive.leftForward(40);
  drive.rightBackward(40);
  delay(50);
  drive.leftForward(0);
  drive.rightBackward(0);
  
  delay(300);
  
  // why are we setting it to 0?
  R_encoder_val = 0;
  L_encoder_val = 0;
}

/* Function: turn_right
Description: Should make the mouse turn right 90 degrees without moving forward

***Why are we using L_encoder_val
*/
void turn_right()  // point turn
{
  Serial.println("turning right........"); 
  int encoder_number = L_encoder_val;  //find the current right encoder value
  
  //turn off all motors at beginning
  analogWrite(R_fwd, LOW);
  analogWrite(L_fwd, LOW);
  analogWrite(R_bkw, LOW);
  analogWrite(L_bkw, LOW);
  
  delay(100);  //delay 1 second
  
  //turn left motor forward, and right motor back -> turn right
  analogWrite(L_fwd, 100);
  analogWrite(R_bkw,100);
  
   // don't stop the analogWrite until mouse has completed a right turn
  while(L_encoder_val - encoder_number < TURN_RIGHT_COUNT);
  //delay(100);  // tune this value for complete turn ******* ///////////////////
  
  //break motors
  drive.leftBackward(40);
  drive.rightForward(40);
  delay(50);
  drive.leftBackward(0);
  drive.rightForward(0);
  
  delay(300);
  
  R_encoder_val = 0;
  L_encoder_val = 0;
}

/* Function: turn_around
Description: Should make the mouse turn around 180 degrees without moving forward

***Why are we using L_encoder_val
*/
void turn_around()  // point turn
{
  Serial.println("turning around........"); 
  int encoder_number = L_encoder_val;  //find the current right encoder value
  
  //turn off all motors at beginning
  analogWrite(R_fwd, LOW);
  analogWrite(L_fwd, LOW);
  analogWrite(R_bkw, LOW);
  analogWrite(L_bkw, LOW);
  
  delay(100);  //delay 1 second
  
  //turn left motor forward, and right motor back -> turn around
  analogWrite(L_fwd, 100);
  analogWrite(R_bkw, 100);
  
   // don't stop the analogWrite until mouse has completed a 180 degree turn
  while(L_encoder_val - encoder_number < TURN_AROUND_COUNT);


  //break motors
  drive.leftBackward(40);
  drive.rightForward(40);
  delay(50);
  drive.leftBackward(0);
  drive.rightForward(0);
  
  delay(300); // tune this value for complete turn ******* ///////////////////
  
  R_encoder_val = 0;
  L_encoder_val = 0;
}

/* Function: move_single_cell
Description: Should make the mouse move forward in the direction of 1 cell
*/
void move_single_cell() {
  //initialize encoder values to 0
  R_encoder_val = 0;
  L_encoder_val = 0;
  frontwallflag = 0;
  
  //while the left encoder has not moved 1 "full" cell yet
  while(L_encoder_val <= ONECELL && !frontwallflag)
  {
    readSensor();    //read in sensors to update sensor readings
    pid();           //call pid to make sure it is going straight based on sensor readings
  }
  
  //stop fucntions after finished moving 1 cell
  motorbreak();
  
}

/** Function: change_dir ****ONLY FUNCTION THAT TOUCHES THE MAIN PROGRAM*****
 * Parameters: this_maze - the maze with flood values
 * x,y - current mouse coordinates
 * dir - current direction mouse is facing.
 * Description: makes the mouse face new direction. updates the new coordinates of the mouse.
 */
void change_dir ( Maze * this_maze, short * x, short * y, short * dir){
  
  Node * this_node;  //initialize a node
  short next_dir;    //new direction to face after performing floodfill calculation

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

  // Turn the actual mouse in the optimal direction
  if (*dir == NORTH) {
    if (next_dir == WEST)
      turn_left();
    else if (next_dir == EAST)
      turn_right();
    else if (next_dir == SOUTH)
      turn_around();
  }

  else if (*dir == EAST) {
    if (next_dir == NORTH)
      turn_left();
    else if (next_dir == SOUTH)
      turn_right();
    else if (next_dir == WEST)
      turn_around();
  }

  else if (*dir == SOUTH) {
    if (next_dir == EAST)
      turn_left();
    else if (next_dir == WEST)
      turn_right();
    else if (next_dir == NORTH)
      turn_around();
  }

  else if (*dir == WEST) {
    if (next_dir == SOUTH)
      turn_left();
    else if (next_dir == NORTH)
      turn_right();
    else if (next_dir == EAST)
      turn_around();
  }

  /* update the direction */
  (*dir) = next_dir;
  delay(200);
}//end change_dir

/** Function: visit_node
 * Parameters: this_maze - maze with flood values
 * this_stack - stack for flood fill
 * x,y - coordinates to be visited
 * flag - whether to update goal cells or not
 * Description: visits the cell, checks for walls, and updates flood values
 */
void visit_node(Maze * this_maze, Stack * this_stack, short x, short y, short flag) {
 
  readSensor();
  Node * this_node;                                  //initialize a node
  short northwall, eastwall, southwall, westwall;    //boolean values of wall on each side

  this_node = this_maze->map[x][y];                  //initialize node to the node we want to go to
  northwall = eastwall = southwall = westwall = FALSE;

  //correspondingly update the walls based on which direction we are facing
  if (direction == NORTH) {    //check if direction is currently facing NORTH
    if (check_front_wall()) {    //there is a front wall
      set_wall(this_node, NORTH);
      northwall = TRUE;
    }
  }
  else if (direction == EAST){  //check if direction is currently facing EAST
    if (check_front_wall()) {
      set_wall(this_node, EAST);
      eastwall = TRUE;
    }
  }
  else if (direction == SOUTH) {
    if (check_front_wall()) {
      set_wall(this_node, SOUTH);
      southwall = TRUE;
    }
  }
  else {//direction we are facing is WEST
    if (check_front_wall()) {
      set_wall(this_node, WEST);
      westwall = TRUE;
    }
  }

//  Serial.print("Row"); 
//  Serial.println(ROW); 
//  Serial.print("Col"); 
//  Serial.println(COL); 

  /* If there is a wall -> do a series of checks and pushes onto the stack */
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


/** Function: visit_node
 * Parameters: this_maze - maze with flood values
 * this_stack - stack for flood fill
 * x,y - coordinates to be visited
 * flag - whether to update goal cells or not
 * Description: visits the cell, checks for walls, and updates flood values
 */
void visit_next_node(Maze * this_maze, Stack * this_stack, short x, short y, short flag) {
  readSensor();
  if(check_front_wall()) //return if there is a wall, meaning that we cannot determine wall info about the
    return;              //next node in front of this one.
  Node * next_node = NULL;//node one cell ahead of direction faced.
  short northwall, eastwall, southwall, westwall;    //boolean values of wall on each side

  northwall = eastwall = southwall = westwall = FALSE;

  if( direction == NORTH && (y > 0) )
    next_node = this_maze->map[x][y-1];
  if( direction == EAST && (x < (SIZE - 1)) )
    next_node = this_maze->map[x+1][y];
  if( direction == SOUTH && (y < (SIZE-1)) )
    next_node = this_maze->map[x][y+1];
  if( direction == WEST && (x > 0) )
    next_node = this_maze->map[x-1][y];
    
  if( next_node == NULL ) //return if we are at the boundry of the maze.
    return;

  //correspondingly update the walls based on which direction we are facing
  if (direction == NORTH) {    //check if direction is currently facing NORTH
    if (check_left_wall()) {      //there is a left wall
      set_wall(next_node, WEST);  //set wall on the left
      westwall = TRUE;             
    }
    if (check_right_wall()) {        //there is a right wall
      set_wall(next_node, EAST); 
      eastwall = TRUE;
    }  
  }
  else if (direction == EAST){  //check if direction is currently facing EAST
    if (check_left_wall()) {
      set_wall(next_node, NORTH);
      northwall = TRUE; 
    }
    if (check_right_wall()) {
      set_wall(next_node, SOUTH);
      southwall = TRUE;
    }
  }
  else if (direction == SOUTH) {
    if (check_left_wall()) {
      set_wall(next_node, EAST);
      eastwall = TRUE; 
    }
    if (check_right_wall()) {
      set_wall(next_node, WEST); 
      westwall = TRUE;
    }
  }
  else {    //direction we are facing is WEST
    if (check_left_wall()) {
      set_wall(next_node, SOUTH);
      southwall = TRUE; 
    }
    if (check_right_wall()) {
      set_wall(next_node, NORTH); 
      northwall = TRUE;
    }
  }

  /* If there is a wall -> do a series of checks and pushes onto the stack */
  if (northwall) {
    if (next_node->row != 0)
      push (this_stack, MAP[(next_node->row)-1][next_node->column]);
    set_wall(next_node, NORTH);
  }
  if (eastwall) {
    if (next_node->column != SIZE-1)
      push (this_stack, MAP[next_node->row][(next_node->column)+1]);
    set_wall(next_node, EAST);
  }
  if (southwall) {
    if (next_node->row != SIZE-1)
      push (this_stack, MAP[(next_node->row)+1][next_node->column]);
    set_wall(next_node, SOUTH);
  }
  if (westwall) {
    if (next_node->column != 0)
      push (this_stack, MAP[next_node->row][(next_node->column)-1]);
    set_wall(next_node, WEST);
  }
  /* push this node itself, as it was updated */
  push(this_stack, next_node);

  /* pop until the stack is empty, and call flood_fill on that node */
  while (!is_empty_Stack(this_stack)) {
    pop(this_stack, &next_node);
    /* NOTE: the flag parameter determines wheter to update goal cells or not */
    flood_fill(next_node, this_stack, flag);
  }
  //set_visited (this_node);
}//end visit_next_node

/** Function: check_goal_reached
 * Parameters: x,y - coordinate to be checked
 * found_goal - flag if goal cell was found or not
 * Description: updates flag for whether goal cell was reached
 */
void check_goal_reached (short * x, short * y, short * found_goal) {
  //if the mouse is in the center of the maze -> it found it's goal
  if (*x == SIZE / 2 || *x == SIZE / 2 - 1) {  
    if (*y == SIZE / 2 || *y == SIZE / 2 - 1) {
      *(found_goal) = TRUE;
    }
  }
}

///* update flag for whether goal cell was reached */
//void check_start_reached (short * x, short * y, short * found_start) {
//
//  if (*x == START_X && *y == START_Y) {
//    *(found_start) = TRUE;
//    //printf("Start Coorinates Reached: %d, %d\n", *x, *y);
//  }
//}


/**Function: set_center_walls
   Description: fills in the wall values of the centers based on where
                you discovered the goal from.
*/
void set_center_walls(short entered_x, short entered_y) {

  // 8, 8 : NORTH or WEST
  if (entered_x == SIZE/2 && entered_y == SIZE/2) {

    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     EAST);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], WEST);
  }

  // 8, 7 : NORTH or EAST
  if (entered_x == SIZE/2 && entered_y == SIZE/2 - 1) {

    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     EAST);
    set_wall(my_maze->map[SIZE/2][SIZE/2], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2], EAST);
  }

  // 7, 7 : SOUTH or EAST
  if (entered_x == SIZE/2 - 1 && entered_y == SIZE/2 - 1) {

    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2 - 1], WEST);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     NORTH);
    set_wall(my_maze->map[SIZE/2 - 1][SIZE/2],     EAST);
    set_wall(my_maze->map[SIZE/2][SIZE/2], SOUTH);
    set_wall(my_maze->map[SIZE/2][SIZE/2], EAST);
  }


  // 7, 8 : SOUTH or WEST
  if (entered_x == SIZE/2 - 1 && entered_y == SIZE/2) {

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
      
    // set the start value to zero 
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

/** Function: break */
void motorbreak()
{
  drive.leftBackward(140);  //90 for real maze
  drive.rightBackward(140);  //90 for real maze
  delay(30);                //30 for real maze
  drive.leftBackward(0);
  drive.rightBackward(0);
}

void test_tone(void)
{
  tone(speaker, 700, 250);
}

void test_tone2(void)
{
  tone(speaker, 550, 250);
}

void test_tone3(void)
{
  tone(speaker, 1500, 250);
}





///////////////////
///// DEBUG ///////
///////////////////
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
  Serial.print("Offcenter: "); 
  Serial.println(RFSensor - LFSensor + 100); 
  
  
  
}

/* prints the flood values of each cell in the maze */
void print_map_serial (const Maze * this_maze) {

  short i, j;

  Serial.print("CURRENT MAP VALUES: \n");
  for (i = 0; i < SIZE; ++i) {
    for (j = 0; j < SIZE; ++j) {
      if(MAPIJ->floodval < 10)
        Serial.print(" ");
      Serial.print(MAPIJ->floodval);
      Serial.print(" ");
    } 
    Serial.print("\n");
  }
  Serial.print("\n");
}

//printing to the lcd screen
void lcd_display_location(void){
     //print current location:
     myDisplay.clear();
     myDisplay.home();
     //myDisplay.print("    ");//erase LCD screen
      
     //myDisplay.home();
     //myDisplay.print(" ");
     if(x<10){
       myDisplay.print(0); 
     }
     myDisplay.print(x);
     if(y<10){
       myDisplay.print(0);
     }
     myDisplay.print(y);
}
