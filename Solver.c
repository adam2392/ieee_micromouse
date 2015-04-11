#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>
#include "Maze.h"

/* update flag for whether goal cell was reached */
void check_start_reached (short * x, short * y, short * found_start) {

  if (*x == START_X && *y == START_Y) {
    *(found_start) = TRUE;
    printf("Start Coorinates Reached: %d, %d\n", *x, *y);
  }
}

/* update flag for whether goal cell was reached */
void check_goal_reached (short * x, short * y, short * found_goal) {

  if (*x == SIZE / 2 || *x == SIZE / 2 - 1) {
    if (*y == SIZE / 2 || *y == SIZE / 2 - 1) {
      *(found_goal) = TRUE;
      printf("Goal Coordinates Reached: %d, %d\n", *x, *y); 
    }
  }
}

/* function for updating the location and direction of mouse 
   the actual "move" function */
void move_dir (Maze * this_maze, short * x, short * y, short * dir) {

  /* x, y are current positions, dir is current directions
     these output params may be updated at the exit of this function */ 

  Node * this_node;   /* the node at this position x, y */ 
  short next_dir;       /* will hold the next direction to go */

  /* debug statements */
  if (get_debug_mode()) {
    printf("In move_dir\n");
    printf("preffered_dir: %d\n", *dir );
    printf("current coords: %d,%d\n", *x, *y);
  }

  this_node = this_maze->map[(*x)][(*y)];
  next_dir = get_smallest_neighbor_dir(this_node, *dir);
  
  if (get_debug_mode())
    printf("%d\n", next_dir);

  /* update the appropriate location value x or y */
  if (next_dir == NORTH) 
    (*x) = (*x) - 1;
  else if (next_dir == EAST) 
    (*y) = (*y) + 1;
  else if (next_dir == SOUTH) 
    (*x) = (*x) + 1;
  else if (next_dir == WEST) 
    (*y) = (*y) - 1;

  /* update the direction */
  (*dir) = next_dir;
}


/* the function that represents a visiting of a node 
   walls will be checked, and flood fill called apprepriately */
void visit_Node (Maze * this_maze, Stack * this_stack, short x, short y, 
                short wallval, char flag) {

  Node * this_node;   /* holds current node at x, y; also for reading from stack */
  int northwall, eastwall, southwall, westwall;  /* for reading in wall data */

  this_node = this_maze->map[x][y];
  northwall = eastwall = southwall = westwall = 0;

  /* debug statements */
  if (get_debug_mode()) {
    printf("In visit_Node\n");
    printf("wallval: %d\n", wallval);
  }

  /* reading the existence of walls in each direction, according to wall data 
     in real mouse, walls will be checked by sensor */
  if (wallval / 8 == TRUE) {
    westwall = TRUE;
    wallval -= 8;
  }
  if (wallval / 4 == TRUE) {
    southwall = TRUE;
    wallval -= 4;
  }
  if (wallval / 2 == TRUE) {
    eastwall = TRUE;
    wallval -= 2;
  }
  if (wallval / 1 == TRUE) {
    northwall = TRUE;
  }

  /* debug statements */
  if (get_debug_mode()) {
    printf("N,E,S,W : %d, %d, %d, %d\n", northwall, eastwall, southwall, westwall);
  }

  /* push to stack the cell on other side of wall if valid 
     sets the walls as specified by the values checked above */
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

}


/* subroutine used for reading in characters for solver simulator */
int getVal(char c)
{

  int rtVal = 0;

  if(c >= '0' && c <= '9')
      rtVal = c - '0';
  
  else
    rtVal = c - 'a' + 10;
  
  return rtVal;

}

void press_enter_to_continue(){
  printf("press enter to continue...\n");
  while(getchar() == NEWLINE);
}


/* MAIN METHOD */
int main (int argc, char ** argv) {

  /* wall simulation values array and variables
     these are used for testing purposes only and will not be used on mouse */
  int Walls[SIZE][SIZE];   /* it will hold the values read from the wall data file */
  int opt;  /* checks for debug flag option -x */
  char ch;  /* character reading for press enter to continue */
  char file_name[25]; /* string to hold file name */
  FILE *fp; /* pointer to file that holds maze wall data */
  int exit_solver_loop;
  int trip_count;       /* counts the number of runs from start->goal->start */

  /* solver variables
     these are essential variables for solving the maze.. so will be used on mouse */
  Maze * my_maze;    /* maze for keeping track of flood values and walls */
  Stack * my_stack;  /* stack used for flood fill */
  short found_dest;   /* flag if goal is reached */
	short direction;    /* keeps track of direction that mouse is moving in */
	short x, y; /* keeps track of current row, col value mouse is in within maze */
  short goal_x, goal_y; /* keeps track of goal's x, y, once found */
  Node * temp;  /* used for in-between start->goal, goal->start transition */



  /******* This file reading part will not be used in actual solver ********/

  /* Sets debug mode according to presense of debug flag x 
     in command line argument */
	set_debug_off();
	while((opt = getopt(argc, argv, "x")) != EOF) 
		if(opt == 'x')
			set_debug_on();


  /* Open File attempt */
  printf("Enter the name of maze file: ");
  gets(file_name);
  fp = fopen(file_name, "r");

  /* if the file does not exist, exit program */
  if (fp == NULL) {
    perror("Error while opening file... program will exit.");
    exit(EXIT_FAILURE);
  }

  /* Reads maze file values and store them to appropriate array */
  for (int j = 0; j < SIZE; j++) 
    for (int i = SIZE-1; i >= 0; i--) 
      if((ch = fgetc(fp)) != EOF) {
        if (ch == ' ' || ch == '\n') {
          i++;
          continue;
        }
        int val = getVal(ch) * 16 + getVal(fgetc(fp));
        Walls[i][j] = val;
      }     
    
  /* close file */
  fclose(fp);


  /********* Actual maze solving part begins here **********/

  /* allocating maze solving resources */
	my_maze = new_Maze();    /* Initialize new maze */
  my_stack = new_Stack();  /* Initialize new stack */
  
  /* Initialize variables */
  x = START_X;
  y = START_Y;
  direction = NORTH;
  found_dest = FALSE;
  exit_solver_loop = FALSE;
  trip_count = 0;
  


  /* print the wall values..
     status of each cell's walls is represented by integer between 0 and 15
     if North Wall on : +1, if East Wall on  : +2
     if South Wall on : +4, if West Wall on  : +8
  */
  printf("\n");
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < SIZE; j++) {
      printf("  %2d  ", Walls[i][j]);
    }
    printf("\n\n");
  }
  printf("\n\n");

  /* print the initial state of the maze flood values */
	print_map(my_maze);
  



  /* Solver Loop */
  while (!exit_solver_loop) {

    found_dest = FALSE;
    direction = NORTH;

    /*** TRIP FROM START TO GOAL ***/
    printf("Begin Forward Trip #%d ... press enter to continue\n", ++trip_count);
    while(getchar() == NEWLINE);
    while(getchar() == NEWLINE);


    while (!found_dest) {
      printf("CURRENT LOCATION: %d, %d\n", x, y);  
      visit_Node(my_maze, my_stack, x, y, Walls[x][y],  FALSE);
      move_dir(my_maze, &x, &y, &direction);
      printf("NEW LOCATION: %d,%d\n", x, y);
      print_map(my_maze);
      check_goal_reached(&x, &y, &found_dest);
      /* negative coord check ... for errors */
      if (x < 0 || y < 0) {
        printf("NEGATIVE COORD: ERROR\n");
        return FALSE;
      }
      //press_enter_to_continue();
    }
    goal_x = x;
    goal_y = y;
    printf("press enter to continue... check the destination walls\n");
    while(getchar() == NEWLINE);


    /*** Reading the walls of the CENTER GOAL CELLS ***/  
    for (int i = 0; i < 4; i++) {
      printf("CURRENT LOCATION: %d, %d\n", x, y);  
      visit_Node(my_maze, my_stack, x, y, Walls[x][y], FALSE);  
      if ( x == SIZE / 2 - 1 && y == SIZE / 2 - 1 )
        x++;
      else if ( x == SIZE / 2 && y == SIZE / 2 - 1 ) 
        y++;
      else if ( x == SIZE / 2 && y == SIZE / 2 ) 
        x--;
      else 
        y--;
      printf("NEW LOCATION: %d,%d\n", x, y);
      print_map(my_maze);
      //press_enter_to_continue();
    }

    printf("destination walls have been checked...\n");
    printf("press enter to continue... begin reflooding from center\n");
    while(getchar() == NEWLINE);


    /*** Reflooding process from start to goal */
    /* Set everything to 255 ! */
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

    printf("cells have been reflooded from center...\n");
    print_map(my_maze);
    press_enter_to_continue();

    /* reset destination found flag */
    found_dest = FALSE;


    /*** TRIP FROM GOAL TO START ***/
    printf("Begin Return Trip #%d ... press enter to continue\n", trip_count);
    while(getchar() == NEWLINE);

    while (!found_dest) {
      printf("CURRENT LOCATION: %d, %d\n", x, y);  
      visit_Node(my_maze, my_stack, x, y, Walls[x][y],  TRUE);
      move_dir(my_maze, &x, &y, &direction);
      printf("NEW LOCATION: %d,%d\n", x, y);
      print_map(my_maze);
      check_start_reached(&x, &y, &found_dest);
      /* negative coord check ... for errors */
      if (x < 0 || y < 0) {
        printf("NEGATIVE COORD: ERROR\n");
        return FALSE;
      }
      //press_enter_to_continue();
    }

    printf("press enter to continue... begin reflooding from start\n");
    while(getchar() == NEWLINE);

    /*** Reflooding process from start to goal */
    /* Set everything to 0 ! */
    for (int i = 0; i < SIZE; i++) 
      for (int j = 0; j < SIZE; j++)
         my_maze->map[i][j]->floodval = 0;
    
    /* with start as zero, update everycell's floodval */
    push_open_neighbors(my_maze->map[goal_x][goal_y], my_stack);
    while(!is_empty_Stack(my_stack)) {
      pop(my_stack, &temp);
      flood_fill(temp, my_stack, FALSE);
    }

    printf("cells have been reflooded from start...\n");
    print_map(my_maze);
    press_enter_to_continue();



    /* prompt for another round of exploring the maze */
    printf("Round trip %d is complete... would you like to run another trip?\n", trip_count);
    do {
      printf(" [y]es - run another trip, [n]o: - exit program  : "); 
      ch = getchar();
    } while (ch != YES && ch != NO);

    if (ch == NO)
      exit_solver_loop = TRUE;

  }

  printf("Exiting Solver...\n");


  /* Deallocate Maze and Stack */
  delete_Maze(&my_maze);
  delete_Stack (&my_stack);

	return TRUE;
}


