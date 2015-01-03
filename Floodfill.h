/*
 * Floodfill.h
 *
 *  Created on: January 2nd, 2015
 *      Author: Adam Li / adam2392

Q:
1. why is largeval 301?
 */


#ifndef Floodfill_H_
#define Floodfill_H_

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 1
#endif

#define SIZE 16	//the size of one dimension of Map
#define LARGEVAL 301


//define directions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//define stack structure constants
#define SPI 1 	//stack pointer index
#define SSI 0	//stack size index (points to size)
#define STACK_OFFSET 2
#define STACKSIZE 80	//defines the size of the stack

/* 
Defines the node for the floodfill algorithm to fill up
*/
typedef struct Node {
	/* data fields */
	short floodval;
	short row;
	short column;
	short visited;

	/* pointers to neighbors */
	struct Node *left;
	struct Node *right;
	struct Node *up;
	struct Node *down;
} Node;

/*
Defines the Maze struct that holds a 2D array of Node pointers
*/
typedef struct Maze {
	Node *map[SIZE][SIZE];
} Maze;

typedef struct Stack {
	short properties[STACK_OFFSET]; //creates a short array of size stack_offset
	Node *the_stack[STACKSIZE];	//creates a Node pointer array of size stacksize
} Stack;

//Node functions
Node *new Node(const short x, const short y);
void delete_Node(Node **npp);
void flood_fill(Node *this_node, Stack *this_stack, const short reflood_flag);
void set_value(Node *this_node, const short value);
short get_smallest_neighbor(Node *this_node);

//Maze functions
Maze *new_Maze();
void delete_Maze(Maze **mpp);
void print_maze(const Maze *this_maze);

//Stack functions
Stack *new_Stack();
void delete_Stack(Stack **spp);
int is_empty_Stack(Stack *this_stack);
void pop(Stack *this_stack, Node **npp);
void push(Stack *this_stack, Node *this_node);

//Debug function setters
void debug_on();
void debug_off();

#endif /* Floodfill_H_ */
