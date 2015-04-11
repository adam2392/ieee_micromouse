/*
 * Floodfill.cpp
 *
 *  Created on: January 2nd, 2015
 *      Author: Adam Li / adam2392

Q:
1. get_smallestneighbor: if vs. else if

 */

#include <stdlib.h>
#include <stdio.h>
#include “Floodfill.h”

static int debug_on = FALSE;	//debug flag

//Debug statement functions
void debug_on() {
	debug_on = TRUE;
}
void debug_off() {
	debug_on = FALSE;
}

/*** Constructors and Destructors for: Node, Maze, Stack ***/

//Node constructor
Node *new_Node(const short x, const short y) {
	Node *this_node;	//initialize a Node *
	short halfsize;		//initialize the middle point of the size of maze

	//debug statement to figure out what Node position we are creating
	if(debug_on)
		printf("allocating %hd, %hd\n", x, y);	
	
	this_node = (Node *) malloc(sizeof(Node)); //allocate memory for the node pointer
	halfsize = SIZE/2;

	//initialize the node's x, y coordinates and the 'VISITED' flag to false
	this_node->row = y;
	this_node->column = x;
	VISITED = FALSE;

	//initializing flood value at coord
	//only works when SIZE is even -- okay, but can we do better?
	if (x < halfsize && y < halfsize)
		this_node->floodval = (halfsize - 1 - x) + (halfsize - 1 - y);
	else if (x < halfsize && y >= halfsize)
		this_node->floodval = (halfsize - 1 - x) + (y - halfsize);
	else if (x >= halfsize && y < halfsize)
		this_node->floodval = (x - halfsize) + (halfsize - 1 - y);
	else
		this_node->floodval = (x - halfsize) + (y - halfsize);

	return this_node;	//return the initialized node
}

//Maze constructor
Maze *new_Maze() {
	Maze *this_maze;		//intialize maze *
	short x, y;				//initialize x,y coords

	this_maze = (Maze *)malloc(sizeof(Maze));	//allocate memory for maze*

	//allocate new Node for each coord of the maze
	for(x = 0; x < SIZE; ++x)
		for(y = 0; y < SIZE; ++y)
			this_maze->map[x][y] = new_Node(x, y);

	//set neighborhood pointers: up, down, right, left
	for(x = 0; x < SIZE; ++x){
		for(y = 0; y < SIZE; ++y) {
			this_maze->map[x][y]->down = (y == 0) ? NULL : (this_maze->map[x][y-1]);
			this_maze->map[x][y]->up = (y == SIZE-1) ? NULL : (this_maze->map[x][y+1]);
			this_maze->map[x][y]->left = (x==0) ? NULL : (this_maze->map[x-1][y]);
			this_maze->map[x][y]->right = (x == SIZE-1) ? NULL : (this_maze->map[x+1][y]);
		}
	}

	return this_maze;
}

//Node destructor
void delete_Node (Node **npp) {
	//debug statements?
	if(debug_on)
		printf("deallocating %d, %d\n", (*npp)->row, (*npp)->column);

	free(*npp);		//free memory
	*npp = 0;		//set node pointers to null
}

//Maze destructor
void delete_Maze(Maze *mpp) {
	//debug statements?
	if(debug_on)
		printf("deallocating maze \n");

	short x, y;
	for (x = 0; x < SIZE; ++x)
		for(y = 0; y < SIZE; ++y)
			delete_Node(&((*mpp)->map[x][y]));	//pass by reference the nodes in the maze 1by1

	free(*mpp);	//free up the memory from malloc
	*mpp = 0;	//set maze ptr to null
}

/*** Maze Functions ***/
/* 
Function: print_maze(const Maze *this_maze)
Description: Prints out the maze and the flood values of each cell
in the maze

Input: Maze *this_maze - a const maze pointer that you use to access
all the flood values of the nodes.
Output: none
*/
void print_maze(const Maze *this_maze) {
	short x, y;

	printf("\n%s\n\n", "Current Map Values: ");

	//prints out the flood values of each cell in the maze
	for (x = 0; x < SIZE; ++x) {
		for (y = 0; y < SIZE; ++y) {
			printf("%s%3hd", " ", this_maze->map[x][y]->floodval);
		}
		printf("\n\n");
	}

	printf("\n");
}

/*** Node Functions ***/

/* function for obtaining this_node's smallest neighbor's floodval */
short get_smallest_neighbor (Node * this_node) {

	/* debug statements */
	if (debug_on)
		printf("In get_smallest_neighbor\n");

	//the Node's floodval will be 1 higher than neighboring cell
	short smallestneighbor = LARGEVAL

	//check if left is available and left's right pointer is available 
	if(this_node->left != NULL && (this_node->left->right != NULL) 
		&& (this_node->left->floodval) < smallestneighbor)
		smallestneighbor = this_node->left->floodval;
	//check right
	else if(this_node->right != NULL && (this_node->right->left != NULL) 
		&& (this_node->right->floodval) < smallestneighbor)
		smallestneighbor = this_node->right->floodval;
	//check down
	else if(this_node->down != NULL && (this_node->down->up != NULL) 
		&& (this_node->down->floodval) < smallestneighbor)
		smallestneighbor = this_node->down->floodval;
	//check up
	else if(this_node->up != NULL && (this_node->up->down != NULL) 
		&& (this_node->up->floodval) < smallestneighbor)
		smallestneighbor = this_node->up->floodval;


	return smallestneighbor;
}

/* Function for setting this node's floodval to a specific value */
void set_value (Node * this_node, const short value) {

	/* debug statements */
	if (debug_on) {
		printf("In set_value\n");
		printf("Floodval set to : %d\n", this_node->floodval);
	}
	
	/* set the flood value to specified value */
	this_node->floodval = value;
}

void set_visited(Node *this_node) {
	if(debug_on)
		printf("Node at %d %d set_visited\n", this_node->row, this_node->column);

	//set visited flag flood value to specified value
	VISITED = TRUE;
}







