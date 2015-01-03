/*
 * Stack.cpp
 * Implements the stack functions 
 *
 *
 *  Created on: January 2nd, 2015
 *      Author: Adam Li / adam2392

Q:

 */

#include <stdlib.h>
#include <stdio.h>
#include “Floodfill.h”

/* 
Function: newStack()
Description: Stack constructor. Allocates memory and also initializes
stack pointer index and stack size index.

Input: None
Output: Stack * - Stack pointer that points to the new stack.
*/
Stack * new_Stack() {
	if(debug_on)
		printf("Allocating Stack of size %d\n", STACKSIZE);

	Stack *this_stack;	//initialize stack ptr

	this_stack = (Stack *)malloc(sizeof(Stack));	//allocate memory

	this_stack->properties[SPI] = 0;				//initialize pointer index
	this_stack->properties[SSI] = STACKSIZE;		//initailize size index

	return this_stack;
}

/* 
Function: delete_Stack(Stack **spp)
Description: Stack destructor. Deallocates memory and also sets the 
double pointer stack to null.

Input: Stack **spp - is a pointer to a stack pointer that points to the stack
we want to delete 
Output: none
*/
void delete_Stack(Stack **spp) {
	if(debug_on)
		printf("deallocating Stack\n");

	//null error checks
	if(spp == 0 || *spp == 0) {
		fprintf(stderr, "NULL POINTER\n");
		return;
	}

	free(*spp);
	*spp = 0;
}

/* 
Function: is_empty_Stack(Stack *this_stack)
Description: check if this_stack is empty

Input: Stack * - a stack pointer to this_stack
Output: int returns 1 if empty, if not return 0
*/
int is_empty_Stack(Stack *this_stack) {
	if(debug_on)
		printf("The stack pointer index %d\n", this_stack->properties[SPI]);

	//check where the stack pointer index is pointing to the '0' location
	if(this_stack->properties[SPI] == 0)
		return 1;
	else
		return 0;
}

/* 
Function: pop(Stack *this_stack, Node **npp)
Description: Takes off a node from the stack.

Input: Stack *this_stack - a stack pointer to the stack that holds the nodes
Node **npp - is a pointer to a node pointer that was popped off
Output: none
*/
void pop(Stack *this_stack, Node **npp) {
	if(debug_on)
		printf("popping the stack at %d\n", this_stack->properties[SPI]);

	short index; 								//initialize a temporary holder index

	index = this_stack->properties[SPI] - 1;	//get the new index after pop

	*npp = this_stack->the_stack[index];		//the Node * gets the Node* in the stack struct

	this_stack->properties[SPI] -= 1;			//decrement the SPI
}

/* 
Function: push(Stack *this_stack, Node *this_node)
Description: Puts a new node pointer onto the stack. 

Input: Stack *this_stack - a stack pointer
Node *this_node - is a node pointer that points to the node
we want to put onto the stack
Output: none
*/
void push(Stack *this_stack, Node *this_node) {
	if(debug_on)
		printf("pushing onto the stack at %d\n", this_stack->properties[SPI] + 1);

	short index;								//initialize a temp. holder index

	index = this_stack->properties[SPI] + 1;	//get the current index

	this_stack->the_stack[index] = this_node;	//put new node onto stack

	this_stack->properties[SPI] += 1;			//increment the SPI
}
