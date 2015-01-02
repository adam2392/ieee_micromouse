/*
 * Stack.cpp
 *
 *  Created on: January 2nd, 2015
 *      Author: Adam Li / adam2392

Q:

 */

#include <stdlib.h>
#include <stdio.h>
#include “Floodfill.h”

//Stack constructor
Stack *new_Stack() {
	Stack *this_stack;

	this_stack = (Stack *)malloc(sizeof(Stack));

	this_stack->properties[SPI] = 0;
	this_stack->properties[SSI] = STACKSIZE;

	return this_stack;
}

//stack destructor
void delete_Stack(Stack **spp) {
	
}