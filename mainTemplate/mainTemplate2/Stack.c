#include <stdlib.h>
#include <stdio.h>
#include "Maze.h"



// Stack Constructor
Stack * new_Stack() {

  Stack * this_stack = (Stack *) malloc(sizeof(Stack));

  this_stack->properties[SPI] = 0;
  this_stack->properties[SSI] = STACKSIZE;

  return this_stack;
}

// Stack Destructor
void delete_Stack (Stack ** spp) {

  if (spp == 0 || *spp == 0) {
    fprintf(stderr, "NULL POINTER\n");
    return;
  }

  free(*spp);

  *spp = 0;

}


// Checks if this_stack is empty
int is_empty_Stack (Stack * this_stack) {

  //printf("%d\n", this_stack->properties[SPI]);

  if (this_stack->properties[SPI] == 0)
    return 1;
  else return 0;
}

// Pops the top element of this_stack
void pop (Stack * this_stack, Node ** npp) {


  short index;

  index = this_stack->properties[SPI] - 1;

  *npp = this_stack->the_stack[index];

  this_stack->properties[SPI] -= 1;

}

// Pushes an element to the top of this_stack
void push (Stack * this_stack, Node * this_node) {

  short index;

  index = this_stack->properties[SPI];

  this_stack->the_stack[index] = this_node;

  this_stack->properties[SPI] += 1;
}


