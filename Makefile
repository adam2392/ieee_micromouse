all: Solver

Solver: Solver.o Maze.o Stack.o
	gcc -o Solver -g Solver.o Maze.o Stack.o

Solver.o: Solver.c 
	gcc -Wall -pedantic -c -g Solver.c Maze.h 

Maze.o: Maze.c Maze.h
	gcc -Wall -pedantic -c -g Maze.c Maze.h

Stack.o: Stack.c Maze.h
	gcc -Wall -pedantic -c -g Stack.c Maze.h

clean: 
	rm -f *.o *.gch Solver

new: 
	make clean
	make
<<<<<<< HEAD

=======
>>>>>>> 65a7c1f6b6fea9d8d59ffef920ee91d5fed03ef8
