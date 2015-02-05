def floodFill(self, data, colsize, rowsize, colstart, rowstart):

	#move forward one cell

	#check if mouse is finished; floodval==0

	
	''' Check adjacent cells '''
	#floodfill adjacent squares and see if floodvals are 
	#consistent -> move forward if possible to lowest floodval direction
		#if adjacent floodvals are inconsistent, take previous + 1
		#take that previous and check if flood vals consistent

	#### Consistency: I think it means should have adjacent cell 
	# have floodval - 1... if not, means need to update

	''' Determine if wall '''
	#call micromouse sensors to determine if pointers to 
	#adjacent cells are walls -> set node pointer = -1
		#recursion -> adjacent cells

''' Summary pseudo code:
1. Create 16x16 array to hold maze elements
	- each element is a node that holds:
	i. pointers to adjacent cells
	ii. floodval
	iii. location (row, column)
	iv. if visited or not?
2. Create stack that can hold 256 entries
3. Make finished cells, '0' value
4. Make rest of cells optimistic values (spiral outwards distance)

5. Push current cell onto stack (aka starting cell)
6. While (!isempty_stack(stack)) {
	pull cell from stack

	if current floodval != min(neighbor floodval) + 1 {
		change current floodval to one grater than min
		of neighbor's and push all cell's accessible neighbors
		onto stack
	}
	else {
		pull cells and keep checking
	}
}

7. Checking dead ends ->
	make all node pointers = -1

