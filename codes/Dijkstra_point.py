"""
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal Markose Jacob
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
"""

# header files
from utils import *
import sys

# take start and goal node as input
b = int(input("Enter the row coordinate for initial node (between 1 and 300) : "))
a = int(input("Enter the column coordinate for initial node (between 1 and 200) : "))
d = int(input("Enter the row coordinate for goal node (between 1 and 300) : "))
c = int(input("Enter the column coordinate for goal node (between 1 and 200) : "))
# print("Enter the row coorditate for initial node : ")
# input(a)



# args = sys.argv
# if(len(args) > 4):
# if(a<=300)&(a>=1)&(b<=200)&(b>=1)&(c<=300)&(c>=1)&(d<=200)&(d>=1):
startRow = a
startCol = b
goalRow = c
goalCol = d

# define constants
start = (startRow, startCol)
goal = (goalRow, goalCol)
clearance = 0
radius = 0
dijkstra = Dijkstra(start, goal, clearance, radius)

if(dijkstra.IsValid(start[0], start[1])):
	if(dijkstra.IsValid(goal[0], goal[1])):
		if(dijkstra.IsObstacle(start[0],start[1]) == False):
			if(dijkstra.IsObstacle(goal[0], goal[1]) == False):
				(explored_states, backtrack_states, distance_from_start_to_goal) = dijkstra.Dijkstra()
				dijkstra.animate(explored_states, backtrack_states, "./dijkstra_point.avi")
				# print optimal path found or not
				if(distance_from_start_to_goal == float('inf')):
					print("No optimal path found.")
				else:
					print("Optimal path found.")
			else:
				print("The entered goal node is an obstacle ")
				print("Please check README.md file for running Dijkstra_point.py file.")
		else:
			print("The entered initial node is an obstacle ")
			print("Please check README.md file for running Dijkstra_point.py file.")
	else:
		print("The entered goal node outside the map ")
		print("Please check README.md file for running Dijkstra_point.py file.")
else:
	print("The entered initial node is outside the map ")
	print("Please check README.md file for running Dijkstra_point.py file.")


#     # find path
#     if(dijkstra.IsValid(start[0], start[1]) and dijkstra.IsValid(goal[0], goal[1]) and dijkstra.IsObstacle(start[0], start[1]) == False and dijkstra.IsObstacle(goal[0], goal[1]) == False):
#         (explored_states, backtrack_states, distance_from_start_to_goal) = dijkstra.Dijkstra()
#         dijkstra.animate(explored_states, backtrack_states, "./dijkstra_point.avi")
	
#         # print optimal path found or not
#         if(distance_from_start_to_goal == float('inf')):
#             print("No optimal path found.")
#         else:
#             print("Optimal path found.")
#     else:
#         print("Invalid state entered. Please run the file again.")
# else:
#     print("The coordinates entered are wrong.")
#     print("Please check README.md file for running Dijkstra_point.py file.")
