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
args = sys.argv
if(len(args) > 4):
    startRow = float(args[1])
    startCol = float(args[2])
    goalRow = float(args[3])
    goalCol = float(args[4])

    # define constants
    start = (startRow, startCol)
    goal = (goalRow, goalCol)
    clearance = 0
    radius = 0
    dijkstra = Dijkstra(start, goal, clearance, radius)

    # find path
    if(dijkstra.IsValid(start[0], start[1]) and dijkstra.IsValid(goal[0], goal[1]) and dijkstra.IsObstacle(start[0], start[1]) == False and dijkstra.IsObstacle(goal[0], goal[1]) == False):
        (explored_states, backtrack_states, distance_from_start_to_goal) = dijkstra.Dijkstra()
        dijkstra.animate(explored_states, backtrack_states, "./dijkstra_point.avi")
    
        # print optimal path found or not
        if(distance_from_start_to_goal == float('inf')):
            print("No optimal path found.")
        else:
            print("Optimal path found.")
    else:
        print("Invalid state entered. Please run the file again.")
else:
    print("Please check README.md file for running Dijkstra_point.py file.")
