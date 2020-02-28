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
import numpy as np

# class for Dijkstra
class Dijkstra(object):
    # init function
    def __init__(self, start, goal, clearance, radius):
        self.start = start
        self.goal = goal
        self.numRows = 200
        self.numCols = 300
        self.clearance = clearance
        self.radius = radius

    # move is valid 
    def IsValid(self, currRow, currCol):
        return (currRow >= 1 and currRow <= self.numRows and currCol >= 1 and currCol <= self.numCols)

    # checks for an obstacle
    def IsObstacle(self, row, col):
        return False

    # action move left
    def ActionMoveLeft(self, currRow, currCol):
        if(self.IsValid(currRow, currCol - 1) and self.IsObstacle(currRow, currCol - 1) == False):
            return True
        return False

    # action move right
    def ActionMoveRight(self, currRow, currCol):
        if(self.IsValid(currRow, currCol + 1) and self.IsObstacle(currRow, currCol + 1) == False):
            return True
        return False

    # action move up
    def ActionMoveUp(self, currRow, currCol):
        if(self.IsValid(currRow - 1, currCol) and self.IsObstacle(currRow - 1, currCol) == False):
            return True
        return False

    # action move down
    def ActionMoveDown(self, currRow, currCol):
        if(self.IsValid(currRow + 1, currCol) and self.IsObstacle(currRow + 1, currCol) == False):
            return True
        return False

    # action move right up
    def ActionMoveRightUp(self, currRow, currCol):
        if(self.IsValid(currRow - 1, currCol + 1) and self.IsObstacle(currRow - 1, currCol + 1) == False):
            return True
        return False

    # action move right down
    def ActionMoveRightDown(self, currRow, currCol):
        if(self.IsValid(currRow + 1, currCol + 1) and self.IsObstacle(currRow + 1, currCol + 1) == False):
            return True
        return False

    # action move left down
    def ActionMoveLeftDown(self, currRow, currCol):
        if(self.IsValid(currRow + 1, currCol - 1) and self.IsObstacle(currRow + 1, currCol - 1) == False):
            return True
        return False

    # action move left up
    def ActionMoveLeftUp(self, currRow, currCol):
        if(self.IsValid(currRow - 1, currCol - 1) and self.IsObstacle(currRow - 1, currCol - 1) == False):
            return True
        return False
    
    # dijkstra algorithm
    def Dijkstra(self):
        # create hashmap to store distances
        distMap = {}
        path = {}
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                distMap[(row, col)] = float('inf')
                path[(row, col)] = -1
            
        # create queue, push the source and mark distance from source to source as zero
        (startRow, startCol) = self.start
        explored_states = []
        queue = []
        queue.append(self.start)
        distMap[self.start] = 0
    
        # run dijkstra algorithm and find shortest path
        while(len(queue) > 0):
            currNode = queue[0]
            explored_states.append(currNode)
            queue.pop(0)
        
            # go through each edge of current node
            if(self.ActionMoveLeft(currNode[0], currNode[1]) and (distMap[(currNode[0], currNode[1] - 1)] > distMap[currNode] + 1)):
                distMap[(currNode[0], currNode[1] - 1)] = distMap[currNode] + 1
                path[(currNode[0], currNode[1] - 1)] = currNode
                queue.append((currNode[0], currNode[1] - 1))
            
            if(self.ActionMoveRight(currNode[0], currNode[1]) and (distMap[(currNode[0], currNode[1] + 1)] > distMap[currNode] + 1)):
                distMap[(currNode[0], currNode[1] + 1)] = distMap[currNode] + 1
                path[(currNode[0], currNode[1] + 1)] = currNode
                queue.append((currNode[0], currNode[1] + 1))
            
            if(self.ActionMoveUp(currNode[0], currNode[1]) and (distMap[(currNode[0] - 1, currNode[1])] > distMap[currNode] + 1)):
                distMap[(currNode[0] - 1, currNode[1])] = distMap[currNode] + 1
                path[(currNode[0] - 1, currNode[1])] = currNode
                queue.append((currNode[0] - 1, currNode[1]))
            
            if(self.ActionMoveDown(currNode[0], currNode[1]) and (distMap[(currNode[0] + 1, currNode[1])] > distMap[currNode] + 1)):
                distMap[(currNode[0] + 1, currNode[1])] = distMap[currNode] + 1
                path[(currNode[0] + 1, currNode[1])] = currNode
                queue.append((currNode[0] + 1, currNode[1]))
            
            if(self.ActionMoveLeftDown(currNode[0] + 1, currNode[1] - 1) and (distMap[(currNode[0] + 1, currNode[1] - 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] + 1, currNode[1] - 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] + 1, currNode[1] - 1)] = currNode
                queue.append((currNode[0] + 1, currNode[1] - 1))
            
            if(self.ActionMoveRightDown(currNode[0], currNode[1]) and (distMap[(currNode[0] + 1, currNode[1] + 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] + 1, currNode[1] + 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] + 1, currNode[1] + 1)] = currNode
                queue.append((currNode[0] + 1, currNode[1] + 1))
            
            if(self.ActionMoveRightUp(currNode[0], currNode[1]) and (distMap[(currNode[0] - 1, currNode[1] + 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] - 1, currNode[1] + 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] - 1, currNode[1] + 1)] = currNode
                queue.append((currNode[0] - 1, currNode[1] + 1))
            
            if(self.ActionMoveLeftUp(currNode[0], currNode[1]) and (distMap[(currNode[0] - 1, currNode[1] - 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] - 1, currNode[1] - 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] - 1, currNode[1] - 1)] = currNode
                queue.append((currNode[0] - 1, currNode[1] - 1))
        
        # backtrack path
        backtrack_states = []
        node = self.goal
        while(path[node] != -1):
            backtrack_states.append(node)
            node = path[node]
        backtrack_states.append(self.start)
        backtrack_states = list(reversed(backtrack_states))      
        return (explored_states, backtrack_states, distMap[self.goal])
