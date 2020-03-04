#.0 header files
import numpy as np
from heapq import heappush, heappop
import cv2

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
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))

    # checks for an obstacle
    def IsObstacle(self, row, col):
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r
        
        # check circle
        dist1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225)) - ((25 + sum_of_c_and_r) * (25 + sum_of_c_and_r))
        
        # check eclipse
        dist2 = ((((row - 100) * (row - 100)) / ((20 + sum_of_c_and_r) * (20 + sum_of_c_and_r))) + (((col - 150) * (col - 150)) / ((40 + sum_of_c_and_r) * (40 + sum_of_c_and_r)))) - 1
        
        # check triangles
        (x1, y1) = (120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r))
        (x2, y2) = (150 - sqrt_of_c_and_r, 50)
        (x3, y3) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist3 = 1
        if(first <= 0 and second <= 0 and third <= 0):
            dist3 = 0
            
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        (x3, y3) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist4 = 1
        if(first >= 0 and second >= 0 and third >= 0):
            dist4 = 0
        
        # check rhombus
        (x1, y1) = (10 - sqrt_of_c_and_r, 225)
        (x2, y2) = (25, 200 - sqrt_of_c_and_r)
        (x3, y3) = (40 + sqrt_of_c_and_r, 225)
        (x4, y4) = (25, 250 + sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist5 = 1
        dist6 = 1
        if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist5 = 0
            dist6 = 0
        
        # check square
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (120 - sqrt_of_c_and_r, 75)
        (x3, y3) = (150, 100 + sqrt_of_c_and_r)
        (x4, y4) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist7 = 1
        dist8 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist7 = 0
            dist8 = 0
        
        # check rod
        first = ((col - 95) * (8.66 + sqrt_of_c_and_r)) - ((5 + sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        second = ((col - 95) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        third = ((col - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (row - 67.5))
        fourth = ((col - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (row - 76.15 - sqrt_of_c_and_r))
        dist9 = 1
        dist10 = 1
        if(first <= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist9 = 0
            dist10 = 0
        
        if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
            return True
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
        visited = {}
        path = {}
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                distMap[(row, col)] = float('inf')
                path[(row, col)] = -1
                visited[(row, col)] = False
            
        # create queue, push the source and mark distance from source to source as zero
        explored_states = []
        queue = []
        heappush(queue, (0, self.start))
        distMap[self.start] = 0
    
        # run dijkstra algorithm and find shortest path
        while(len(queue) > 0):
            _, currNode = heappop(queue)
            visited[currNode] = True
            explored_states.append(currNode)
        
            # if goal node then exit
            if(currNode[0] == self.goal[0] and currNode[1] == self.goal[1]):
                break
        
            # go through each edge of current node
            if(self.ActionMoveLeft(currNode[0], currNode[1]) and visited[(currNode[0], currNode[1] - 1)] == False and (distMap[(currNode[0], currNode[1] - 1)] > distMap[currNode] + 1)):
                distMap[(currNode[0], currNode[1] - 1)] = distMap[currNode] + 1
                path[(currNode[0], currNode[1] - 1)] = currNode
                heappush(queue, (distMap[(currNode[0], currNode[1] - 1)], (currNode[0], currNode[1] - 1)))
            
            if(self.ActionMoveRight(currNode[0], currNode[1]) and visited[(currNode[0], currNode[1] + 1)] == False and (distMap[(currNode[0], currNode[1] + 1)] > distMap[currNode] + 1)):
                distMap[(currNode[0], currNode[1] + 1)] = distMap[currNode] + 1
                path[(currNode[0], currNode[1] + 1)] = currNode
                heappush(queue, (distMap[(currNode[0], currNode[1] + 1)], (currNode[0], currNode[1] + 1)))
            
            if(self.ActionMoveUp(currNode[0], currNode[1]) and visited[(currNode[0] - 1, currNode[1])] == False and (distMap[(currNode[0] - 1, currNode[1])] > distMap[currNode] + 1)):
                distMap[(currNode[0] - 1, currNode[1])] = distMap[currNode] + 1
                path[(currNode[0] - 1, currNode[1])] = currNode
                heappush(queue, (distMap[(currNode[0] - 1, currNode[1])], (currNode[0] - 1, currNode[1])))
            
            if(self.ActionMoveDown(currNode[0], currNode[1]) and visited[(currNode[0] + 1, currNode[1])] == False and (distMap[(currNode[0] + 1, currNode[1])] > distMap[currNode] + 1)):
                distMap[(currNode[0] + 1, currNode[1])] = distMap[currNode] + 1
                path[(currNode[0] + 1, currNode[1])] = currNode
                heappush(queue, (distMap[(currNode[0] + 1, currNode[1])], (currNode[0] + 1, currNode[1])))
            
            if(self.ActionMoveLeftDown(currNode[0] + 1, currNode[1] - 1) and visited[(currNode[0] + 1, currNode[1] - 1)] == False and (distMap[(currNode[0] + 1, currNode[1] - 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] + 1, currNode[1] - 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] + 1, currNode[1] - 1)] = currNode
                heappush(queue, (distMap[(currNode[0] + 1, currNode[1] - 1)], (currNode[0] + 1, currNode[1] - 1)))
            
            if(self.ActionMoveRightDown(currNode[0], currNode[1]) and visited[(currNode[0] + 1, currNode[1] + 1)] == False and (distMap[(currNode[0] + 1, currNode[1] + 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] + 1, currNode[1] + 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] + 1, currNode[1] + 1)] = currNode
                heappush(queue, (distMap[(currNode[0] + 1, currNode[1] + 1)], (currNode[0] + 1, currNode[1] + 1)))
            
            if(self.ActionMoveRightUp(currNode[0], currNode[1]) and visited[(currNode[0] - 1, currNode[1] + 1)] == False and (distMap[(currNode[0] - 1, currNode[1] + 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] - 1, currNode[1] + 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] - 1, currNode[1] + 1)] = currNode
                heappush(queue, (distMap[(currNode[0] - 1, currNode[1] + 1)], (currNode[0] - 1, currNode[1] + 1)))
            
            if(self.ActionMoveLeftUp(currNode[0], currNode[1]) and visited[(currNode[0] - 1, currNode[1] - 1)] == False and (distMap[(currNode[0] - 1, currNode[1] - 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] - 1, currNode[1] - 1)] = distMap[currNode] + 1.4142
                path[(currNode[0] - 1, currNode[1] - 1)] = currNode
                heappush(queue, (distMap[(currNode[0] - 1, currNode[1] - 1)], (currNode[0] - 1, currNode[1] - 1)))
        
        # return if no optimal path
        if(distMap[self.goal] == float('inf')):
            return (explored_states, [], distMap[self.goal])
        
        # backtrack path
        backtrack_states = []
        node = self.goal
        while(path[node] != -1):
            backtrack_states.append(node)
            node = path[node]
        backtrack_states.append(self.start)
        backtrack_states = list(reversed(backtrack_states))  
        # print(backtrack_states)    
        return (explored_states, backtrack_states, distMap[self.goal])
    
    # animate path
    def animate(self, explored_states, backtrack_states, path):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))
        image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)
        count = 0
        for state in explored_states:
            image[int(self.numRows - state[0]), int(state[1] - 1)] = (255, 255, 0)
            if(count%80 == 0):
                out.write(image)
            count = count + 1

        count = 0
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                if(image[int(self.numRows - row), int(col - 1), 0] == 0 and image[int(self.numRows - row), int(col - 1), 1] == 0 and image[int(self.numRows - row), int(col - 1), 2] == 0):
                    if(self.IsValid(row, col) and self.IsObstacle(row, col) == False):
                        image[int(self.numRows - row), int(col - 1)] = (154, 250, 0)
                        if(count%80 == 0):
                            out.write(image)
                        count = count + 1
            
        if(len(backtrack_states) > 0):
            for state in backtrack_states:
                image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)
                
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()
