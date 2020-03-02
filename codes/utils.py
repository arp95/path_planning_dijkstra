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

    # calculate triangle area
    def area(self, x1, y1, x2, y2, x3, y3):
        return np.abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0)  
        
    # move is valid 
    def IsValid(self, currRow, currCol):
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))

    # checks for an obstacle
    def IsObstacle(self, row, col):
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r
        
        # check circle
        dist1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225)) - ((25 + sum_of_c_and_r)*(25 + sum_of_c_and_r))
        
        # check eclipse
        dist2 = ((((row - 100) * (row - 100)) / ((20 + sum_of_c_and_r) * (20 + sum_of_c_and_r))) + (((col - 150) * (col - 150))/((40 + sum_of_c_and_r) * (40 + sum_of_c_and_r)))) - 1
        
        # check triangles
        area1 = self.area(120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r), 150 - sqrt_of_c_and_r, 50, 185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        area2 = self.area(row, col, 150 - sqrt_of_c_and_r, 50, 185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        area3 = self.area(120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r), row, col, 185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        area4 = self.area(120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r), 150 - sqrt_of_c_and_r, 50, row, col)
        dist3 = (area2 + area3 + area4) - area1
        if(dist3 < 1e-5):
            dist3 = 0
        area1 = self.area(150 - sqrt_of_c_and_r, 50, 185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247), 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area2 = self.area(row, col, 185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247), 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area3 = self.area(150 - sqrt_of_c_and_r, 50, row, col, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area4 = self.area(150 - sqrt_of_c_and_r, 50, 185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247), row, col)
        dist4 = (area2 + area3 + area4) - area1
        if(dist4 < 1e-5):
            dist4 = 0
        
        # check rhombus
        area1 = self.area(10 - sqrt_of_c_and_r, 225, 25, 200 - sqrt_of_c_and_r, 40 + sqrt_of_c_and_r, 225)
        area2 = self.area(row, col, 25, 200 - sqrt_of_c_and_r, 40 + sqrt_of_c_and_r, 225)
        area3 = self.area(10 - sqrt_of_c_and_r, 225, row, col, 40 + sqrt_of_c_and_r, 225)
        area4 = self.area(10 - sqrt_of_c_and_r, 225, 25, 200 - sqrt_of_c_and_r, row, col)
        dist5 = (area2 + area3 + area4) - area1
        if(dist5 < 1e-5):
            dist5 = 0
        area1 = self.area(10 - sqrt_of_c_and_r, 225, 25, 250 + sqrt_of_c_and_r, 40 + sqrt_of_c_and_r, 225)
        area2 = self.area(row, col, 25, 250 + sqrt_of_c_and_r, 40 + sqrt_of_c_and_r, 225)
        area3 = self.area(10 - sqrt_of_c_and_r, 225, row, col, 40 + sqrt_of_c_and_r, 225)
        area4 = self.area(10 - sqrt_of_c_and_r, 225, 25, 250 + sqrt_of_c_and_r, row, col)
        dist6 = (area2 + area3 + area4) - area1
        if(dist6 < 1e-5):
            dist6 = 0
        
        # check square
        area1 = self.area(120 - sqrt_of_c_and_r, 75, 150 - sqrt_of_c_and_r, 50, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area2 = self.area(row, col, 150 - sqrt_of_c_and_r, 50, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area3 = self.area(120 - sqrt_of_c_and_r, 75, row, col, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area4 = self.area(120 - sqrt_of_c_and_r, 75, 150 - sqrt_of_c_and_r, 50, row, col)
        dist7 = (area2 + area3 + area4) - area1
        if(dist7 < 1e-5):
            dist7 = 0
        area1 = self.area(120 - sqrt_of_c_and_r, 75, 150, 100 + sqrt_of_c_and_r, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area2 = self.area(row, col, 150, 100 + sqrt_of_c_and_r, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area3 = self.area(120 - sqrt_of_c_and_r, 75, row, col, 185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        area4 = self.area(120 - sqrt_of_c_and_r, 75, 150, 100 + sqrt_of_c_and_r, row, col)
        dist8 = (area2 + area3 + area4) - area1
        if(dist8 < 1e-5):
            dist8 = 0
        
        # check rod
        area1 = self.area(30 - sqrt_of_c_and_r, 95, 67.5, 30.05 - sqrt_of_c_and_r, 76.15 + sqrt_of_c_and_r, 35.5)
        area2 = self.area(row, col, 67.5, 30.05 - sqrt_of_c_and_r, 76.15 + sqrt_of_c_and_r, 35.5)
        area3 = self.area(30 - sqrt_of_c_and_r, 95, row, col, 76.15 + sqrt_of_c_and_r, 35.5)
        area4 = self.area(30 - sqrt_of_c_and_r, 95, 67.5, 30.05 - sqrt_of_c_and_r, row, col)
        dist9 = (area2 + area3 + area4) - area1
        if(dist9 < 1e-5):
            dist9 = 0
        area1 = self.area(30 - sqrt_of_c_and_r, 95, 76.15 + sqrt_of_c_and_r, 35.5, 38.66, 100 + sqrt_of_c_and_r)
        area2 = self.area(row, col, 76.15 + sqrt_of_c_and_r, 35.5, 38.66, 100 + sqrt_of_c_and_r)
        area3 = self.area(30 - sqrt_of_c_and_r, 95, row, col, 38.66, 100 + sqrt_of_c_and_r)
        area4 = self.area(30 - sqrt_of_c_and_r, 95, 76.15 + sqrt_of_c_and_r, 35.5, row, col)
        dist10 = (area2 + area3 + area4) - area1
        if(dist10 < 1e-5):
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
            image[self.numRows - state[0], state[1] - 1] = (255, 255, 0)
            if(count%80 == 0):
                out.write(image)
            count = count + 1

        count = 0
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                if(image[self.numRows - row, col - 1, 0] == 0 and image[self.numRows - row, col - 1, 1] == 0 and image[self.numRows - row, col - 1, 2] == 0):
                    if(self.IsValid(row, col) and self.IsObstacle(row, col) == False):
                        image[self.numRows - row, col - 1] = (154, 250, 0)
                        if(count%80 == 0):
                            out.write(image)
                        count = count + 1
            
        if(len(backtrack_states) > 0):
            for state in backtrack_states:
                image[self.numRows - state[0], state[1] - 1] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)
                
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()
