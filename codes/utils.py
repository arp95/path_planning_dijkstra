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
        return (currRow >= 1 and currRow <= self.numRows and currCol >= 1 and currCol <= self.numCols)

    # checks for an obstacle
    def IsObstacle(self, row, col):
        # check circle
        dist1 = ((row - 150)*(row - 150) + (col - 225)*(col - 225)) - 625
        
        # check eclipse
        dist2 = ((((row - 100)*(row - 100))/400) + (((col - 150)*(col - 150))/1600)) - 1
        
        # check triangles
        area1 = self.area(120, 20, 150, 50, 185, 25)
        area2 = self.area(row, col, 150, 50, 185, 25)
        area3 = self.area(120, 20, row, col, 185, 25)
        area4 = self.area(120, 20, 150, 50, row, col)
        dist3 = (area2 + area3 + area4) - area1
        area1 = self.area(150, 50, 185, 25, 185, 75)
        area2 = self.area(row, col, 185, 25, 185, 75)
        area3 = self.area(150, 50, row, col, 185, 75)
        area4 = self.area(150, 50, 185, 25, row, col)
        dist4 = (area2 + area3 + area4) - area1
        
        # check rhombus
        area1 = self.area(10, 225, 25, 200, 40, 225)
        area2 = self.area(row, col, 25, 200, 40, 225)
        area3 = self.area(10, 225, row, col, 40, 225)
        area4 = self.area(10, 225, 25, 200, row, col)
        dist5 = (area2 + area3 + area4) - area1
        area1 = self.area(10, 225, 25, 250, 40, 225)
        area2 = self.area(row, col, 25, 250, 40, 225)
        area3 = self.area(10, 225, row, col, 40, 225)
        area4 = self.area(10, 225, 25, 250, row, col)
        dist6 = (area2 + area3 + area4) - area1
        
        # check square
        area1 = self.area(120, 75, 150, 50, 185, 75)
        area2 = self.area(row, col, 150, 50, 185, 75)
        area3 = self.area(120, 75, row, col, 185, 75)
        area4 = self.area(120, 75, 150, 50, row, col)
        dist7 = (area2 + area3 + area4) - area1
        area1 = self.area(120, 75, 150, 100, 185, 75)
        area2 = self.area(row, col, 150, 100, 185, 75)
        area3 = self.area(120, 75, row, col, 185, 75)
        area4 = self.area(120, 75, 150, 100, row, col)
        dist8 = (area2 + area3 + area4) - area1
        
        # check rod
        area1 = self.area(30, 95, 67.5, 30.05, 76.15, 35.5)
        area2 = self.area(row, col, 67.5, 30.05, 76.15, 35.5)
        area3 = self.area(30, 95, row, col, 76.15, 35.5)
        area4 = self.area(30, 95, 67.5, 30.05, row, col)
        dist9 = (area2 + area3 + area4) - area1
        if(dist9 < 1e-5):
            dist9 = 0
        area1 = self.area(30, 95, 76.15, 35.5, 38.66, 100)
        area2 = self.area(row, col, 76.15, 35.5, 38.66, 100)
        area3 = self.area(30, 95, row, col, 38.66, 100)
        area4 = self.area(30, 95, 76.15, 35.5, row, col)
        dist10 = (area2 + area3 + area4) - area1
        if(dist10 < 1e-5):
            dist10 = 0
        
        if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 ==0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
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
