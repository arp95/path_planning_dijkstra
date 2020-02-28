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

# move is valid 
def IsValid(currRow, currCol, numRows, numCols):
    return (currRow >= 1 and currRow <= numRows and currCol >= 1 and currCol <= numCols)

# checks for an obstacle
def IsObstacle(row, col):
    return False

# action move left
def ActionMoveLeft(currRow, currCol, numRows, numCols):
    if(IsValid(currRow, currCol - 1, numRows, numCols) and IsObstacle(currRow, currCol - 1) == False):
        return True
    return False

# action move right
def ActionMoveRight(currRow, currCol, numRows, numCols):
    if(IsValid(currRow, currCol + 1, numRows, numCols) and IsObstacle(currRow, currCol + 1) == False):
        return True
    return False

# action move up
def ActionMoveUp(currRow, currCol, numRows, numCols):
    if(IsValid(currRow - 1, currCol, numRows, numCols) and IsObstacle(currRow - 1, currCol) == False):
        return True
    return False

# action move down
def ActionMoveDown(currRow, currCol, numRows, numCols):
    if(IsValid(currRow + 1, currCol, numRows, numCols) and IsObstacle(currRow + 1, currCol) == False):
        return True
    return False

# action move right up
def ActionMoveRightUp(currRow, currCol, numRows, numCols):
    if(IsValid(currRow - 1, currCol + 1, numRows, numCols) and IsObstacle(currRow - 1, currCol + 1) == False):
        return True
    return False

# action move right down
def ActionMoveRightDown(currRow, currCol, numRows, numCols):
    if(IsValid(currRow + 1, currCol + 1, numRows, numCols) and IsObstacle(currRow + 1, currCol + 1) == False):
        return True
    return False

# action move left down
def ActionMoveLeftDown(currRow, currCol, numRows, numCols):
    if(IsValid(currRow + 1, currCol - 1, numRows, numCols) and IsObstacle(currRow + 1, currCol - 1) == False):
        return True
    return False

# action move left up
def ActionMoveLeftUp(currRow, currCol, numRows, numCols):
    if(IsValid(currRow - 1, currCol - 1, numRows, numCols) and IsObstacle(currRow - 1, currCol - 1) == False):
        return True
    return False
