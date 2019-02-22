
import time
import numpy as np
import cv2
from Queue import PriorityQueue
import math

def main():
    box_count = 100
    size = 700
    radius = 5
    stepSize = 1
    while True:
        grid = np.zeros((box_count, box_count))
        generateObstacles(grid, 20, box_count)
        start = generateTargets(grid, radius, 2)
        end = generateTargets(grid, radius, 3)
        

        
        came_from, cost_so_far = a_star_search(grid, start, end, stepSize, radius)
        path= createRoute(came_from, start, end)
        if path:
            image = drawGrid(box_count, size, grid, path[1:-1])  
        else:
            print("infeasible")
            continue
        k = cv2.waitKey(0) & 0xFF
        if (k == 27):
            cv2.destroyAllWindows()
            break
        elif (k==97):
            print(type(image))
        else:
            cv2.destroyAllWindows()

def createRoute(came_from, start, end):
    path = []
    path.append(end)
    tmp=end
    if tmp in came_from:
        while came_from[tmp]:
            path.append(came_from[tmp])
            if came_from[tmp] in came_from:
                tmp = came_from[tmp]
            else:
                return None
        return path[::-1]
    else:
        return None

def drawGrid(box_count, size, grid, path):
    image = np.array(np.ones((size, size, 3))*255 ,dtype=np.uint8)
    
    y_start = 0
    y_end = size
    ss = size // box_count

    for x in range(0, size, ss):
        cv2.line(image, (x, y_start), (x, y_end), (0, 0, 0))
    cv2.line(image,(size-1, y_start), (size-1, y_end),(0, 0, 0))
    x_start = 0
    x_end = size

    for y in range(0, size, ss):
        cv2.line(image, (x_start, y), (x_end, y), (0, 0, 0))
    cv2.line(image, (x_start, size-1), (x_end, size-1), (0, 0, 0))
    for i,j in path:
        pts=np.array([[i*ss, j*ss], [(i)*ss, (j+1)*ss], [(i+1)*ss, (j+1)*ss], [(i+1)*ss, (j)*ss]])
        cv2.fillPoly(image, [pts], color=(0, 255, 255))
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j]==1:
                pts=np.array([[i*ss, j*ss], [(i)*ss, (j+1)*ss], [(i+1)*ss, (j+1)*ss], [(i+1)*ss, (j)*ss]])
                cv2.fillPoly(image, [pts], color=(0, 128, 255))
            elif grid[i][j]==2:
                pts=np.array([[i*ss, j*ss], [(i)*ss, (j+1)*ss], [(i+1)*ss, (j+1)*ss], [(i+1)*ss, (j)*ss]])
                cv2.fillPoly(image, [pts], color=(0, 255, 0))
            elif grid[i][j]==3:
                pts=np.array([[i*ss, j*ss], [(i)*ss, (j+1)*ss], [(i+1)*ss, (j+1)*ss], [(i+1)*ss, (j)*ss]])
                cv2.fillPoly(image, [pts], color=(255, 0, 0))

    cv2.imshow('Route Finding',image)
    return image

def generateObstacles(grid, count, maxOSize):
    size=grid.shape[0]
    obstacles=(np.random.rand(count,2)*size).astype(int)
    for i in range(count):
        spreadObstacle(grid, tuple((obstacles[i][0], obstacles[i][1])), 0, maxOSize)
        

def spreadObstacle(grid, obstacle, currentIt, maxIt):
    
    lottery=(np.random.rand(1,1)*4).astype(int)
    if currentIt==0:
        grid[obstacle[0]][obstacle[1]]=1
    if currentIt==maxIt:
        return

    maxSize=grid.shape[0]-1
    if lottery==0 and obstacle[1]!=0: #add to the left block
        grid[obstacle[0]][obstacle[1]-1]=1
        spreadObstacle(grid, tuple((obstacle[0],obstacle[1]-1)), currentIt+1, maxIt)

    elif lottery==1 and obstacle[0]!=0: #add to the up block
        grid[obstacle[0]-1][obstacle[1]]=1
        spreadObstacle(grid, tuple((obstacle[0]-1,obstacle[1])), currentIt+1, maxIt)
    
    elif lottery==2 and obstacle[1]!=maxSize: #add to the right block
        grid[obstacle[0]][obstacle[1]+1]=1
        spreadObstacle(grid, tuple((obstacle[0],obstacle[1]+1)), currentIt+1, maxIt)
    
    elif lottery==3 and obstacle[0]!=maxSize: #add to the down block
        
        grid[obstacle[0]+1][obstacle[1]]=1
        spreadObstacle(grid, tuple((obstacle[0]+1,obstacle[1])), currentIt+1, maxIt)

    else:
        spreadObstacle(grid, tuple((obstacle[0],obstacle[1])), currentIt+1, maxIt)
    
    return


def generateTargets(grid, targetRadius, targetVal):
    randPoints = (np.random.rand(2,2)*grid.shape[0]).astype(int)
    if checkNodesAvaible(grid, randPoints[0], targetRadius, targetRadius):
        for i in range(targetRadius):
            for j in range(targetRadius):
                grid[randPoints[0][0]+i][randPoints[0][1]+j]=targetVal
                grid[randPoints[0][0]-i][randPoints[0][1]+j]=targetVal
                grid[randPoints[0][0]+i][randPoints[0][1]-j]=targetVal
                grid[randPoints[0][0]-i][randPoints[0][1]-j]=targetVal
        return tuple((randPoints[0][0], randPoints[0][1]))
    else:
        return generateTargets(grid, targetRadius, targetVal)

def checkNodesAvaible(grid, center, xRange, yRange):
    for i in range(xRange):
        for j in range(yRange):
            if center[0]-i < 0 or center[0]+i > grid.shape[0]-1 or center[1]-j < 0 or center[1]+j >grid.shape[1]-1:
                return False
            else:
                if (grid[center[0]+i][center[1]+j] > 0 and
                    grid[center[0]-i][center[1]+j] > 0 and 
                    grid[center[0]+i][center[1]-j] > 0 and 
                    grid[center[0]-i][center[1]-j] > 0):
                    return False
    return True
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)
    #return 0
    #return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
def issStepAvaible(grid, current, next, radius):
    xDirection = -1 if current[0]>next[0] else 1
    yDirection = -1 if current[1]>next[1] else 1
    for i in range(abs(current[0]-next[0])+1):
        for j in range(abs(current[1]-next[1])+1):
            for k in range(radius):
                for l in range(radius):
                    if current[0] +i*xDirection-k < 0 or current[0] +i*xDirection+k > grid.shape[0]-1 or current[1] +j*yDirection-l < 0 or current[1] +j*yDirection+l > grid.shape[1]-1:
                        return False
                    if grid[current[0] +i*xDirection+k][current[1] +j*yDirection+l]==1:
                        return False
                    if grid[current[0] +i*xDirection-k][current[1] +j*yDirection+l]==1:
                        return False
                    if grid[current[0] +i*xDirection+k][current[1] +j*yDirection-l]==1:
                        return False
                    if grid[current[0] +i*xDirection-k][current[1] +j*yDirection-l]==1:
                        return False
    return True

def isStepAvaible(grid, next, radius):

    for k in range(radius):
        for l in range(radius):
            if next[0]-k < 0 or next[0]+k > grid.shape[0]-1 or next[1]-l < 0 or next[1]+l > grid.shape[1]-1:
                return False
            if grid[next[0]+k][next[1]+l]==1:
                return False
            if grid[next[0]-k][next[1]+l]==1:
                return False
            if grid[next[0]+k][next[1]-l]==1:
                return False
            if grid[next[0]-k][next[1]-l]==1:
                return False
    return True



def a_star_search(grid, start, goal, stepSize, radius):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for new_position in [(0, -stepSize), (0, stepSize), (-stepSize, 0), (stepSize, 0)]:
            next = (current[0] + new_position[0], current[1] + new_position[1])
            new_cost = cost_so_far[current] + stepSize
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                if next[0] > (grid.shape[0] - 1) or next[0] < 0 or next[1] > (grid.shape[1] - 1) or next[1] < 0:
                    continue
                if not isStepAvaible(grid, next, radius):
                    continue
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

if __name__ == '__main__':
    main()
