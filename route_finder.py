
import time
import numpy as np
import cv2
import math
import ptvsd
import heapq

#ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
#ptvsd.wait_for_attach()


class Position:
    def __init__(self, posX, posY):
        self.x = posX
        self.y = posY
    def __repr__(self):
        return "X: "+str(self.x)+", Y: "+str(self.y)

class Thing:
    def __init__(self, pos, ang, r, t):
        self.x = pos.x
        self.y = pos.y
        self.angle = ang
        self.radius = r 
        self.type = t
    def __repr__(self):
        return self.type + ", X: "+str(self.x)+", Y: "+str(self.y)+", Angle: "+str(self.angle)+", Radius: "+ str(self.radius)


def main():
    box_count = 200
    size = 200
    radius = 5
    stepSize = 5

    while True:
        things = generateThings(radius, radius, 15, size)

        #image = drawGrid(box_count, size, things)  

        came_from, cost_so_far = a_star_search(things[2:], things[0], things[1], stepSize, radius, size)
        path= createRoute(came_from, things[0], things[1])
        #print(path)
        
        if path:
            image = drawGrid(box_count, size, things, path[1:-1])  
            print("Path:")
            print(path)
        else:
            print("infeasible")
            continue

        k = cv2.waitKey(0) & 0xFF
        #If key ESC pressed
        if (k == 27):
            cv2.destroyAllWindows()
            break
        #If key a pressed
        elif (k==97):
            print(type(image))
        else:
            cv2.destroyAllWindows()
        

## TODO: Needs to implement collision control in generating objects.
def generateThings(obstacleRadius, thingRadius, obstacleCount, maxSize):
    
    tmpList = []

    randPoints = np.random.rand(2,3)
    randPoints[:, 0] = (thingRadius + randPoints[:, 0] * (maxSize-2*thingRadius)).astype(int)
    randPoints[:, 1] = (thingRadius + randPoints[:, 1] * (maxSize-2*thingRadius)).astype(int)
    randPoints[:, 2] = (randPoints[:, 2] * 360).astype(int)
    tmpList.append(Thing(Position(randPoints[0,0], randPoints[0,1]), randPoints[0,2], thingRadius, "Start"))
    tmpList.append(Thing(Position(randPoints[1,0], randPoints[1,1]), randPoints[1,2], thingRadius, "End"))

    counter=0
    while counter<obstacleCount:
        randPoints = np.random.rand(1,3)
        randPoints[0,0] = (obstacleRadius + randPoints[0,0] * (maxSize-2*obstacleRadius)).astype(int)
        randPoints[0,1] = (obstacleRadius + randPoints[0,1] * (maxSize-2*obstacleRadius)).astype(int)
        randPoints[0,2] = (randPoints[0,2] * 360).astype(int)
        tmpThing = Thing(Position(randPoints[0,0], randPoints[0,1]),randPoints[0,2], obstacleRadius, "Obstacle")
        if not isCollided(tmpList, (tmpThing.x, tmpThing.y), obstacleRadius, 5):
            tmpList.append(tmpThing)
            counter+=1    
    return tmpList


def drawGrid(box_count, size, things, path):
    image = np.array(np.ones((size, size, 3))*255 ,dtype=np.uint8)
    
    y_start = 0
    y_end = size
    ss = size // box_count

    #for x in range(0, size, ss):
        #cv2.line(image, (x, y_start), (x, y_end), (0, 0, 0))
    #cv2.line(image,(size-1, y_start), (size-1, y_end),(0, 0, 0))
    x_start = 0
    x_end = size

    #for y in range(0, size, ss):
    #    cv2.line(image, (x_start, y), (x_end, y), (0, 0, 0))
    #cv2.line(image, (x_start, size-1), (x_end, size-1), (0, 0, 0))

    for i,j in path:
        pts=np.array([[i*ss, j*ss], [(i)*ss, (j+1)*ss], [(i+1)*ss, (j+1)*ss], [(i+1)*ss, (j)*ss]], dtype=np.int32)
        #print(pts)
        cv2.fillPoly(image, [pts], color=(0, 0, 255))
    for i in range(len(things)):
        pts=np.array([[things[i].x-things[i].radius, things[i].y-things[i].radius],
        [things[i].x-things[i].radius, things[i].y+things[i].radius],
        [things[i].x+things[i].radius, things[i].y+things[i].radius],
        [things[i].x+things[i].radius, things[i].y-things[i].radius]]).astype(np.int64)
        if things[i].type == "Obstacle":
            cv2.fillPoly(image, [pts], color=(0, 128, 255))
        elif things[i].type == "Start":
            cv2.fillPoly(image, [pts], color=(0, 255, 0))
        elif things[i].type == "End":
            cv2.fillPoly(image, [pts], color=(255, 0, 0))
    cv2.imshow('Route Finding',image)
    return image


def createRoute(came_from, start, end):
    path = []
    path.append((end.x, end.y))
    tmp=(end.x,end.y)
    if tmp in came_from:
        while came_from[tmp]:
            path.append(came_from[tmp])
            if came_from[tmp] in came_from:
                tmp = came_from[tmp]
            else:
                break
        return path[::-1]
    else:
        return None


def heuristic(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)
    #return 0
    #return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    

def isCollided(objects, current, thingRadius, safeRadius):
    for i in range(len(objects)):
        if (abs(current[0]- objects[i].x) <= thingRadius + objects[i].radius + safeRadius) and (abs(current[1]- objects[i].y) <= thingRadius + objects[i].radius + safeRadius):
            return True
    return False

def isStepAvaible(objects, next, thingRadius, safeRadius, maxSize):
    if not isCollided(objects, next, thingRadius, safeRadius):
        if next[0] < 0 or next[0] > (maxSize-1) or next[1] < 0 and next[1] < (maxSize-1):
            return False
        else:
            return True
    else:
        return False
    return True


#burada priority queue kisminda bir hata var istenileni secmiyor
def a_star_search(objects, start, goal, stepSize, radius, maxSize):
    
    h = []
    heapq.heappush(h, (0, (start.x, start.y, start.angle)))
    #print "Start",(start.x, start.y, start.angle)
    #print "Goal",(goal.x, goal.y, goal.angle)
    came_from = {}
    cost_so_far = {}
    came_from[(start.x, start.y, start.angle)] = None
    cost_so_far[(start.x, start.y, start.angle)] = 0
    iteration =0
    while len(h)>0:
        current = heapq.heappop(h)
        if current[1][0] == goal.x and current[1][1] == goal.y:
            break
        for new_position in [(0, -stepSize), (0, stepSize), (-stepSize, 0), (stepSize, 0)]:
            next = (current[1][0] + new_position[0], current[1][1] + new_position[1])
            new_cost = cost_so_far[current[1]] + stepSize
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                if next[0] > (maxSize - 1) or next[0] < 0 or next[1] > (maxSize - 1) or next[1] < 0:
                    continue
                if not isStepAvaible(objects, next, start.radius, radius, maxSize):
                    continue
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal.x, goal.y, next[0], next[1])
                #breakpoint()
                heapq.heappush(h, (priority, next))
                came_from[next] = current[1]
                #print(current[1], next, priority, came_from[next], iteration)
        iteration+=1
                
    return came_from, cost_so_far


if __name__ == '__main__':
    main()
