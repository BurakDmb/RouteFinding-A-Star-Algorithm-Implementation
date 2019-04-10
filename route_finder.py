
import time
import numpy as np
import cv2
import math
import heapq
#ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
#ptvsd.wait_for_attach()

plot = True

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
    radius = 4
    stepSize = 3

    while True:
        things, obstacles= generateThings(radius, radius, 6, size)

        #image = drawGrid(box_count, size, things)  

        came_from, cost_so_far, last = a_star_search(obstacles, things[0], things[1], stepSize, radius, size)
        path= createRoute(came_from, things[0], last)
        #print(path)
        
        if path:
            #image = drawGrid(box_count, size, things, path[1:-1])  
            newPath = [(i,j) for (i,j,k) in path]
            x = [i for (i,j,k) in path]
            y = [j for (i,j,k) in path]
            #print("Path:")
            #print(newPath)
            if plot:
                from route_plotter import plotGraph
                plotGraph(things, obstacles, path=(x,y))
        else:
            print("infeasible")
            continue

        k = cv2.waitKey(0) & 0xFF
        #If key ESC pressed
        if (k == 27):
            cv2.destroyAllWindows()
            break
        else:
            cv2.destroyAllWindows()
        

## TODO: Needs to implement collision control in generating objects.
def generateThings(obstacleRadius, thingRadius, obstacleCount, maxSize):
    
    tmpList1 = []
    tmpList2 = []

    randPoints = np.random.rand(2,3)
    randPoints[:, 0] = (thingRadius + randPoints[:, 0] * (maxSize-2*thingRadius))
    randPoints[:, 1] = (thingRadius + randPoints[:, 1] * (maxSize-2*thingRadius))
    randPoints[:, 2] = (randPoints[:, 2] * 360)
    tmpList1.append(Thing(Position(randPoints[0,0], randPoints[0,1]), randPoints[0,2], thingRadius, "Start"))
    tmpList1.append(Thing(Position(randPoints[1,0], randPoints[1,1]), randPoints[1,2], thingRadius, "End"))

    counter=0
    while counter<obstacleCount:
        randPoints = np.random.rand(1,3)
        randPoints[0,0] = (obstacleRadius + randPoints[0,0] * (maxSize-2*obstacleRadius)).astype(int)
        randPoints[0,1] = (obstacleRadius + randPoints[0,1] * (maxSize-2*obstacleRadius)).astype(int)
        randPoints[0,2] = (randPoints[0,2] * 360).astype(int)
        tmpThing = Thing(Position(randPoints[0,0], randPoints[0,1]),randPoints[0,2], obstacleRadius, "Obstacle")
        if not isCollided(tmpList2, (tmpThing.x, tmpThing.y), obstacleRadius, 5):
            tmpList2.append(tmpThing)
            counter+=1    
    return tmpList1, tmpList2



        

def createRoute(came_from, start, end):
    path = []
    path.append((end[0], end[1], end[2]))
    tmp=(end[0],end[1], end[2])
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


def heuristic(x1, y1, theta1, x2, y2, theta2):
    
    return 1*(2*math.pi-math.radians(abs(theta2-theta1))) + 1*(abs(x1-x2)+abs(y1-y2))
    return abs(x1 - x2) + abs(y1 - y2)
    #return 0
    #return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    

def isCollided(objects, current, thingRadius, safeRadius):
    for i in range(len(objects)):
        if (abs(current[0]- objects[i].x) <= thingRadius + objects[i].radius + safeRadius) \
            and (abs(current[1]- objects[i].y) <= thingRadius + objects[i].radius + safeRadius):
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


#burada priority queue kisminda bir hata var istenileni secmiyor
def a_star_search(objects, start, goal, stepSize, radius, maxSize, maxItCount = 200000):
    
    h = []
    heapq.heappush(h, (0, (start.x, start.y, start.angle)))
    #print "Start",(start.x, start.y, start.angle)
    #print "Goal",(goal.x, goal.y, goal.angle)
    came_from = {}
    cost_so_far = {}
    came_from[(start.x, start.y, start.angle)] = None
    cost_so_far[(start.x, start.y, start.angle)] = 0
    iteration =0
    while len(h)>0 and iteration < maxItCount:
        current = heapq.heappop(h)
        if abs(current[1][0]-goal.x) < start.radius and abs(current[1][1] - goal.y) < start.radius and abs(current[1][2] - goal.angle) < 20:
            
            break
        for angle in [ -20, 0, 20]:
            next =(current[1][0] + stepSize*math.sin(math.radians(current[1][2] + angle)), current[1][1] + stepSize*math.cos(math.radians(current[1][2] + angle)), current[1][2] + angle)
            new_cost = cost_so_far[current[1]] + stepSize
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                if next[0] > (maxSize - 1) or next[0] < 0 or next[1] > (maxSize - 1) or next[1] < 0:
                    continue
                if not isStepAvaible(objects, next, start.radius, radius, maxSize):
                    continue
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal.x, goal.y, goal.angle, next[0], next[1], next[2])
                #breakpoint()
                heapq.heappush(h, (priority, next))
                came_from[next] = current[1]
                #print(current[1], next, priority, came_from[next], iteration)
        iteration+=1
        #print(iteration, len(h))
                
    return came_from, cost_so_far, next


if __name__ == '__main__':
    main()
