import cv2
import random
import numpy as np
import matplotlib.pyplot as plt

class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None
        
class RRTAlgorithm():
    def __init__(self, start, goal, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.rho = stepSize
        self.grid = grid
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
    
    def addChild (self, locationX, locationY):
        if(locationX==self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX, locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
            
    def sampleAPoint(self):
        x = random.randint(0, grid.shape[1]-1)
        y = random.randint(0, grid.shape[0]-1)
        point = np.array([x, y])
        return point
    
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0]-1
        return point
    
    #check if obstacle lies between the start node and end point of the edge
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = locationStart.locationX + i*u_hat[0]
            testPoint[1] = locationStart.locationY + i*u_hat[1]
            #check if testPoint lies within obstacle
            if self.grid[int(round(testPoint[1])),int(round(testPoint[0]))] == 0:
                return True
        return False
    
    #find unit vector between a node and an end point which form a vector
    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd [1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    #find the nearest node from a given unconnected point (Euclidean distance)
    def findNearest (self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
        pass
    
    #find euclidean distance between a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist
    
    #check if the goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <=self.rho:
            return True
        pass
    
    #reset nearestNode and nearest Distance
    def resetNearestValues (self):
        self.nearestNode = None
        self.nearestDist = 10000
        
    #trace the path from goal node to start node
    def retraceRRTPath(self, goal):
        #end the recursion when goal node reaches the start node
        if (goal.locationX == self.randomTree.locationX):
            return
        self.numWaypoints += 1
        #insert currentPoint to the Waypoints array from the beginning
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0, currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)
        
#Loading the image
gray_image = cv2.imread("map_image.png", 0)
kernel = np.ones((5, 5), np.uint8) 
img_erosion = cv2.erode(gray_image, kernel, iterations=1) 

# making binary
threshold_value = 220
_, inverted_binary_map = cv2.threshold(img_erosion, threshold_value, 255, cv2.THRESH_BINARY)

grid = cv2.bitwise_not(inverted_binary_map)

# Define parameters
start = np.array([674, 333])  # Starting point
goal = np.array([1291, 370])  # Goal point

stepSize = 15
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='gray')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#Begin
rrt = RRTAlgorithm (start, goal, grid, stepSize)
i=1
while True:
    #Reset nearest values
    rrt.resetNearestValues()
    print("Iteration: ",i)
    i+=1
    #algorithm begins here
    point = rrt.sampleAPoint()
    rrt.findNearest (rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if (bool == False):
        rrt.addChild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go',markersize = 2, linestyle="--")
        #if goal found, append to path
        if (rrt.goalFound(new)):
            rrt.addChild(goal[0], goal[1])
            print("Goal found!")
            break
        
#trace back the path returned, and add start to waypoints
rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0, start)
print("Number of waypoints: ", rrt.numWaypoints) 
print("Path Distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)

#plot the waypoints
for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt. Waypoints[i+1][0]], [rrt. Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro',linestyle="--")
    plt.pause(0.10)
    
cv2.waitKey(3000)
