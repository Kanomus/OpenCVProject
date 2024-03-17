
DOCUMENTATION

Project 1
Medial axis detection of Moving Objects

Purpose:
This code performs video processing using background subtraction, edge detection, and Hough transform to detect and draw lines in a video stream.

Libraries Used:
cv2: OpenCV library for image and video processing.
numpy: Library for numerical operations.

Functions and Classes:
cv2.VideoCapture: Opens the specified video file for reading.
cv2.createBackgroundSubtractorKNN: Creates a background subtractor object using the K-nearest neighbours method.
cv2.getStructuringElement: Generates a structuring element for morphological operations.
cv2.imshow: Displays an image or video frame.
cv2.waitKey: Waits for a key event for a specified amount of time.
cv2.destroyAllWindows: Closes all OpenCV windows.

Processing Loop:
while loop: Iterates through each frame of the video.
vid.read(): Reads the next frame from the video.
cv2.resize: Resizes the frame to half its original size.
cv2.cvtColor: Converts the frame to grayscale.
subtractor.apply: Applies background subtraction to the grayscale frame.
edgedetection.detectedges: Detects edges in the foreground using a custom edge detection method.
hough.hough_lines: Detects lines in the edge-detected image using the Hough transform.
hough.group_lines_by_angle: Groups detected lines based on their angles.
hough.find_longest_group: Finds the longest group of lines.
hough.draw_mean_line: Draws a mean line based on the detected lines.
cv2.imshow: Displays the processed frame.
cv2.waitKey: Waits for a key press to exit the loop.


Hough Class:
Purpose:
This module contains functions for Hough line detection and line grouping.
Functions:
hough_lines(edges, rho_resolution=1, theta_resolution=1, threshold=100): Performs Hough line detection on edge-detected image.
group_lines_by_angle(lines, theta_tolerance=np.pi/36): Groups detected lines based on similar angles.
find_longest_group(grouped_lines): Finds the longest group of lines.
draw_mean_line(frame, lines): Draws a mean line based on the detected lines.

Basic Working of the code:

Firstly we are loading the video using VideoCapture(). Then we created a background substractor using createBackgroundSubtractorKNN() function. Then we created an infinite loop inside of which we read the frames of the video as image. Then we reduce the size of the image to make it easier to work with. Then we applied the background substractor to the image. Then we applied the cv2.erode function to reduce the noise.

 Then we used the sobel_edge_detection from the sobeledge.py folder to get the edges of the image. In sobel edge detection sobel_x and sobel_y are defined. The gradient is calculated by using filter2D function. Magnitude and Direction are also calculated. This edges are stored in edgemap which is typecasted to 8 bit integer and returned.

Alternatively, we can use edgedetection.py
for edgedetection.py
This module contains a custom edge detection algorithm.
detectedges(img, padding=0, stride=1, threshold=10): Detects edges in the input image using a custom edge detection method.
verticalDifferentialKernel: Kernel for vertical differential operation.
horizontalDifferentialKernel: Kernel for horizontal differential operation.


The new_img was made in order to check edges and hough lines. It is not used further in code.

The Hough lines are calculated by functions in hough.py
The basic idea is to get lines then choose lines with similar angle in given tolerance. Then the longest group of lines is chosen and their mean line is calculated and drawn.

The lines are got through the hough_lines  
The function initialises an array called the accumulator. This array serves as a voting space, with cells corresponding to different combinations of rho (distance from the origin) and theta (angle from the horizontal axis). Each cell accumulates votes whenever a potential line passes through a point in the image.
For every edge pixel in the input image, the function iterates through a range of theta values, effectively generating potential lines passing through that edge point. It then computes the corresponding rho value for each theta and increments the accumulator at the corresponding (rho, theta) indices. This process is repeated for all edge pixels, accumulating votes for potential lines.
After the voting process, the function identifies peaks in the accumulator array with a given threshold.
The function converts the peak indices into actual rho-theta pairs, which represent lines in polar coordinates and returns the detected line.

Grouping Lines by Angle:
 group_lines_by_angle function groups the detected lines based on their angle (theta) similarity. It takes a list of lines (lines) and groups them together if their angles are close enough within a given tolerance (theta_tolerance).

Finding Longest Group: 
find_longest_group function finds the longest group of lines among the grouped lines. It returns the longest group of lines.

Drawing Mean Line: 
draw_mean_line function calculates the mean line from a group of lines and draws it on the frame. It takes the frame and a list of lines (lines) as inputs. The mean line is calculated based on the mean rho and theta values of the input lines. 
























Project 2 
Global Planner

Refer to global_planner_final.py in https://github.com/Kanomus/OpenCVProject

This project implements the Rapidly-exploring Random Tree (RRT) algorithm for path planning in the provided map.

Classes
The code consists of two main classes:
1. `treeNode`: Represents a node in the RRT tree.
2. `RRTAlgorithm`: Implements the RRT algorithm for path planning.

 Libraries Used:
- cv2: OpenCV library for image processing.
- numpy: Library for numerical operations.
- matplotlib`: Library for visualisation.

Code Variables and Functions used:
1. `treeNode` :
   - Represents a node in the RRT tree.
   - Attributes:
 	- `locationX`: X-coordinate of the node.
 	- `locationY`: Y-coordinate of the node.
 	- `children`: List of child nodes.
 	- `parent`: Parent node.
   - Methods:
 	- `__init__()`: Initializes the node with its coordinates.

2. RRTAlgorithm` :
   - Implements the RRT algorithm.
   - Attributes:
 	- `randomTree`: Root node of the RRT tree.
 	- `goal`: Goal node.
 	- `rho`: Step size for tree expansion.
 	- `grid`: Binary grid representing obstacles.
 	- `path_distance`: Total distance of the path.
 	- `numWaypoints`: Number of waypoints in the path.
 	- `Waypoints`: List of waypoints in the path.
   - Methods:
 	- `__init__()`: Initializes the RRT algorithm with start, goal, grid, and step size.
 	- `addChild()`: Adds a child node to a given node.
 	- `sampleAPoint()`: Randomly samples a point in the grid.
 	- `steerToPoint()`: Steers the tree towards a target point.
 	- `isInObstacle()`: Checks if there's an obstacle between two points.
 	- `unitVector()`: Calculates the unit vector between two points.
 	- `findNearest()`: Finds the nearest node to a given point.
 	- `distance()`: Calculates the Euclidean distance between two points.
 	- `goalFound()`: Checks if the goal is reached within the step size.
 	- `resetNearestValues()`: Resets the nearest node and distance.
 	- `retraceRRTPath()`: Retraces the path from the goal to the start node.

Code Execution

The code starts with the image input via cv2.imread as an image and a black and gray image, i.e. gray_image.
Kernel  kernel = np.ones((5, 5), np.uint8)  is defined.
The gray image is dilated, so give thicker roads so the nodes can move more quickly.
With the threshold at 254, the binary image gives a better contrast.
grid is a binary image formed form gray image.


The step size for tree expansion is set to 15.
A goal region is defined as a circle around the goal point with a radius equal to the step size. This region is not filled and is visualized in blue.
A new figure named "RRT Algorithm" is created for visualization.
The map image (image) is displayed using imshow() with a grayscale colormap.
The start and goal points are plotted as red circles ('ro' for start and 'bo' for goal) on the map.
The goal region circle is added to the plot using add_patch() function of the axis.
This region is not filled (fill=False) and is colored blue (color='b').
Labels for the X and Y axes are set using xlabel() and ylabel() functions respectively.
The labels include units in meters for clarity.

When the left mouse button is clicked (cv2.EVENT_LBUTTONDOWN), it captures the coordinates (x, y) of the click and sets the start point to these coordinates.
It then creates a copy of the original image (image_copy), draws the start point on this copy using drawPoints(), and displays the modified image with the start point highlighted.
When the right mouse button is clicked (cv2.EVENT_RBUTTONDOWN), it similarly captures the coordinates and sets the goal point.
It also creates a copy of the original image, draws the goal point on this copy, and displays the modified image with the goal point highlighted.
This function is responsible for capturing mouse events and updating the displayed image with the selected start and goal points.

Setting Mouse Callback and Displaying Image:
The original map image (image) is displayed using cv2.imshow().
The Capture_Event() function is set as the mouse callback function using cv2.setMouseCallback(). This ensures that mouse events on the image are captured and processed by the Capture_Event() function.
The program waits for a key press using cv2.waitKey(0) after the mouse events are processed. If the key pressed is Enter (key code 13) and both start and goal points are selected (goal and start are not None), then the program destroys all windows and exits.
This part of the code sets up the environment for selecting start and goal points by displaying the map image and capturing mouse events. It enables the user to interactively select the start and goal points on the map image.

Then we created a RRTAlgorithm object called rrt with the required parameters and started an infinite loop which will run until we reach the goal point. 
Inside the loop, we first reset the nearest values using the resetNearest values so that the last iteration won’t effect this one. then we created a random point using sampleAPoint method. We then found the nearest node to that point by using findNearest and then we created a new point in the direction of that random point using steerToPoint method. Then we checked if there is any obstacle between the new node and old one using the isInObstacle method. If we got false, that means there is no obstacle, then we add the new node as a child of the old one. Then we plot that node in green with a 0.1 sec pause so that it is visible to our eyes. Then we again repeat the loop and keep finding new nodes in random directions till we reach the goal. When we reach the goal, the last condition becomes true and we break out from the loop. 
Then we call the retraceRRTPath method to get the nodes of the path and then prints the number of waypoints, path distance, and also the coordinates of the individual waypoints. We then plot the waypoints with red using a for loop.

Explanation of Functions

The class has attributes such as randomTree which represents the root of the tree, goal which represents the goal node, rho which is the step size, grid which is the grid of obstacles, path_distance to keep track of the distance covered in the path, nearestDist to keep track of the nearest distance during the search, numWaypoints to store the number of waypoints in the path, and Waypoints which stores the waypoints found during the search.
The RRT algorithm typically operates by iteratively expanding the tree from the start node towards the goal node while randomly sampling points in the configuration space and connecting them to the nearest node in the tree, subject to certain constraints (e.g., avoiding obstacles). 

addChild: used to add a child node to the nearest node in the tree. If the child's x-coordinate matches the goal's x-coordinate, the child will be set as a direct child of the goal node, otherwise, a new node is created with the specified location and added as a child to the nearest node.

sampleAPoint: This method randomly samples a point in the grid. It generates random x and y coordinates within the range of the grid's dimensions and returns the sampled point.

steerToPoint: This method is used to steer from one point towards another point within a certain step size (rho). It calculates the offset vector from the start location towards the end location with a magnitude of rho. Then, it checks if the resulting point lies within the grid boundaries and if it's not obstructed by any obstacles. If the point is valid, it returns the coordinates of the point, otherwise, it returns [-1, -1].
isInObstacle: This method checks if there is any obstacle between two given locations (locationStart and locationEnd). It does so by generating points along the line connecting the start and end locations, and checking if any of these points lie within an obstacle in the grid. If such a point is found, it returns True, indicating that there is an obstacle in the path. Otherwise, it returns False

findNearest: This method recursively searches for the nearest node in the tree to a given point. It updates self.nearest Node and self.nearestDist with the nearest node and its distance to the given point, respectively.

goalFound: This method checks if the goal has been reached within the step size (rho). It calculates the distance between the goal node and a given point and returns True if the distance is less than or equal to rho.

resetNearestValues: This method resets the attributes nearestNode and nearestDist to their initial values. It's useful for clearing previous values before performing a new search or computation.

retraceRRTPath: This method retraces the path from the goal node back to the start node. It does so recursively by tracing through the parent nodes of each node until it reaches the start node. Along the way, it collects waypoints which represent the path from the goal to the start node.

drawPoints: This function draws points on an image using OpenCV. It takes an image (img) as input along with optional start and goal points. If start or goal points are provided, circles representing these points are drawn on the image in blue (for start) and red (for goal). 

Capture_Event: This callback function is intended to handle mouse events (specifically, left and right button clicks) on an OpenCV window. When the left mouse button is clicked (cv2.EVENT_LBUTTONDOWN), it records the coordinates of the click as the start point. When the right mouse button is clicked (cv2.EVENT_RBUTTONDOWN), it records the coordinates of the click as the goal point. It then updates the image with the newly drawn points using drawPoints and displays the updated image.
