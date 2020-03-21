# Astar_algorithm

Implementation of Astar algorithm in python

## Authors

 1. Sri Manika Makam
 2. Pradeep Gopal

## Overview

 Implemented the Astar algorithm for a rigid robot.

## Dependencies

 1. numpy library
 2. cv2 library
 3. queue library
 4. math library
 5. time library
 6. argparse library
 7. Python 3.6
 8. ubuntu 16.04
 
## Instructions to run

The inputs are coordinates of start point, orientation of start point, coordinates of goal point, robot radius, clearance, theta (the angle between the action at each node) and step size. All the inputs are float. The orientation of goal point is taken by default as 0.

Go to the directory where code is present and run the following command

```
python Astar_rigid.py --user_input 1
```
If 'user_input' is 1, then user is allowed to give inputs of his wish. 
If 'user_input' is 0:

start point = (50.0, 30.0, 60.0), theta = 30.0, goal point = (150, 150, 0), robot_radius = 1.0, clearance = 1.0, step_size = 1.0

The program will be terminated when you press any key on the keyboard after the path is displayed.

## Output

The time taken by the algorithm to find the shortest path for default inputs is approximately 9 seconds.
The time taken to find the shortest path including the live exploration of nodes is approximately 320 seconds.

The program will display the explored nodes as the search space is explored and will display the shortest path after it is found.

The video output can be accessed here:
https://drive.google.com/drive/folders/1AgRXgyogmuREwNg90ecDwI6LoKeRHZlb?usp=sharing

