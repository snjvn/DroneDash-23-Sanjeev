# Report of Drone Dash 2023
Participant: Sanjeev N (210030022)
Code written by me: whatever is in scripts/demo_node.py

## Objective
The objective has been to navigate, without collision, to an aruco marker and land on it (as fast as possible).

## Navigation Algorithm
Navigates with offboard mode.

First idea: fly up above all the walls (because there's no ceiling!), then there would be no obstacles.

But this is the serious idea:
The image from the depth camera is divided into a 3x3 grid. The drone simply moves towards the brightest square (that is, the square which has more depth). The algorithm contains a restriction on how high the drone can fly.

## Landing Algorithm
Trouble detecting the aruco marker. But the idea was to pass the rgb image to openCV's arucomarker detector. The marker was seen on rviz, but not detected by openCV.

## Simulation results from my end
The 3x3 grid algorithm would let the drone reach the aruco marker in world 1 and 2.

## Possible Improvements
A rather trivial thing- yawing/rotating- has not been implemented. Implementing that would make it possible to navigate world 0.
