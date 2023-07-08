# Report of Drone Dash 2023
Participants: Sanjeev N (210030022)

## Objective
The objective has been to navigate, without collision, to the aruco marker and land on it (as fast as possible).

## Navigation Algorithm
Testing had been done with the offboard mode, so the drone stops for sometime after every move.

The image from the depth camera is divided into a 2x2 grid (or 3x3 if possible). The drone simply moves towards the brightest square (that is, the square which is more deep).

## Landing Algorithm
not made yet

## Simulation results from my end
The 2x2 grid algorithm would let the drone reach the aruco marker in world 1.
