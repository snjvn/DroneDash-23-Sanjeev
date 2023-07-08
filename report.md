# Report of Drone Dash 2023
Participant: Sanjeev N (210030022)
Code written by me: whatever is in scripts/demo_node.py

## Objective
The objective has been to navigate, without collision, to an aruco marker and land on it (as fast as possible).

## Navigation Algorithm
Navigates with offboard mode.

### First idea: 
fly up above all the walls (because there's no ceiling!), then there would be no obstacles (and no aruco tag!).

### But this is the serious idea:
The image from the depth camera is divided into a 3x3 grid. The drone simply moves towards the deepest square. This 'depth' assigned to each square is calculated as the mean depth of all pixels in that square.
![WhatsApp Image 2023-07-08 at 23 29 23](https://github.com/snjvn/DroneDash-23-Sanjeev/assets/91363279/ef3564e1-9029-4c9a-8531-6e125a4c7cba)

The algorithm also contains a restriction on how high the drone can fly.

The 5 second delay in between every move (given in original demo_node) was removed. Instead, the drone moves in very small steps, which get executed at a very fast rate- this should make the drone move smoothly.

## Landing Algorithm
Trouble detecting the aruco marker. 
But the idea was to pass the rgb image to openCV's aruco-tag detector. Once the tag was detected, we should land by moving forward and dropping down (then we'll land on the tag).

## Simulation results from my end
The 3x3 grid algorithm would let the drone reach the aruco marker in world 1.
The drone was able to dodge the pillars quite well in world 2.


## Possible Improvements
A rather trivial thing- yawing/rotating- has not been implemented. Implementing that would make it possible to navigate world 0, and reach the aruco tag of world 2.
