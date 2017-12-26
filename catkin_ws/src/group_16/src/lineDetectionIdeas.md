INPUT: 1080 x 1092 pixel rgb image as cv::Mat
OUTPUT: x-velocity and z-rotation, both as float64


## Use Canny-edge-detection
Canny detection finds edges on a picture. Those are then used for a new line detection.

1) Scale down to reduce pixel.
2) Make greyscale, because Canny transform only works on greyscale.


## Hough transform
Hough transform finds lines by fitting a lot of possibilities and finding ones
with at least x fitting points. Those are then returned as a line.

1) Scale down to reduce pixel
2) Make greyscale, because Hough transform only works on greyscale.
3) Take lines, find best fitting line.
4) Take point on line, calculate output for direction to that point.


## Use neural network
Neural networks are trained by giving it in- and output data and letting in figure
out, what the method to calculate is.

1) Perform principal component analysis to reduce image to ~50 dimensions.
2) Train network by telling what to do with training data.
3) Output is direct.

Network structure:
- 50 float inputs
- x hidden layer
- 2 outputs


## Use reinforcement learning
The robot is trained by driving around and being told, when he does something wrong/
correct. He can be trained by just sitting next to it and pressing buttons.

1) Reduce picture data to have only small amount of variables.
2) Train RL by let it run and press +/- for good/bad actions.
3) RL gets reduced image as input, outputs direct.