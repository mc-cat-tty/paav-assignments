# Dataset 2 Report

Dataset 2 exhibits some false positives - mainly buildings and trees - when the current clustering algo is run with the same parameters used up until now.
However, even the finest tweaking of the paramers cannot solve this issue, since it is instrinsic of the algorithm.

In particular, the issue consists of some buildings being clustered as vehicles. Also some trees seem to be affected by this problem.
The proposed strategy aims to filter out these bounding boxes according to their diagonal and form-factor.

A form-factor that neither a truck nor a car can have is height/max(width, depth) much higher than 1; given that this would mean having
a vehicle more tall than wide. Discarding objects with such ratio helps filtering out trees.
Doing so, also some bicyclers and pedestrians might be excluded from the scene.
This is clearly an unwanted byproduct since pedestrians could represent a danger for autonomous vehicles.

Another condition is imposed on top of the previous one to remove buildings: bounding boxes' diagonal length.
Diagonals higher than a certain threshold are filtered out: they can either be a huge clusters or malformed bounding
boxes, like the sidewalk of scene 2.

In conclusion, these strategies tend to overfit the problem, since a set of parameters suitable for scene 2 could not
be valid for other, more complex, scenes. It would be definitely better to perform PC-base object detection, which although comes at 
an higher computational cost compared to a purely geometric filtering. PointPillars NN provides the means to perform object
detection on a point cloud.