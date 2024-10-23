# Dataset 2 Report

Dataset 2 exhibits some issues when the current clustering algo is run with the same parameters used up until now.
However, even the finest tweaking of the paramers cannot solve this issue, since it is instrinsic of the algorithm.

In particular, the issue consists of some buildings being clustered as vehicles.
Also some trees seem to be affected by this problem.

The proposed strategy aims to filter out these huge bounding boxes according to their size and form-factor.
A form-factor that neither a truck nor a car can have is height/width higher than 1; given that this would mean having
a vehicle more tall than wide. Doing so some bicyclers and pedestrians might be excluded from the scene.
This is clearly an unwanted behaviour since pedestrians could represent a danger for autonomous vehicles.

For this reason another condition is imposed on top of the previous one: bounding box size.
Boxes with the aforementioned form-factor and a limited number of points are accepted anyway.