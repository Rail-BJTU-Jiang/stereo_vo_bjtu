# reference github links

https://github.com/Mayankm96/Stereo-Odometry-SOFT

## vo-ref-github
1. https://github.com/wzygzlm?tab=repositories: pal_segmentation using template image and histogram.
3. https://github.com/SimonsRoad/PL_MVO_IROS17: li haoang IROS 2017 code.
4. https://github.com/mattboler/Structureless-Visual-Odometry: gtsam的matlab使用教程。他本身的VO不怎么样。
5. https://github.com/PyojinKim/OPVO

## vo-todo
1. ~~speed-up feature detection: the bottle-neck should be the non-maxima suppression;~~
2. ~~speed-up feature macthing by using tracking;~~: approx. 200 ms per frame. but problem may happen that the updated motion are pretty small.
3. ~~move bucketing to feature detection;~~
4. add key frame;
5. more improvement;
6.  add sub-pixel refinement using parabolic fitting
7.  add feature selection based on feature tracking i.e. the age of features
8.  implement Nister's algorithm and SLERP for rotation estimation
9.  use Gauss-Newton optimization to estimate translation from weighted reprojection error


# results on kitti
1. 00 pretty well;
2. 01 not so good;
4. 03 not so good;
5. 04 not so good; should be the motion estimation problem.
6. 06 ok;
7. 07 ok;
8. 10 ok


### nouse
1. ~~https://github.com/ritwikbera/SOFT_Visual_Odometry~~: pretty slow 7s-8s per frame. code quality is just so so. 
2. ~~https://github.com/avisingh599/vo-howard08~~: pretty slow, 3D-3D motion estimation, use clique to remove outliers. Johannes's work is based on this code.
