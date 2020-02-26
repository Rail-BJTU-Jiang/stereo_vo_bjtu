# reference github links

~~https://github.com/Mayankm96/Stereo-Odometry-SOFT~~

## vo-todo
1. ~~speed-up feature detection: the bottle-neck should be the non-maxima suppression;~~
2. ~~speed-up feature macthing by using tracking;~~: approx. 200 ms per frame. but problem may happen that the updated motion are pretty small.
3. ~~move bucketing to feature detection;~~
4. add key frame;
6. ~~add sub-pixel refinement using parabolic fitting~~
7. add feature selection based on feature tracking i.e. the age of features
8. implement Nister's algorithm and SLERP for rotation estimation
9. use Gauss-Newton optimization to estimate translation from weighted reprojection error.
10. more improvement;


## results on kitti
1. 00 pretty well;
2. 01 not so good;
4. 03 not so good;
5. 04 not so good; should be the motion estimation problem.
6. 06 ok;
7. 07 ok;
8. 10 ok;

---

## vo-ref-github
3. https://github.com/SimonsRoad/PL_MVO_IROS17: li haoang IROS 2017 code.
4. https://github.com/mattboler/Structureless-Visual-Odometry: gtsam的matlab使用教程。他本身的VO不怎么样。
5. https://github.com/PyojinKim/OPVO

---

### nouse
1. ~~https://github.com/ritwikbera/SOFT_Visual_Odometry~~: pretty slow 7s-8s per frame. code quality is just so so. 
2. ~~https://github.com/avisingh599/vo-howard08~~: pretty slow, 3D-3D motion estimation, use clique to remove outliers. Johannes's work is based on this code.
3. ~~https://github.com/alexkreimer/odometry~~: 方法靠谱，通过算两个H来分别计算R和t，但是代码及其乱且没有注释。投稿BMVC并没有中。**不过有一个生成latex table的工具还可以。还有一个python批量跑MATLAB的脚本。**

---

### MSCKF (next key work)
1. https://github.com/foreversuiyi/SlidingWindowOptimization: toronto 那里有MSCKF版本。
2. https://github.com/isamabdullah88/Visual-Inertial-Odometry：对比此版本和toronto那边的版本。
3. https://github.com/mramezani64/Stereo-Visual-Inertial-Odometry： stereo MSCKF.

## fore-end object segmentation and tracking
1. ~~https://github.com/wzygzlm?tab=repositories: pal_segmentation using template image and histogram.~~ ：效果不是很好，而且template matching的部分尤其不好。

**maryland**
2. ~~https://github.com/anirudhtopiwala/ENPM-673-Perception-for-Autonomous-Robots~~: code quality is poor, only the gmm part maybe useful. but now i know how to use gmmdist.
3. ~~https://github.com/sudrag/Perception-and-Computer-Vision-in-MATLAB~~: 和2都是Maryland的课程作业，不过code quality is better.
4. https://github.com/StevieG47/Matlab-ComputerVision: 和2-3一样，Maryland vision课程作业。

maryland这里关注3个点：
1. GMM
2. KLT
3. MSER

GMM需要训练数据：
1. 用whycon source code做detection，然后把结果保存下来，保存结果为，detect的bb box位置和尺寸。
2. gmm目前来看适用于单纯的颜色分割，是pixel level的图像分割。

Template matching and tracking:
[lagadic / camera_localization](https://github.com/lagadic/camera_localization)



### scramuzza's course on vision for mobile robots (monocular vo) try this
2. ~~https://github.com/simon-schaefer/vodom~~： code quality is good, but monocular and slow. nice thing is that it includes a re-initialization.
3. ~~https://github.com/aroumie1997/visual-odometry-project~~: code quality is ok. 
4. ~~https://github.com/bpfel/VisualOdometry_Matlab~~: code quality is good. no new things.
5. ~~https://github.com/nicovanduijn/visualOdometry~~：与123很相似，没有太大的差别。
6. ~~https://github.com/ToniRV/visual-odometry-pipeline~~: similar to 1-5. 此人很牛，在MIT了。可以关注其github.
7. ~~https://github.com/ynager/visual_odometry_pipeline~~. code质量应该是最好的一个，目前在ETH读博士。
8. ~~https://github.com/jukindle/VisionAlgosVOPipeline~~: led relative pose estimation的code.
9. https://github.com/samuelnyffenegger/VAMR-project: 参考其BA。



## useful code section
```
%% Project 2- Anirudh Topiwala
% Reference taken by Dr. Mubarak Shah’s Lecture
function [F, inliers] = Ransac(oldPoints, newPoints, imgSize)

%% This function is used to divide the points in 8x8 grid and then select 8 random points for estimating fundamental matrix.
  
% Convert image points to homogeneous coordinates by adding 1.
    oldPoints = [oldPoints, ones(length(oldPoints),1)];
    newPoints = [newPoints, ones(length(newPoints),1)];

    % Making an 8x8 grid 
    grid = cell(8,8);
    gridlength = zeros(8); resolution = imgSize/8;
    % Dividing points into the 64 grids depending upon their position.
    for i = 1:8
        for j = 1:8
            grid{i,j} = find((oldPoints(:,2)>((i-1)*resolution(1)))&(oldPoints(:,2)<=(i*resolution(1)))&(oldPoints(:,1)>((j-1)*resolution(2)))&(oldPoints(:,1)<=(j*resolution(2))));
            gridlength(i,j) = numel(grid{i,j});
        end
    end
    filledgrid = find(gridlength~=0); % indexing the grids which have atleast some length or which are not empty.
    
    % Randomly selecting which index will be used. 
    k = 0; gridcheck = zeros(1000,8);
    indexcheck = zeros(1000,8);
    while k < 1000
        k = k+1;
        gridcheck(k,:) = filledgrid(randperm(length(filledgrid),8)); % Getting 8 random indices
        for index = 1:8
            i = rem(gridcheck(k,index),8);
            if i == 0
                i = 8;
            end
            j = ceil(gridcheck(k,index)/8);
            indexcheck(k,index) = randi(gridlength(i,j));
        end
        % Checking if there is a repetition in grid index selected. If yes, then rerun the loop. 
        if k~=1   
            samegrid = find(all(gridcheck(1:k-1,:)==gridcheck(k,:),2));  
            if ~isempty(samegrid)
                if any(all(indexcheck(samegrid,:)==indexcheck(k,:),2))
                    k = k - 1;
                end
            end
        end
    end
    i = rem(gridcheck,8);
    i(i==0) = 8;
    j = ceil(gridcheck/8);
    
    %% Once we have the the points sorted into the grid, we now find the best fundamental matrix
    points1 = zeros(8,3); points2 = zeros(8,3);
    maxinliers = 0; inliers = [];
    for k = 1:1000
        % Get the 8 points to calculate fundamental matrix.
        for index = 1:8
            points1(index,:) = oldPoints(grid{i(k,index),j(k,index)}(indexcheck(k,index)),:);
            points2(index,:) = newPoints(grid{i(k,index),j(k,index)}(indexcheck(k,index)),:);
        end
        
        % Estimate the fundamental matrix
        F = getfundamentalmatrix(points1, points2);
        
        % Calculate error
        ep1 = F*oldPoints'; ep2 = F'*newPoints';
        e = sum(newPoints*F.*oldPoints,2).^2./(sum(ep1(1:2,:).^2)'+sum(ep2(1:2,:).^2)');
        % Storing the maximum number of inliers
        totinliers = e <= 0.01;
        if maxinliers < sum(totinliers)
            maxinliers = sum(totinliers);
            inliers = totinliers;
        end
    end
    
    %% Estimate the fundamental matrix using max inliers
    F = getfundamentalmatrix(oldPoints(inliers,:), newPoints(inliers,:)); % Calculating fundamental with max inliers.

end
```
