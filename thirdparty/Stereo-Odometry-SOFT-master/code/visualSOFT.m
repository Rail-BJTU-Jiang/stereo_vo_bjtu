function [R, tr, vo_previous_updated] = visualSOFT(t, I1_l, I2_l, I1_r, I2_r, P1, P2, vo_params, vo_previous)
% VISUALSOFT Given the timestep t , Stereo Odometry based on careful Feature selection
% and Tracking is implemented to estimate the rotation and translation between
% the frames at t-1 and t
%
% INPUT:
%   - t: frame instant for which computation is being performed
%   - I1_l: left image for time t-1
%   - I2_l: left image for time t
%   - I1_r: right images for time t-1
%   - I2_r: right images for time t
%   - P1(3, 4): projection matrix of camera 1 (left)
%   - P2(3, 4): rojection matrix of camera 2 (right)
%   - vo_params: structure containing parameters for the algorithm
%   - vo_previous: structure containing certain data from time step t-1
%
% OUTPUT:
%   - R(3, 3): Rotation estimate between time t-1 and t
%   - tr(1, 3): Translation estimate between time t-1 and t
%   - vo_previous_updated: structure containing certain data from time step t

%% Initialize parameters
dims = size(I2_l);      % dimensions of image (height x width)
time = zeros(4, 1);      % variable to store time taken by each step


%% Feature points extraction
tic;
% % compute new features for current frames
% pts2_l = computeFeatures(I2_l, vo_params.feature);
% pts2_r = computeFeatures(I2_r, vo_params.feature);
% retrieve extracted features from time t-1
pts1_l = vo_previous.pts1_l;
pts1_r = vo_previous.pts1_r;
time(1) = toc;



%% Circular feature matching
tic;
[matches, stereo_matches_2, pts2_l, pts2_r] = track_match(I1_l, I2_l, I1_r, I2_r, pts1_l, pts1_r, ...
    dims, vo_params.feature, vo_params.matcher, vo_previous.stereo_matches);
% [matches, stereo_matches_2] = matchFeaturePoints(I1_l, I1_r, I2_l, I2_r, ...
%     pts1_l, pts2_l, pts1_r, pts2_r, dims, vo_params.matcher, vo_previous.stereo_matches);
time(2) = toc;

%% Feature Selection using bucketing
tic;
bucketed_matches = matches;%bucketFeatures(matches, vo_params.bucketing);
time(3) = toc;

%% Rotation (R) and Translation(tr) Estimation by minimizing Reprojection Error
[R, tr, inlierIdx] = updateMotionP3P(bucketed_matches, P1, P2, dims);

%% plotting

fprintf('Time taken for feature processing: %6.4f\n', time(1));
fprintf('Time taken for feature matching: %6.4f\n', time(2));
fprintf('Time taken for feature selection: %6.4f\n', time(3));
fprintf('Time taken for motion estimation: %6.4f\n', time(4));

% show features before bucketing
% subplot(2, 2, 1);
% % cla reset
% imagesc(cat(3,I2_l,I2_l,I2_l))
% hold on;
% % m_pts2_l = horzcat(bucketed_matches(:).pt2_l);
% % plotFeatures(m_pts2_l,  '+r', 1, 0)
% % show features after bucketing
% m_pts2_l = horzcat(bucketed_matches(:).pt2_l);
% plotFeatures(m_pts2_l,  '.g', 1, 0)
% title(sprintf('matched features at frame %d', t))
% hold off
% % 
% subplot(2, 2, 3);
% cla reset
% imagesc(cat(3,I2_l,I2_l,I2_l));hold on;
% m_pts2_l = horzcat(bucketed_matches.pt1_l(inlierIdx,:));
% m_pts2_2 = horzcat(bucketed_matches.pt2_l(inlierIdx,:));
% % for all matches
% x_from = zeros(1, size(matches, 2));
% x_to = zeros(1, size(matches, 2));
% y_from = zeros(1, size(matches, 2));
% y_to = zeros(1, size(matches, 2));
% num = 0;
% for kk = 1:size(m_pts2_l,1)
%     num = num + 1;
%     x_from(num) = m_pts2_l(kk,1);
%     x_to(num)   = m_pts2_2(kk,1);
%     y_from(num) = m_pts2_l(kk,2);
%     y_to(num)   = m_pts2_2(kk,2);
% end
% % plot line
% plot([y_from; y_to], [x_from; x_to], '-g', 'Linewidth', 1);
% % plot end points
% scatter(y_from, x_from, '.', 'r', 'Linewidth', 2);
% scatter(y_to, x_to, '.', 'b', 'Linewidth', 2);
% % plotFeatures(m_pts2_l,  '+g', 1, 0);hold on;
% % plotFeatures(m_pts2_2,  '+r', 1, 0)
% % showFlowMatches(I1_l, I2_l, bucketed_matches, '-y', 1, '+', 2);
% % % showStereoMatches(I2_l, I2_r, matches, 1, '-y', 1, '+', 2);
% title(sprintf('inlier features in left camera at frame %d', t));

%% Preparation for next iteration
% allocate features detected in current frames as previous
if sum(inlierIdx) < 150
    %% redetect
    pts2_l = computeFeatures(I2_l, vo_params.feature);
    pts2_r = computeFeatures(I2_r, vo_params.feature);
    stereo_matches_2 = [];
end
vo_previous_updated.pts1_l = pts2_l;
vo_previous_updated.pts1_r = pts2_r;
vo_previous_updated.stereo_matches = stereo_matches_2;
end


