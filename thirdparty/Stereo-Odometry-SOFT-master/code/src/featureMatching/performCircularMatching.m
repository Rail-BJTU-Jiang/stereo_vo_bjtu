function [matches,matches_3_lr] = performCircularMatching(pts1_l, pts2_l, pts1_r, pts2_r, ...
                                           dims, match_params, stereo_matches_1)
%%performCircularMatching Given feature points in a pair of stereo images at
% time instants t-1 and t, perform circular matching using SAD algorithm
%
% INPUT:
%   - pts1_l(1, N1): feature keypoints in images I1_l
%   - pts2_l(1, N2): feature keypoints in images I2_l
%   - pts1_r(1, N3): feature keypoints in images I1_r
%   - pts2_r(1, N4): feature keypoints in images I2_r
%   - dims(1, 2): shape of image (height x width)
%   - match_params: structure comprising of following
%       - match_binsize: matching bin width/height (for computation efficiency)
%       - match_radius: matching radius (dx/dy in pixels)
%       - match_disp_tolerance: dv tolerance for stereo matches (in pixels)
%
% OUTPUT:
%   - matches: array of structure of matched points across four images
%       - pt1_l: matched point in left image at time t-1
%       - pt2_l: matched point in right image at time t
%       - pt1_r: matched point in left image at time t-1
%       - pt2_r: matched point in right image at time t

    height = dims(1);
    width = dims(2);

    if isempty(stereo_matches_1)    
        matches_1_lr = stereo_match(pts1_l, pts1_r, width, height, false, false);
    else
        matches_1_lr = stereo_matches_1;
    end
    
    matches_2_ll = frame_match(pts1_l, pts2_l, width, height, false, false);
    matches_3_lr = stereo_match(pts2_l, pts2_r, width, height, false, false);
    matches_4_rr = frame_match(pts2_r, pts1_r, width, height, false, false);

    indleft = find(~isnan(matches_2_ll(:,2)) & ~isnan(matches_1_lr(:,2)));
    ind2 = ~isnan(matches_3_lr(matches_2_ll(indleft,2),2)); indleft = indleft(ind2);
    ind3 = ~isnan(matches_4_rr(matches_3_lr(matches_2_ll(indleft,2),2),2)); indleft = indleft(ind3);

    circular_matches_1_lr = matches_4_rr(matches_3_lr(matches_2_ll(indleft,2),2),:);

    ind_match_succed = matches_1_lr(indleft,2) == circular_matches_1_lr(:,2);
    indleft = indleft(ind_match_succed);
    
    matches = struct('pt1_l',{pts1_l.location(matches_1_lr(indleft,1),:)},...
                     'pt1_r',{pts1_r.location(matches_1_lr(indleft,2),:)},...
                     'pt2_l',{pts2_l.location(matches_2_ll(indleft,2),:)},...
                     'pt2_r',{pts2_r.location(matches_3_lr(matches_2_ll(indleft,2),2),:)});
    
    fprintf('Matches: %i\t', length(matches.pt1_l));
end
