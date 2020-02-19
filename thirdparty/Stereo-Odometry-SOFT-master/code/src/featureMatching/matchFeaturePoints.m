function varargout = matchFeaturePoints(I1_l, I1_r, I2_l, I2_r, ...
                                      pts1_l, pts2_l, pts1_r, pts2_r, ...
                                      dims, match_params, stereo_matches_1)
%%MATCHFEATUREPOINTS Given feature points in two images, the function returns a
% set of points that have ben matched using SAD algorithm
%
% INPUT:
%   - I1_l: left image for time t-1
%   - I1_r: right images for time t-1
%   - I2_l: left image for time t
%   - I2_r: right image for time t
%   - pts1_l(1, N1): feature keypoints in images I1_l
%   - pts2_l(1, N2): feature keypoints in images I2_l
%   - pts1_r(1, N3): feature keypoints in images I1_r
%   - pts2_r(1, N4): feature keypoints in images I2_r
%   - dims(1, 2): shape of image (height x width)
%   - match_params: structure comprising of following
%       - match_binsize: matching bin width/height (for computation efficiency)
%       - match_radius: matching radius (dx/dy in pixels)
%       - match_ncc_window: window size of the patch for normalized cross-correlation
%       - match_ncc_tolerance: threshold for normalized cross-correlation
%       - refinement: pixel location refinement (0=none,1=pixel,2=subpixel)
%
% OUTPUT:
%   - matches: array of structure of matched points across four images
%       - pt1_l: matched point in left image at time t-1
%       - pt2_l: matched point in right image at time t
%       - pt1_r: matched point in left image at time t-1
%       - pt2_r: matched point in right image at time t

    % perform circular matching
%     matches = performCircularMatching(pts1_l, pts2_l, pts1_r, pts2_r, dims, match_params);

    height = dims(1);
    width = dims(2);

    if isempty(stereo_matches_1)    
        matches_1_lr = stereo_match(pts1_l, pts1_r, width, height, false, false);
    else
        matches_1_lr = stereo_matches_1;
    end
    
    %% debug plot
%     im = cat(2,I1_l,I1_r);
%     imshow(im)
%     hold on;
%     pt1 = fliplr(cat(1,pts1_l.location));
%     pt2 = fliplr(cat(1,pts1_r.location));
%     plot(pt1(matches_1_lr(:,1),1),pt1(matches_1_lr(:,1),2),'r.')
%     plot(pt2(matches_1_lr(:,2),1)+size(I1_l,2),pt2(matches_1_lr(:,2),2),'g.')
%     plot([pt1(matches_1_lr(:,1),1)';pt2(matches_1_lr(:,2),1)'+size(I1_l,2)],[pt1(matches_1_lr(:,1),2)';pt2(matches_1_lr(:,2),2)'],'b-')
%     
    matches_2_ll = frame_match(pts1_l, pts2_l, width, height, false, false);
    %% debug plot
%     im = cat(1,I2_l,I1_l);
%     imshow(im)
%     hold on;
%     pt1 = fliplr(cat(1,pts1_l.location));
%     pt2 = fliplr(cat(1,pts2_l.location));
%     plot(pt2(matches_2_ll(:,2),1),pt2(matches_2_ll(:,2),2),'r.')
%     plot(pt1(matches_2_ll(:,1),1),pt1(matches_2_ll(:,1),2)+size(I1_l,1),'g.')
%     plot([pt1(matches_2_ll(:,1),1)';pt2(matches_2_ll(:,2),1)'],[pt1(matches_2_ll(:,1),2)'+size(I1_l,1);pt2(matches_2_ll(:,2),2)'],'b-')
%     
    matches_3_lr = stereo_match(pts2_l, pts2_r, width, height, false, false);

    %% debug plot
%     im = cat(2,I2_l,I2_r);
%     imshow(im)
%     hold on;
%     pt1 = cat(2,pts2_l.location);
%     pt2 = cat(2,pts2_r.location);
%     plot(pt1(matches_3_lr(:,1),1),pt1(matches_3_lr(:,1),2),'r.')
%     plot(pt2(matches_3_lr(:,2),1)+size(I1_l,2),pt2(matches_3_lr(:,2),2),'g.')
%     plot([pt1(matches_3_lr(:,1),1)';pt2(matches_3_lr(:,2),1)'+size(I1_l,2)],[pt1(matches_3_lr(:,1),2)';pt2(matches_3_lr(:,2),2)'],'b-')

    matches_4_rr = frame_match(pts2_r, pts1_r, width, height, false, false);
    %% debug plot
%     im = cat(1,I1_r,I2_r);
%     imshow(im)
%     hold on;
%     pt1 = cat(2,pts2_r.location);
%     pt2 = cat(2,pts1_r.location);
%     plot(pt2(matches_4_rr(:,2),1),pt2(matches_4_rr(:,2),2),'r.')
%     plot(pt1(matches_4_rr(:,1),1),pt1(matches_4_rr(:,1),2)+size(I1_l,1),'g.')
%     plot([pt1(matches_4_rr(:,1),1)';pt2(matches_4_rr(:,2),1)'],[pt1(matches_4_rr(:,1),2)'+size(I1_l,1);pt2(matches_4_rr(:,2),2)'],'b-')    
    
    indleft = find(~isnan(matches_2_ll(:,2)) & ~isnan(matches_1_lr(:,2)));
    ind2 = ~isnan(matches_3_lr(matches_2_ll(indleft,2),2)); indleft = indleft(ind2);
    ind3 = ~isnan(matches_4_rr(matches_3_lr(matches_2_ll(indleft,2),2),2)); indleft = indleft(ind3);

    circular_matches_1_lr = matches_4_rr(matches_3_lr(matches_2_ll(indleft,2),2),:);

    ind_match_succed = matches_1_lr(indleft,2) == circular_matches_1_lr(:,2);
    indleft = indleft(ind_match_succed);
    
%     tic
    matches = struct('pt1_l',{pts1_l.location(matches_1_lr(indleft,1),:)},...
                     'pt1_r',{pts1_r.location(matches_1_lr(indleft,2),:)},...
                     'pt2_l',{pts2_l.location(matches_2_ll(indleft,2),:)},...
                     'pt2_r',{pts2_r.location(matches_3_lr(matches_2_ll(indleft,2),2),:)});
    
%     toc
%     tic
%     matches = cell(1, length(indleft));
%     for i = 1:length(indleft)
%         ii = indleft(i);
%         matches{i}.pt1_l = pts1_l.location(matches_1_lr(ii,1),:);
%         matches{i}.pt1_r = pts1_r.location(matches_1_lr(ii,2),:);
%         matches{i}.pt2_l = pts2_l.location(matches_2_ll(ii,2),:);
%         matches{i}.pt2_r = pts2_r.location(matches_3_lr(matches_2_ll(ii,2),2),:);
%     end
%     matches = cell2mat(matches);
%     toc
    fprintf('Matches: %i\t', length(matches.pt1_l));
    
    varargout{1} = matches;
    varargout{2} = matches_3_lr;
end
