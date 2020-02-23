function varargout = track_match(I1_l, I2_l, I1_r, I2_r, pts1_l, pts1_r, ...
    dims, feature_params, match_params, stereo_matches_1)

    lk_params = {'WinSize',[15 15], 'MaxLevel', 2, ...
        'Criteria',struct('type','Count+EPS', 'maxCount',10, 'epsilon',0.03)};
    
    [pts1_l, pts2_l, ind] = trackFeatures(I1_l, I2_l, pts1_l, lk_params, feature_params);
    [pts1_r, pts2_r, ~] = trackFeatures(I1_r, I2_r, pts1_r, lk_params, feature_params);
    
    height = dims(1);
    width = dims(2);

    if isempty(stereo_matches_1)    
        matches_1_lr = stereo_match(pts1_l, pts1_r, width, height, false, false);
    else
        matches_1_lr = stereo_match(pts1_l, pts1_r, width, height, false, false);
    end
    
    matches_3_lr = stereo_match(pts2_l, pts2_r, width, height, false, false);

    indleft = find(~isnan(matches_1_lr(:,2)));
    ind2 = ~isnan(matches_3_lr(indleft,2)); 
    indleft = indleft(ind2);

    circular_matches_1_lr = matches_3_lr(indleft,:);

    ind_match_succed = matches_1_lr(indleft,2) == circular_matches_1_lr(:,2);
    indleft = indleft(ind_match_succed);
    
    matches = struct('pt1_l',{pts1_l.location(matches_1_lr(indleft,1),:)},...
                     'pt1_r',{pts1_r.location(matches_1_lr(indleft,2),:)},...
                     'pt2_l',{pts2_l.location(matches_1_lr(indleft,1),:)},...
                     'pt2_r',{pts2_r.location(matches_3_lr(matches_1_lr(indleft,1),2),:)});
                 
    matches.pt1_l = cv.cornerSubPix(I1_l, matches.pt1_l);
    matches.pt1_r = cv.cornerSubPix(I1_r, matches.pt1_r);
    matches.pt2_l = cv.cornerSubPix(I2_l, matches.pt2_l);
    matches.pt2_r = cv.cornerSubPix(I2_r, matches.pt2_r);

    
    fprintf('Matches: %i\t', length(matches.pt1_l));
    
    varargout{1} = matches;
    varargout{2} = matches_3_lr;
    varargout{3} = pts2_l;
    varargout{4} = pts2_r;
end

function [pts1, pts2, ind] = trackFeatures(I1, I2, pts1, lk_params, feature_params)
    % apply filters
    [height, width] = size(I1);
    [I_dx, I_dy] = sobel5x5(I2);

    p1 = fliplr(vertcat(pts1.location));
    p1=num2cell(p1,2)';
    
    [p2, status] = cv.calcOpticalFlowPyrLK(I1, I2, p1, lk_params{:});
    p2 = reshape(cell2mat(p2),2,[])';p2 = round(fliplr(p2));
    
    ind = (status == 1) & ...
        p2(:,2) >= feature_params.margin & p2(:,2) <= (width-feature_params.margin) & ...
        p2(:,1) >= feature_params.margin & p2(:,1) <= (height-feature_params.margin);
    
    pts1.location = pts1.location(ind,:);
    %pts1.class = pts1.class(ind,:);
    %pts1.value = pts1.value(ind,:);
    pts1.descriptor = pts1.descriptor(ind,:);
    
    pts2 = struct('location', p2(ind,:));
    
    pts2 = computeDescriptors_fast(I_dx, I_dy, pts2);
end

