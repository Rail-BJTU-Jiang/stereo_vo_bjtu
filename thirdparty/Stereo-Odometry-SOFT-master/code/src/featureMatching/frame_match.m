function matches = frame_match(keypoints1, keypoints2, w, h)
    pts1 = int32(vertcat(keypoints1.location));
    pts2 = int32(vertcat(keypoints2.location));
    
    desc1 = vertcat(keypoints1.descriptor);
    desc2 = vertcat(keypoints2.descriptor);
    
    desc1bi=reshape(de2bi(desc1')',size(desc1,2)*8,[])';
    desc2bi=reshape(de2bi(desc2')',size(desc2,2)*8,[])';
    
%     ind1 = pts1(:,1)+pts1(:,2)*w;
%     ind2 = pts2(:,1)+pts2(:,2)*w;
    
    max_disparity = 20;
    radius = 20;
    
    matches = NaN(size(pts1,1),2);
        
    lu_corner = -max_disparity*w - radius;
    rb_corner =  max_disparity*w + radius;
    
    for i = 1:size(pts1,1)
%         ind = ind1(i);
        %% search correspondence
        validind = zeros(1,size(pts2,1),'logical');
        validind(pts2(:,1) >= (pts1(i,1)-radius) & pts2(:,1) <= (pts1(i,1)+radius) & ...
            pts2(:,2) >= (pts1(i,2)-max_disparity) & pts2(:,2) <= (pts1(i,2)+max_disparity)) = 1;
%         for j = -max_disparity:max_disparity
%             minind = ind - j*w - radius;
%             maxind = ind - j*w + radius;
%             validind(ind2 >= minind & ind2 <=maxind) = 1;
%         end

        validind = find(validind);
        %% extract descriptor
        if ~isempty(validind)
            validdesc2 = desc2bi(validind,:);
            % matching
            dists = pdist2((desc1bi(i,:)), (validdesc2), 'hamming');
            % find keypoint in image2 associated with minimum cost
            [~,minid] = min(dists);
            matches(i,:) = [i, validind(minid)];
            %[~, indices] = sort(dists, 'ascend');
            %min_ind = valid_in2(indices(1));
            %validity = true;
        end
    end
    matches(isnan(matches(:,1)),:) = [];
end