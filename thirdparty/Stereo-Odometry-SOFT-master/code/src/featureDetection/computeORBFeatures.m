function keypts_with_descriptors = computeORBFeatures(I, feature_params)
    [h,w]=size(I);
    bucket_num_x = floor(w/feature_params.bucket_width);
    bucket_num_y = floor(h/feature_params.bucket_height);
    max_num = feature_params.bucket_max_features;

    bucket_size_y = feature_params.bucket_height;
    bucket_size_x = feature_params.bucket_width;
    
    detector = cv.FeatureDetector('ORB');
%     detector.MaxFeatures = max_num;  %def:500

    for i = 1:bucket_num_y
        for j = 1:bucket_num_x
            %% patches
            orig_row = 1+(i-1)*bucket_size_y;
            orig_col = 1+(j-1)*bucket_size_x;
            roi = I(orig_row:i*bucket_size_y,orig_col:j*bucket_size_x);
            keypts_patch = detector.detect(roi);
            if(length(keypts_patch) > max_num)
                %% ideally, impossible
                keypts_patch = keypts_patch(1:max_num);
            end
            %% to their original position
            for k = 1:length(keypts_patch)
                keypts_patch(k).pt = keypts_patch(k).pt + [orig_col,orig_row];
            end
            %% attach to the list
            if(size(keypts_patch,2)~=0 && exist('keypts', 'var')==0)
                keypts = keypts_patch;
            elseif exist('keypts', 'var')~=0
                keypts(:,end+1:end+size(keypts_patch,2)) = keypts_patch;
            end
        end
    end
    
    %% compute descriptors
    extractor = cv.DescriptorExtractor('ORB');
    descriptors = extractor.compute(I, keypts);
    
%     out = cv.drawKeypoints(I, keypts, 'Color',[255 0 0]);
%     imshow(out), title('ORB')
    keypts_with_descriptors.keypts = keypts;
    keypts_with_descriptors.location = vertcat(keypts.pt);
    keypts_with_descriptors.descriptor = vertcat(descriptors);

%     for i = 1:size(keypts, 2)
%         % extract keypoint location
%         keypts_with_descriptors(i).location = keypts(i).pt;
%         keypts_with_descriptors(i).descriptor = descriptors(i,:);
%     end
end
