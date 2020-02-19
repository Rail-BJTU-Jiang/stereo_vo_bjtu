function [keypoints, num] = nonMaximumSuppression(I_f1, I_f2, nms_n, nms_tau, margin)
%NONMAXIMUMSUPPRESION Extracting corner and blob feature points 
% Non-maximum- and non-minimum-suppression are employed over the
% filtered images, resulting in feature candidates which belong
% to one of four classes (i.e., blob min (0), blob max (1), corner min (2),
% corner max(3))
%
% Source: Algorithm 4 in "Efficient Non-Maximum Suppression", Alexander 
%         Neubeck and Luc Van Gool, ICPR'06
%
% INNPUT:
%   - I_f1: output of filtered image of blob kernel
%   - I_f2: output of filtered image of checkerboard kernel
%   - nms_n: min. distance between maxima (in pixels)
%   - nms_tau: interest point peakiness threshold
%   - margin: leaving margin for safety 
%
% OUTPUT:
%   - keypoints(1, N): keypoints detected. 
%                   Each keypoints has: (location(x, y), value, class)
%   - num: Number of keypoints detected

% size of image
[height, width] = size(I_f1);

% number of feature points
num = 0;

% create a structure to store keypoints
keypoints = struct('location', {}, 'value', {}, 'class', {});

% extracting features through non-maximum and non-minimum suppresions
% for both filtered images (blobs and corners)
for i = (nms_n + margin):(nms_n + 1):(height - nms_n - margin)
    for j = (nms_n + margin):(nms_n + 1):(width - nms_n - margin)
        % loop variables
%         f1min_i = i; f1min_j = j; f1max_i = i; f1max_j = j;
%         f2min_i = i; f2min_j = j; f2max_i = i; f2max_j = j;
        
        % get intensity values
%         f1min_val = I_f1(i, j);
%         f1max_val = f1min_val;
%         f2min_val = I_f2(i, j);
%         f2max_val = f2min_val;
        
        % Step a)
        % Partitions input image into blocks of sizes (n + 1) x (n + 1).
        % Search for maxima/minima within each block.
%         tic
        I1_block = I_f1(i:i+nms_n,j:j+nms_n);
        I2_block = I_f2(i:i+nms_n,j:j+nms_n);
        
        [f1min_val, minid1] = min(I1_block(:));[f1min_i,f1min_j]=ind2sub(size(I1_block),minid1);f1min_i=f1min_i+i-1;f1min_j=f1min_j+j-1;
        [f1max_val, minid2] = max(I1_block(:));[f1max_i,f1max_j]=ind2sub(size(I1_block),minid2);f1max_i=f1max_i+i-1;f1max_j=f1max_j+j-1;
        [f2min_val, minid3] = min(I2_block(:));[f2min_i,f2min_j]=ind2sub(size(I1_block),minid3);f2min_i=f2min_i+i-1;f2min_j=f2min_j+j-1;
        [f2max_val, minid4] = max(I2_block(:));[f2max_i,f2max_j]=ind2sub(size(I1_block),minid4);f2max_i=f2max_i+i-1;f2max_j=f2max_j+j-1;
%         disp([[f1min_i,f1min_j],[f1max_i,f1max_j],[f2min_i,f2min_j],[f2max_i,f2max_j]]);
%         toc;
%         tic
%         for i2 = i:(i + nms_n)
%             for j2 = j:(j + nms_n)
%                 % for blob detector
%                 currval = I_f1(i2, j2);
%                 if (currval < f1min_val)         
%                     f1min_i   = i2;
%                     f1min_j   = j2;
%                     f1min_val = currval;
%                 elseif (currval > f1max_val) 
%                     f1max_i   = i2;
%                     f1max_j   = j2;
%                     f1max_val = currval;
%                  end
%                 % for corner detector
%                 currval = I_f2(i2, j2);
%                 if (currval < f2min_val)         
%                     f2min_i   = i2;
%                     f2min_j   = j2;
%                     f2min_val = currval;
%                 elseif (currval > f2max_val) 
%                     f2max_i   = i2;
%                     f2max_j   = j2;
%                     f2max_val = currval;
%                  end
%             end
%         end
%         disp([[f1min_i,f1min_j],[f1max_i,f1max_j],[f2min_i,f2min_j],[f2max_i,f2max_j]]);
%         toc;
        
        % Step b)
        % Test at the full neighborhood of candidate is tested. Elements 
        % of the block itself are skipped, since they have already been
        % considered by construction
        
        % for f1 minimum
        failed_f1min = checkMinimumValidity(I_f1, f1min_val, f1min_i, f1min_j, nms_n, margin);
        
        if not(failed_f1min)
            if (f1min_val <= - nms_tau)
                num = num + 1;
                keypoints(num).location = [f1min_i, f1min_j];
                keypoints(num).value = f1min_val;
                keypoints(num).class = 1;
            end
        end
        
        % for f1 maximum
        failed_f1max = checkMaximumValidity(I_f1, f1max_val, f1max_i, f1max_j, nms_n, margin);

        if not(failed_f1max)
            if (f1max_val >= nms_tau)
                num = num + 1;
                keypoints(num).location = [f1max_i, f1max_j];
                keypoints(num).value = f1max_val;
                keypoints(num).class = 2;
            end
        end
        
        % for f2 minimum
        failed_f2min = checkMinimumValidity(I_f2, f2min_val, f2min_i, f2min_j, nms_n, margin);
        
        if not(failed_f2min)
            if (f2min_val <= - nms_tau)
                num = num + 1;
                keypoints(num).location = [f2min_i, f2min_j];
                keypoints(num).value = f2min_val;
                keypoints(num).class = 3;
            end
        end
        
        % for f2 maximum
        failed_f2max = checkMaximumValidity(I_f2, f2max_val, f2max_i, f2max_j, nms_n, margin);
        
        if not(failed_f2max)
            if (f2max_val >= nms_tau)
                num = num + 1;
                keypoints(num).location = [f2max_i, f2max_j];
                keypoints(num).value = f2max_val;
                keypoints(num).class = 4;
            end
        end
    end
end

end
