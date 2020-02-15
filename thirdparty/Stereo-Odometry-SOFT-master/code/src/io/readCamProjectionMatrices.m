function [P1, P2] = createCamProjectionMatrices(data_params)
%CREATECAMPROJECTIONMATRICES Creates projection matrices for a stereo
%camera. It is assumed that the parameters are for rectified images.
%
% Input:
%   - data_params: contains calib file.
%
% Output:
%   - P1(3, 4): projection matrix for the left camera
%   - P2(3, 4): projection matrix for the right camera
      fid = fopen(data_params.calib_file,'r');
      while ~feof(fid)
            linecnt = fgetl(fid);
            cntcls = split(linecnt,':');
            if strcmp(cntcls{1},'P0')
                  % for left camera
                  vP1 = str2num(cntcls{2});
                  P1 = [vP1(1:4);vP1(5:8);vP1(9:12)];
            elseif strcmp(cntcls{1},'P1')
                  % for right camera
                  vP2 = str2num(cntcls{2});
                  P2 = [vP2(1:4);vP2(5:8);vP2(9:12)];
            end
      end
end

