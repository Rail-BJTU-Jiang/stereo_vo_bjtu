function gt = read_oxts(oxts_path)
    txtfiles = dir(fullfile(oxts_path,'/*.txt'));
    for i = 1:length(txtfiles)
        if strcmp(txtfiles(i).name,'times.txt') == 1 || strcmp(txtfiles(i).name,'calib.txt') == 1
            continue;
        else
            
        end
        fid = fopen(fullfile(txtfiles(i).folder,txtfiles(i).name),'r');
        cnt = fgetl(fid);
        data = str2num(cnt);
        if length(data) ~= 12
            continue;
        else
            gt = data;
            while ~feof(fid)
               cnt = fgetl(fid);
               data = str2num(cnt);
               gt(end+1,:) = data;
            end
        end
    end
end