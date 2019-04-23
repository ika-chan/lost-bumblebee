
file_derectory = 'C:\Users\pwestra\Downloads\2011_09_30_drive_0018_sync\2011_09_30\2011_09_30_drive_0018_sync\oxts\data\';
rad_2_degree = pi/180;
pre_multiply = [-0.1713 0.9849 0.0265;...
                -0.9847 -0.1721 0.0276;...
                0.0317 -0.0213 0.9993];
                
for i = 0:1:2761
    groundtruth = regexp(fileread(sprintf('%s/%010d.txt',file_derectory,i)),'\','split');
    K = str2num(groundtruth{1});
    K(1) = K(1)*rad_2_degree;
    K(2) = K(2)*rad_2_degree;
    if i == 0
        
        x = 6371*1000*cos(K(1))*cos(K(2));
        y = 6371*1000*cos(K(1))*sin(K(2));
        z = K(3);
        rot{i+1} = [1 0 0;0 1 0;0 0 1];
    end
    if i == 0
        trans{i+1} = [0 0 0];
    end
    if i > 0
        x1 = 6371*1000*cos(K(1))*cos(K(2));
        y1 = 6371*1000*cos(K(1))*sin(K(2));
        trans{i+1} = [x1-x,y1-y,K(3)-z];
    end
    
    rot1 = [1 0 0;0 cos(K(5)) -sin(K(5));0 sin(K(5)) cos(K(5))];
    rot2 = [cos(K(4)) 0 sin(K(4)); 0 1 0;-sin(K(4)) 0 cos(K(4))];
    rot3 = [cos(K(6)) -sin(K(6)) 0;sin(K(6)) cos(K(6)) 0;0 0 1];
    rot{i+1} = pre_multiply*rot1*rot2*rot3;
end
num = 1:length(rot);

groundTruthPoses = table(num',trans',rot');
groundTruthPoses.Properties.VariableNames = {'ViewId','Location','Orientation'};
images = imageDatastore('C:\Users\pwestra\Downloads\2011_09_30_drive_0018_sync\2011_09_30\2011_09_30_drive_0018_sync\image_03\data');
