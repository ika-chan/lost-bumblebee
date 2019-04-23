function [gt, Measurements, Transform] = Data_Parsing_old(Landmark_location)
j = 1;
file = 0;
load('Relative_total.mat');
load('SE3_FINAL.mat');
while file < 2301

%      Transformation = csvread(strcat(Transform_Location,'\',num2str(file),'.csv'));
    Measurements{j}.centroid_x = [];
    Measurements{j}.centroid_y =[];
    Measurements{j}.p =[];
    Measurements{j}.range =[];
    if file == 0;
        Transform{j}.R = eye(3);
        Transform{j}.p = [0;0;0];
    else
        Transform{j}.R = Relative{file+1}(1:3,1:3);
        Transform{j}.p = Relative{file+1}(1:3,4);
    end
    gt{j}.R = pose{file+1}(1:3,1:3);
    gt{j}.p = pose{file+1}(1:3,4);
        
    try
        M = csvread(strcat(Landmark_location,'\','landmark',num2str(file),'.csv'));   
    catch
        M = [];
    end
    [r,c] = size(M);
    if r ==0 
        Measurements{j} = [];
    else
    for i = 1:1:r
        if M(i,6) == 3
            range = sqrt((M(i,7)-5)^2 + M(i,8)^2 + M(i,9)^2);
            Measurements{j}.centroid_x = [Measurements{j}.centroid_x (M(i,1)+M(i,3))/2];
            Measurements{j}.centroid_y = [Measurements{j}.centroid_y (M(i,2)+M(i,4))/2];
            Measurements{j}.p = [Measurements{j}.p [(M(i,7)-5); M(i,8);M(i,9)]];
            Measurements{j}.range = [Measurements{j}.range range];
        end
    end
    end
    j = j + 1;
    file = file + 1;
end

