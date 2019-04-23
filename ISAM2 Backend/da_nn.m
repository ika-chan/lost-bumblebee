function [Maha_mat,L] = da_nn(Measurements, points, pose,sensor_noise)
Maha_mat = [];
L = [];
for k = 1:size(Measurements.p,2)
    for i = 1:length(points)
        Mahalanobis = Maha(Measurements.range(1,k),points{i}.p,points{i}.cov,...
                      pose.p,pose.cov(1:3,1:3),sensor_noise);
        Maha_mat(i,k)=Mahalanobis;
    end
end
for j = 1:size(Measurements.p,2)
    if length(Maha_mat) == 0
        L(j,1) = 0;
    else
    [M,I] = min(Maha_mat(:,j));
    if M < chi2inv(0.9,1)
        L(j,1) = points{I}.index;
    else
        L(j,1) = 0;
    end
    
    end
       
end