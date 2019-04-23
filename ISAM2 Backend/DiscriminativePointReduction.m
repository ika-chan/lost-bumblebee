function [points] = DiscriminativePointReduction(points,pose)
point_temp = [];
counter = 1;
if length(points) == 0
    points = point_temp;
    return;
end
for i = 1:length(points.p)
GPV = points.p{i} - pose.p; % Global Position Vector
RPV = pose.R ^(-1) * GPV; % Relative Position Vector
range = sqrt(points.p{i}'*points.p{i});
    if RPV(1) > 0 
        point_temp{counter}.index = i;
        point_temp{counter}.p = points.p{i};
        % Landmark location noise in global frame
        point_temp{counter}.cov = pose.R*points.cov{i}*pose.R';             
        counter = counter + 1;
    end
end
points = point_temp;

        
        
        
