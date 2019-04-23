function poses = get2Dposes(values, marginals)
%get2Dposes get the 2Dpose location and covariance from the result
%   poses.cord{i}     saves the coordinate of the ith point
%   poses.cov{i}      saves the covariance of the ith point
import gtsam.*
keys = KeyVector(values.keys);
% Plot points and covariance matrices
index = 1;
for i = 0:keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Pose2')
            poses.cov{index} = marginals.marginalCovariance(key);
            poses.cord{index} = [
                p.x
                p.y];
            index = index + 1;
    end
end

end

