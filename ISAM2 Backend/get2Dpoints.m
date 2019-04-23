function points = get2Dpoints(values, marginals)
%get2Dpoints get the 2Dpoint location and covariance from the result
%   point.cord{i}     saves the coordinate of the ith point
%   point.cov{i}      saves the covariance of the ith point
import gtsam.*
keys = KeyVector(values.keys);
% Plot points and covariance matrices
index = 1;
for i = 0:keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Point2')
            points.cov{index} = marginals.marginalCovariance(key);
            points.cord{index} = [
                p.x
                p.y];
            index = index + 1;
    end
end

end

