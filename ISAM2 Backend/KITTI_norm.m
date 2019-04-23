function [tError, RError] = KITTI_norm(SE3_1_pre,SE3_1_cur,SE3_2_pre,SE3_2_cur)
%SE3norm
Rel_1 = SE3_1_pre\SE3_1_cur;
Rel_2 = SE3_2_pre\SE3_2_cur;
error = Rel_1\Rel_2;
error_p = error(1:3,4);
error_R = error(1:3,1:3);
tError = sqrt(error_p'*error_p);

a = error_R(1,1);
b = error_R(2,2);
c = error_R(3,3);
d = 0.5*(a+b+c-1);
RError = acos(max(min(d,1),-1));
end

