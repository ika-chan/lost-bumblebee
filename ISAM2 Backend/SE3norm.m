function result = SE3norm(SE3_1,SE3_2)
%SE3norm 
result = norm(logm(SE3_1\SE3_2),'fro');
end

