function [weight] = SS_Weight(Maha_mat, L, candidate_points)
[r,c] = size(Maha_mat);
weight = [];
for i = 1:c
    summer = 0;
    weighter = zeros(r,1);
    for j = 1:r
        if ismember(candidate_points{j}.index,L)
            summer = summer + Maha_mat(j,i);
        end
    end
    for j = 1:r
        if ismember(candidate_points{j}.index,L)
            weighter(j) = Maha_mat(j,i)/summer;
        end
    end
    
    if weighter == 1
        weighter = 0.5;
    end
    weight = [weight ((1-weighter)./sum((1-weighter)))];
end

            
    