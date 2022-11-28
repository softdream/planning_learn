clc
clear
close all
load  optPath.mat
load  v_QP.mat
%% 由于速度序列个数小于路径序列个数，浓缩路径序列
dt = 0.1;
cumS = 0;
idx = [];
for i = 2:size(v_QP,1)
    cumS = cumS + v_QP(i,2)*dt;
    for j = 1:length(cumLength_opt)-1
        if cumLength_opt(j) <= cumS && cumS < cumLength_opt(j+1)
            idx(end+1) = j;
            break
        end
    end
end

%% 保存
refPath = path_opt(idx,:);
refSpeed = v_QP(2:end,2);
save refTrajectory.mat refPath refSpeed