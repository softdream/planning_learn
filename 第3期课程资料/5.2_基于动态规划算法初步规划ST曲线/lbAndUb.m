function [S_lb, S_ub] = lbAndUb(T,S_lb,S_ub)
% 起止时间
t1 = S_lb(1,1);
t2 = S_lb(end,1);

% 补全0~t1时刻的上下限
T1 = (0:0.1:t1-0.1)';
temp = inf(length(T1),1);
S_lb = [[T1, temp]; S_lb];
S_ub = [[T1, -temp]; S_ub];

% 补全t1~T时刻的上下限
T2 = (t2+0.1:0.1:T)';
temp = inf(length(T2),1);
S_lb = [S_lb; [T2, temp]];
S_ub = [S_ub; [T2, -temp]];
end