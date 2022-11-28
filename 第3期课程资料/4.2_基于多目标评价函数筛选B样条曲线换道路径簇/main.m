clc
clear
close all
load scenario.mat

%%
% 起点信息
idx_A = 85;
startPos = rightWaypoints(idx_A,:);

% 初始化
path = cell(5,5);
cumsumLength = cell(5,5);
totalLength = zeros(5,5);
mranCur = zeros(5,5);
cotrolPt = cell(5,5);
x = [];
y = [];

%% 两层for循环
for idx_O = idx_A+11 : idx_A+15
    for idx_C = idx_A+4 : idx_A+8
        idx_F =  2*idx_O - idx_A;
        idx_D =  idx_F - (idx_C - idx_A);
        idx_B = (idx_A + idx_C)/2;
        idx_E = (idx_D + idx_F)/2;
       
        % 控制点
        P1 = rightWaypoints(idx_A,:);
        P3 = rightWaypoints(idx_C,:);
        P4 = leftWaypoints(idx_D,:);
        P6 = leftWaypoints(idx_F,:);
        P2 = (P1 + P3)/2;
        P5 = (P4 + P6)/2;
        P = [P1; P2; P3; P4; P5; P6]';
        
        % 调用函数生成6控制点3次准均匀B样条曲线
        row = idx_O - idx_A - 10;
        col = idx_C - idx_A -3;
        path{row,col} = B_spline_func(P,5,3);  
        
        % 计算路径的路径长度评价函数
        X_r = path{row,col}(:,1);
        Y_r = path{row,col}(:,2);
        diffX = diff(X_r);
        diffY = diff(Y_r);
        deltDist = sqrt(diffX.^2 + diffY.^2);
        cumsumLength{row,col} = [0;cumsum(deltDist)];
        totalLength(row,col) = sum(deltDist);
        
        % 计算路径的平均曲率
        for i = 1:length(X_r)-2
            cur(i) = getCur(X_r(i:i+2),Y_r(i:i+2));
        end
        cur(end:end+2) = cur(end);
        mranCur(row,col) = mean(abs(cur));
        
        % 控制点集合
        cotrolPt{row,col} = P;
        
        % 构造x,y列向量，用于画三维图
        x(end+1) = idx_O;
        y(end+1) = idx_C;
    end
end


%% 利用多目标转单目标方法求解B样条曲线最优控制点
% 单独取出路径长度、曲率、曲率变化率三个分代价值
f1 = totalLength;
f2 = mranCur;

% 根据权重系数，构造综合目标函数
f1_norm = mapminmax(f1, 0, 1);
f2_norm = mapminmax(f2,0, 1);
alpha = [0.3 0.7];
f_all = alpha(1)*f1_norm + alpha(2)*f2_norm;

% 求解综合代价函数的最小值的索引，从而获得控制点
f_min = min(f_all,[],'all');
idx = find(f_min == f_all);
path_opt = path{idx};
cumLength_opt = cumsumLength{idx};
totalLength_opt = totalLength(idx);
%% 画图
% 画路径簇及最优路径
plot(scenario)
hold on
for i = 1:5
    for j = 1:5
        plot(path{i,j}(:,1),path{i,j}(:,2),'B');
    end
end
plot(path_opt(:,1),path_opt(:,2),'r')

% 画代价函数变化曲面
z = reshape(f_all,length(f_all(:)),1);
[X,Y,Z] = griddata(x,y,z,...
    linspace(min(x),max(x),100)',...
    linspace(min(y),max(y),100));
figure
surf(X,Y,Z)      % 画曲面图
shading flat     % 各小曲面之间不要网格
xlabel('idx_O');
ylabel('idx_C');
zlabel('评价函数值');

%% 保存
save optPath.mat path_opt cumLength_opt totalLength_opt