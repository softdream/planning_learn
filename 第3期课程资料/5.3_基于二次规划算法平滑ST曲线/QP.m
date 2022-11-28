clc
clear
close all
load stGragh.mat
load ST_opt.mat
%% 参数初始化
% 性能函数指标权重
w1 = 0.2;   % 加速度权重
w2 = 1;     % 加加速度权重
w3 = 1e3;   % 与动态规划得到的最优ST曲线的误差指标权重

% DP和QP的离散时间和阶段数量
deltT_DP = 1;
deltT_QP = 0.1;
N_DP = size(ST_opt,1) - 1;
N_QP = deltT_DP / deltT_QP;

% 约束
v_max = 16;         % 最大速度
v_min = 0;          % 最小速度
acc_max = 3;        % 最大加速度
acc_min = -3;       % 最小加速度
v_ini = 12;         % 初始速度
a_ini = 0;          % 初始加速度

%% 目标函数
H = zeros(6*N_DP);      % 初始化H矩阵
f = zeros(6*N_DP,1);    % 初始化f矩阵
t = deltT_DP;           % 每一段5次曲线的终点时刻 
for i = 1:N_DP
    % 加速度指标
    H1 = 2 * [0       0         0         0            0             0
        0       0         0         0            0             0
        0       0        4*t       6*t^2        8*t^3        10*t^4
        0       0       6*t^2      12*t^3      18*t^4        24*t^5
        0       0       8*t^3      18*t^4    (144/5)*t^5     40*t^6
        0       0      10*t^4      24*t^5      40*t^6     (400/7)*t^6
        ];
    
    % 加加速度指标
    H2 = 2 * [0       0         0         0            0             0
        0       0         0         0            0             0
        0       0         0         0            0             0
        0       0         0       36*t         72*t^2        120*t^3
        0       0         0      72*t^2       192*t^3        360*t^4
        0       0         0     120*t^3       360*t^4        720*t^5
        ];
    
    % 与动态规划得到的最优ST曲线的误差指标
    H3 = 2 * [1       t       t^2       t^3         t^4           t^5
        t       t^2      t^3       t^4          t^5           t^6
        t^2     t^3      t^4       t^5          t^6           t^7
        t^3     t^4      t^5       t^6          t^7           t^8
        t^4     t^5      t^6       t^7          t^8           t^9
        t^5     t^6      t^7       t^8          t^9           t^10];
    
    % 将上述计算的各性能指标按照权重相加，构造H矩阵
    idx1 = (i-1)*6+1;
    idx2 = i*6;
    tempH = w1 * H1 + w2 * H2 + w3 * H3;
    H(idx1:idx2, idx1:idx2) = H(idx1:idx2, idx1:idx2) + tempH;
    
    % 由于“与期望的S误差”会产生f项，在此添加
    refS = ST_opt(i+1,2);
    tempF = -2 * refS * [1       t       t^2       t^3         t^4           t^5]' *w3;
    f(idx1:idx2) = f(idx1:idx2) + tempF;
    
end

%% 等式约束
% 每一段的邻接连接点约束：位置、速度、加速度、加加速度约束
for i = 1:N_DP-1
    t = deltT_DP;
    % 位置
    aeq1 = [1  t   t^2       t^3       t^4       t^5      -1    0    0   0    0   0];
    % 速度
    aeq2 = [0  1   2*t      3*t^2     4*t^3     5*t^4      0   -1    0   0    0   0];
    % 加速度
    aeq3 = [0  0    2        6*t      12*t^2    20*t^3     0    0   -2   0    0   0];
    % 加加速度
    aeq4 = [0  0    0         6        24*t     60*t^2     0    0   0   -6    0   0];
    
    % b向量
    beq = [0 0 0 0]';
    
    % 将此段约束量添加到大矩阵中
    startRow = (i-1)*4+1;
    endRow = i*4;
    startCol = (i-1)*6+1;
    endCol = (i+1)*6;
    Aeq1(startRow:endRow, startCol:endCol) = [aeq1; aeq2; aeq3; aeq4];
    Beq1(startRow:endRow,1) = beq;
end

% ST曲线起点:位置、速度、加速度约束
Aeq2 = zeros(3,6*N_DP);
Beq2 = zeros(3,1);
t = 0;
aeq1 = [1  t   t^2       t^3       t^4       t^5];
aeq2 = [0  1   2*t      3*t^2     4*t^3     5*t^4];
aeq3 = [0  0    2        6*t      12*t^2    20*t^3];
beq = [ST_opt(1,2), v_ini,  a_ini]';
Aeq2(1:3, 1:6) = [aeq1; aeq2; aeq3];
Beq2(1:3) = beq;

% 组建等式约束矩阵
Aeq = [Aeq1; Aeq2];
Beq = [Beq1; Beq2];

%% 线性不等式约束
% ST图的障碍物上下边界约束
for i = 1:N_DP
    for j = 1:N_QP
        t = j*0.1;
        
        % 上下边界
        lb = 0;    % 注意：此处与速度、加速度的上下边界刚好相反
        ub = S_lb((i-1)*N_QP+j, 2)+2;
        
        % 构造Ax ≤b
        a1 = [-1      -t       -t^2      -t^3      -t^4      -t^5];
        a2 = [1       t        t^2       t^3       t^4       t^5];
        b = [-lb ub]';
        
        % 行列索引
        startRow = (i-1)*2*N_QP + (j-1)*2+1;
        endRow = (i-1)*2*N_QP + j*2;
        startCol = (i - 1) * 6 + 1;
        endCol = i * 6;
        
        % 添加到A、B大矩阵中
        A1(startRow:endRow, startCol:endCol) = [a1; a2];
        B1(startRow:endRow,1) = b;
    end
end


% 速度约束
for i = 1:N_DP
    for j = 1:N_QP
        t = j*0.1;
        
        % 上下边界
        lb = v_min;
        ub = v_max;
        
        % 构造Ax ≤b
        a1 = [ 0       -1        -2*t      -3*t^2     -4*t^3     -5*t^4];
        a2 = [0       1        2*t      3*t^2     4*t^3     5*t^4];
        b = [-lb ub]';
        
        % 行列索引
        startRow = (i-1)*2*N_QP + (j-1)*2+1;
        endRow = (i-1)*2*N_QP + j*2;
        startCol = (i - 1) * 6 + 1;
        endCol = i * 6;
        
        % 添加到A、B大矩阵中
        A2(startRow:endRow, startCol:endCol) = [a1; a2];
        B2(startRow:endRow,1) = b;
    end
end

% 加速度约束
for i = 1:N_DP
    for j = 1:N_QP
        t = j*0.1;
        
        % 上下边界
        lb = acc_min;
        ub = acc_max;
        
        % 构造Ax ≤b
        a1 = [ 0       0       -2        -6*t       -12*t^2     -20*t^3];
        a2 = [0       0        2         6*t        12*t^2      20*t^3];
        b = [-lb ub]';
        
        % 行列索引
        startRow = (i-1)*2*N_QP + (j-1)*2+1;
        endRow = (i-1)*2*N_QP + j*2;
        startCol = (i - 1) * 6 + 1;
        endCol = i * 6;
        
        % 添加到A、B大矩阵中
        A3(startRow:endRow, startCol:endCol) = [a1; a2];
        B3(startRow:endRow,1) = b;
    end
end

% 组建不等式约束矩阵
A = [A1; A2; A3];
B = [B1; B2; B3];

%  调用QP算法
options = optimoptions('quadprog','TolFun',1e-16);
x = quadprog(H,f,A,B,Aeq,Beq,[],[],[],options);

%% 将得到的解x重新调整为N_DP段
coeff = reshape(x,6,N_DP)';
for i = 1:N_DP
    if i == 1
        S_QP(1,1) = 0;
        S_QP(1,2) = ST_opt(1,2);
    end
    for j = 1:N_QP
        idx = (i-1)*N_QP + j+1;
        t = j*deltT_QP;
        T = (i-1)*deltT_DP + j*deltT_QP;
        % 位置
        S_QP(idx,1) = T;
        S_QP(idx,2) = coeff(i,1) + coeff(i,2)*t + coeff(i,3)*t^2 ...
            + coeff(i,4) * t^3 + coeff(i,5) * t^4 +coeff(i,6) * t^5;
        
        % 速度
        v_QP(idx,1) = T;
        v_QP(idx,2) = coeff(i,2) + 2*coeff(i,3)*t + 3*coeff(i,4) * t^2 + ...
            4*coeff(i,5) * t^3 + 5*coeff(i,6) * t^4;
        
        % 加速度
        a_QP(idx,1) = T;
        a_QP(idx,2) = 2*coeff(i,3) + 6*coeff(i,4) * t + ...
            12*coeff(i,5) * t^2 + 20*coeff(i,6) * t^3;
    end
end

%% 画图
figure
fill(obsZone(:,1), obsZone(:,2)+5, 'b')
hold on
plot(ST_opt(:,1),ST_opt(:,2),'g-*');    
plot(S_QP(:,1),S_QP(:,2),'r.');

%% 保存
save v_QP.mat v_QP
