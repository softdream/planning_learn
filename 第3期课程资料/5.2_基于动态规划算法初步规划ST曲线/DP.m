clc
clear
close all
load stGragh.mat

%% 初始化
% 约束条件
v_ini = 12;                % 起点速度
a_ini = 0;                 % 起点加速度
a_end = 0;                 % 终点加速度
v_max = 16;                % 最大速度
a_max = 3;                 % 最大加速度
omega = [0.4, 0.6];        % 加速度和加加速度权重

% 定义状态空间，用于DP搜索
T = 8;                     % ST图总时间
dt = 1;                    % 时间离散步长
dS = 0.2;                  % 距离离散步长
N = round(T / dt);         % 一共N个阶段

% 调用函数，将每一个时刻的障碍物上下限补全
[S_lb, S_ub] = lbAndUb(T,S_lb,S_ub);

% 构造表征s/t的状态空间
t_seq = 0:dt:T;                                   % 时间离散向量
s_seq = 0:dS:round(totalLength_opt,1);            % 距离离散向量
stateSpace = cell(length(s_seq),length(t_seq));
for i = 1:length(s_seq)
    for j = 1:length(t_seq)
        row = length(s_seq)-i+1;
        col = j;
        stateSpace{row,col} = [t_seq(j),s_seq(i)];
    end
end

%% 第1阶段（时刻0）的初步计算
% 遍历第2阶段，初步生成从第一阶段的起点到第二阶段的所有可能状态
state = [];
for i = size(stateSpace,1):-1:1
    % 计算位置、速度、加速度、加加速度
    s = stateSpace{i,2}(2);
    v = s / dt;
    a = (v - v_ini) / dt;
    jerk = (a - a_ini) / dt;
    
    % 若满足速度、加速度约束，则将此状态追加到state中
    if v <= v_max && abs(a) <= a_max
        % 计算加速度和加加速度代价
        accCost = abs(a);
        jerkCost = abs(jerk);
        cost =  accCost*omega(1) + 2*jerkCost*omega(2);
        
        % 将第1阶段行索引、第2阶段行索引、位置、速度、加速度、加加速度、代价值存放在state中
        state(end+1,1:6) = [size(stateSpace,1),i,s,v,a,cost];
    end
end

% 初始化最优序列
optSeq = cell(0);     % 每一阶段至第1阶段的累计最优序列
for i = 1:size(state,1)
    optSeq{i,1} = state(i,:);
end

%% 从第2阶段（时刻dt）逐步遍历寻优
lastState = state;          % 将上面第1阶段获得的备选可行状态赋值给lastState

% 第1层循环：第i个阶段
for i = 2:N+1
    % 时间和下一可行状态初始化
    t = round((i-1)*dt,2);
    nextStateTemp = [];
    
    % 第2层循环：当前第i阶段的第j个可能状态
    for j = 1:size(lastState,1)
        state_j = [];
        lastStepIdx = lastState(j,2);
        
        %  第3层循环：第i+1阶段的第k个可能状态
        for k = lastStepIdx:-1:1
            % 计算位置、速度、加速度、加加速度
            s = stateSpace{k,(i-1)}(2);
            v = (s - lastState(j,3)) / dt;
            a = (v -  lastState(j,4)) / dt;
            jerk = (a -  lastState(j,5)) / dt;
            
            % 若满足速度、加速度、障碍物约束，则将此状态追加到state_j中
            idx = t*10+1;
            if v <= v_max && abs(a) <= a_max && ...
                    (s < S_lb(idx,2) || s > S_ub(idx,2))
                % 计算代价函数
                accCost = abs(a);
                jerkCost = abs(jerk);
                cost =  accCost*omega(1) + 2*jerkCost*omega(2);
                
                % 将第i阶段行索引、第i+1阶段行索引、位置、速度、加速度、加加速度、代价值存放在state中
                state_j(end+1,1:6) = [lastStepIdx,k,s,v,a,cost];
            end
        end
        nextStateTemp = [nextStateTemp; state_j];
    end
    
    % 故障判断：若nextStateTemp为空，表明下阶段已无任何可行状态，则执行下一个循环
    if ~isempty(nextStateTemp)
        % 先将下一阶段的可行状态排序，然后只取非重复行索引序号放进s_Temp
        [~,sortIdx] = sort(nextStateTemp(:,2));
        nextStateTemp = nextStateTemp(sortIdx,:);
        s_Temp = unique(nextStateTemp(:,2));
        
        % 由于从第i阶段的若干状态到第i+1阶段的某个状态可能有多个选择，那么应该择优处理
        nextState = [];
        for j = 1:length(s_Temp)
            s = s_Temp(j);                                % 第i+1阶段的s值
            idx = find(nextStateTemp(:,2) == s);          % 找出到底有多少个重复的s值
            [~,minIdx] = min(nextStateTemp(idx,6));       % 若有多个，选择从i阶段到i+1阶段的s值的代价最小的那一个          
            nextState(end+1,:) = nextStateTemp(idx(minIdx),:);
        end
        
        % 将本阶段的所有最优状态序列添加至opt_seq中
        for j = 1:size(nextState,1)
            lastStepIdx = nextState(j,1);   %来自于上一个阶段的哪一个状态
            for k = 1:size(optSeq,1)
                if optSeq{k,1}(end,2) == lastStepIdx
                    opt_seq_temp{j,1} = [optSeq{k,1};nextState(j,:)];
                    break
                end
            end
        end
        optSeq = opt_seq_temp;
        lastState = nextState;
    end
end

%% 找出最优ST曲线
t_opt= [];
s_opt=[];
for j = 1:size(optSeq{1,1},1)
    idx = optSeq{1,1}(j,1);
    t_opt(j) = stateSpace{idx,j}(1);
    s_opt(j) = stateSpace{idx,j}(2);
end
ST_opt = [t_opt', s_opt'];

%% 画图
figure
hold on
fill(obsZone(:,1), obsZone(:,2), 'b')
for i = 1:size(optSeq,1)
    t_seq= [];
    s_seq=[];
    for j = 1:size(optSeq{i,1},1)
        idx = optSeq{i,1}(j,1);
        t_seq(j) = stateSpace{idx,j}(1);
        s_seq(j) = stateSpace{idx,j}(2);
    end
    plot(t_seq, s_seq, 'c')
end
plot(ST_opt(:,1),ST_opt(:,2),'r-*','linewidth',2);
xlabel('时间/s')
ylabel('距离/m')
grid on
%% 保存
save ST_opt.mat ST_opt S_ub S_lb