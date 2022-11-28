clc
clear
close all
load  refPath.mat

%% 相关参数定义
refPos = refPath_8;
targetSpeed = 10;      % m/s
Kv = 0.1;              % 前视距离系数
Kp = 0.8;              % 速度P控制器系数
Ld0 = 2;               % Ld0是预瞄距离的下限值
dt = 0.1;              % 时间间隔，单位：s
L = 2.9;               % 车辆轴距，单位：m

% 计算纵向累计距离和参考航向角
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % 航向角
refHeading(end+1) = refHeading(end);

% 车辆初始状态定义
iniPos = refPos(1,:);
iniSpd = 0;
iniHeading = refHeading(1);

%% 主程序
% 当前车辆信息
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;
 
% 将初始状态纳入实际状态数组中
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_PP = [];

% 循环遍历轨迹点
while true
    % 寻找预瞄距离范围内最近路径点
    targetIdx = findLookaheadPoint(currentPos, currentSpd, refPos, Kv, Ld0);
    
    % 判断是否超出索引
    if targetIdx >= size(refPos,1)
        break
    end
   
    % 计算控制量
    [delta,latErr]  = PP_controller(targetIdx,currentPos, currentHeading, currentSpd, refPos,refHeading, Kv, Ld0,L);
    
    % 如果误差过大，退出循迹
    if abs(latErr) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度
    currentAcc = Kp* (targetSpeed-currentSpd)/dt;
    
    % 更新状态量
    [currentPos, currentHeading, currentSpd] = updateState(currentAcc,currentPos, currentHeading, currentSpd,delta,L, dt);
    
    % 保存每一步的实际量
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_PP(end+1,:) = [cumLength(targetIdx),latErr];    
end

% 画图
figure
plot(refPos(:,1), refPos(:,2), 'b');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
hold on 
for i = 1:size(posSet,1)
    scatter(posSet(i,1), posSet(i,2),150, '.r');
    pause(0.01)
end
legend('规划车辆轨迹', '实际行驶轨迹')

% 保存
path_PP = posSet;
save PP.mat path_PP latErr_PP

%% 首先在参考轨迹上搜索离当前车辆位置最近的点
function  targetIdx = findLookaheadPoint(currentPos, currentSpd, refPos, Kv, Ld0)
% 找到距离当前位置最近的一个参考轨迹点的序号
deltX = refPos(:,1)-currentPos(1);
deltY = refPos(:,2)-currentPos(2);
dist = sqrt(deltX.^2 + deltY.^2);
[~,idx] = min(dist); 
Ld = Kv*currentSpd + Ld0;       % Ld0是预瞄距离的下限值；


% 从该点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
L_steps = 0;           % 参考轨迹上几个相邻点的累计距离
while L_steps < Ld && idx < size(refPos,1)
    L_steps = L_steps + norm(refPos(idx + 1,:) - refPos(idx,:));
    idx = idx+1;
end
targetIdx = idx;
end


%% 获得控制量：前轮转向
function [delta,latError] = PP_controller(targetIdx,currentPos, currentHeading, currentSpd,refPos,refHeading, Kv, Ld0, L)
lookaheadPoint = refPos(targetIdx,:);
alpha = atan2(lookaheadPoint(2) - currentPos(2), lookaheadPoint(1) - currentPos(1))  - currentHeading;
Ld = Kv*currentSpd + Ld0;

% 求位置、航向角的误差
x_error  = currentPos(1) - refPos(targetIdx,1);
y_error = currentPos(2) - refPos(targetIdx,2);
heading_r = refHeading(targetIdx);

% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% 前轮转角
delta = atan2(2*L*sin(alpha), Ld);

end

%% 更新状态量
function [pos_new, heading_new, v_new] = updateState(acc_old,pos_old, heading_old, v_old,delta,wheelbase, dt)
v_new =  v_old + acc_old*dt;
heading_new=  heading_old + v_new*dt*tan(delta)/wheelbase;
pos_new(1) = pos_old(1) + v_new*cos(heading_new)*dt;
pos_new(2) =  pos_old(2) + v_new*sin(heading_new)*dt;
end

