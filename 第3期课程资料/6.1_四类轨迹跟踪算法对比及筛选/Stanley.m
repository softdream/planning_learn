clc
clear
close all
load  refPath.mat

%% 相关参数定义
% 参考轨迹
refPos = refPath_sin;            

% 计算轨迹的参考航向角
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % 航向角
refHeading(end+1) = refHeading(end);

% 其他常量参数
targetSpeed = 10;           % 目标速度，单位： m /s
k = 10;                      % 增益参数
Kp = 1;                     % 速度P控制器系数
dt = 0.1;                   % 时间间隔，单位：s
L = 2;                      % 车辆轴距，单位：m

% 车辆初始状态定义
iniPos = refPos(1,:);
iniSpd = 0;
iniHeading = refHeading(1);
%% 主程序
% 车辆初始状态定义
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;

% 将初始状态纳入实际状态数组中
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_Stanley = [];

% 循迹
while true
    % 寻找预瞄距离范围内最近路径点
    dist = sqrt((refPos(:,1)-currentPos(1)).^2 + (refPos(:,2)-currentPos(2)).^2);
    [~,targetIdx] = min(dist);
    
    % 判断是否超出索引
    if targetIdx >= size(refPos,1)-1
        break
    end
       
    % 计算前轮转角
    [delta,latErr] = stanleyControler(targetIdx,currentPos, currentHeading, currentSpd,refPos,refHeading,k);
    
    % 如果误差过大，退出循迹
    if abs(latErr) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度
    currentAcc = Kp* (targetSpeed-currentSpd);
    
    % 更新状态量
    [currentPos, currentHeading, currentSpd] = updateState(currentAcc,currentPos, currentHeading, currentSpd,delta,L, dt);
    
    % 保存每一步的实际量
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_Stanley(end+1,:) = [cumLength(targetIdx),latErr];
    
    
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
path_Stanley = posSet;
save stanley.mat path_Stanley latErr_Stanley
%% 首先在参考轨迹上搜索离当前位置最近的点
function target_idx = findTargetIdx(state,refPos)
for i = 1:size(refPos,1)
    d(i,1) = norm(refPos(i,:) - state(1:2));
end
[~,target_idx] = min(d);  % 找到距离当前位置最近的一个参考轨迹点的序号
end

%% 获得控制量
function [delta,latError] = stanleyControler(targetIdx,currentPos, currentHeading, currentSpd,refPos,refHeading,k);
% 根据百度Apolo，计算横向误差
dx = currentPos(1) - refPos(targetIdx,1);
dy = currentPos(2) - refPos(targetIdx,2);
phi_r = refHeading(targetIdx);
latError = dy*cos(phi_r) - dx*sin(phi_r);

% 分别计算只考虑航向误差的theta和只考虑横向误差的theta
theta_fai =  refHeading(targetIdx)- currentHeading;
theta_y = atan2(-k*latError,currentSpd);

% 将两个角度合并即为前轮转角
delta = theta_fai + theta_y;
end

%% 更新状态量
function [pos_new, heading_new, v_new] = updateState(acc_old,pos_old, heading_old, v_old,delta,wheelbase, dt)
v_new =  v_old + acc_old*dt;
heading_new=  heading_old + v_new*dt*tan(delta)/wheelbase;
pos_new(1) = pos_old(1) + v_new*cos(heading_new)*dt;
pos_new(2) =  pos_old(2) + v_new*sin(heading_new)*dt;
end
