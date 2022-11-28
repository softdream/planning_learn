clc
clear
close all
load  refPath.mat

%% 初始参数
dt = 0.1;   % 时间步长
L = 2.9;    % 轴距

%% 参考轨迹的相关参数
% 定义参考轨迹
refPos = refPath_sin;
refSpeed = 10;    % 参考速度

% 计算航向角
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % 航向角
refHeading(end+1) = refHeading(end);

% 曲率
for i = 1:size(refPos,1)-2
    x = refPos(i:i+2,1);
    y = refPos(i:i+2,2);
    refCur(i) = getCur(x,y);
end
refCur(i+1) = refCur(i);
refCur(i+2) = refCur(i);

% 根据阿克曼转向原理，计算参考前轮转角
refDelta = atan(L*refCur);

% 车辆初始状态定义
iniPos = refPos(1,:);
iniSpd = 0;
iniHeading = refHeading(1);
%% 主程序
% 当前车辆信息
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;

% 赋初值
Delta = 0;
U = [0.01;0.01];

% 轨迹跟踪实际量
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_MPC = [];

% 循迹
while true
    % 寻找预瞄距离范围内最近路径点
    dist = sqrt((refPos(:,1)-currentPos(1)).^2 + (refPos(:,2)-currentPos(2)).^2);
    [~,targetIdx] = min(dist); 
    
    % 调用MPC控制器
    [Delta,currentSpd,idx,latErr,U] = MPC_controller(targetIdx,currentPos,currentHeading,refPos,refHeading,refDelta,dt,L,U,refSpeed);
    
    % 判断是否超出索引
    if targetIdx >= size(refPos,1)-2
        break
    end
    
    
    % 误差太大，退出程序
    if abs(latErr) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 更新状态量
    [currentPos,currentHeading] = updateState(currentPos,currentHeading, currentSpd, Delta, dt,L); 
    
    % 保存每一步的实际量
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_MPC(end+1,:) = [cumLength(targetIdx),latErr];
    
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
path_MPC = posSet;
save MPC.mat path_MPC latErr_MPC

%%
function [currentPos,currentHeading] = updateState(currentPos,currentHeading, currentSpd, Delta, dt,L)
max_steer = 60 * pi/180; 
Delta = max(min(max_steer, Delta), -max_steer);
currentPos(1) = currentPos(1) + currentSpd * cos(currentHeading) * dt;
currentPos(2) = currentPos(2) + currentSpd * sin(currentHeading) * dt;
currentHeading = currentHeading + currentSpd / L * tan(Delta) * dt ;
end