clc
clear
close all
load  refTrajectory.mat

%% 初始参数
dt = 0.1;   % 时间步长
L = 2.9;    % 轴距

%% 参考轨迹的相关参数
% 定义参考轨迹
refPos = refPath;

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
iniSpd = refSpeed(1);
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
headingSet = [];
spdSet = [];
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
    headingErr = currentHeading-refHeading(targetIdx);
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = [cumLength(targetIdx),headingErr];
    spdSet(end+1,:) = [refSpeed(targetIdx), currentSpd];
    latErr_MPC(end+1,:) = [cumLength(targetIdx),latErr];
    
end

%% 画图
% 画轨迹跟踪图
figure
plot(refPos(:,1), refPos(:,2), 'b');
hold on 
scatter(posSet(:,1), posSet(:,2),150, '.r');
grid on
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');

% 画横向误差图
figure
plot(latErr_MPC(:,1),latErr_MPC(:,2),'r','linewidth',1.5);
grid on
xlabel('距离/m');
ylabel('横向误差/m');

% 画航向误差图
figure
plot(headingSet(:,1),headingSet(:,2),'r','linewidth',1.5);
grid on
xlabel('距离/m');
ylabel('航向误差/rad');

% 画速度误差图
figure
plot(spdSet(:,1),spdSet(:,2),'r','linewidth',1.5);
hold on
plot([3,12],[3,12],'b--','linewidth',1.5);
grid on
legend('参考速度-跟踪速度','y=x 参考线')
xlabel('参考速度 / m/s');
ylabel('实际速度 / m/s');


legend('规划车辆轨迹', '实际行驶轨迹')

%% 函数
function [currentPos,currentHeading] = updateState(currentPos,currentHeading, currentSpd, Delta, dt,L)
max_steer = 60 * pi/180; 
Delta = max(min(max_steer, Delta), -max_steer);
currentPos(1) = currentPos(1) + currentSpd * cos(currentHeading) * dt;
currentPos(2) = currentPos(2) + currentSpd * sin(currentHeading) * dt;
currentHeading = currentHeading + currentSpd / L * tan(Delta) * dt ;
end