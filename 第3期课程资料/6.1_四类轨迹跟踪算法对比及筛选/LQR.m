clc
clear
close all
load  refPath.mat

%% 相关参数定义
dt = 0.1;
L = 2.9 ;
Q = 10*eye(3);
R = eye(2)* 2;

%% 轨迹处理
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
    refCur(i) = abs(getCur(x,y));
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

% 轨迹跟踪实际量
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_LQR = [];

% 循环
while true
     % 寻找预瞄距离范围内最近路径点
    dist = sqrt((refPos(:,1)-currentPos(1)).^2 + (refPos(:,2)-currentPos(2)).^2);
    [~,targetIdx] = min(dist); 
    
    % 判断是否超出索引
    if targetIdx >= size(refPos,1)-2
        break
    end
    
    % LQR控制器
    [v_delta,delta,delta_r,latErr] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refCur,L,Q,R,dt);    
    
    % 如果误差过大，退出循迹
    if abs(latErr) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 更新状态
    [currentPos,currentHeading,currentSpd,Delta] = update(currentPos,currentHeading,currentSpd, v_delta,delta, dt,L, refSpeed,delta_r);
    
    % 保存每一步的实际量
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_LQR(end+1,:) = [cumLength(targetIdx),latErr];
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
path_LQR = posSet;
save LQR.mat path_LQR latErr_LQR


%% LQR控制
function [v_delta,Delta_delta,delta_r,latError] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refCur,L,Q,R,dt)
% 求位置、航向角参考量
x_r = refPos(targetIdx,1);
y_r = refPos(targetIdx,2);
heading_r = refHeading(targetIdx);
delta_r = refDelta(targetIdx);

% 求位置、航向角的误差
x_error  = currentPos(1) - x_r;
y_error = currentPos(2) - y_r;
yaw_error =  currentHeading - heading_r;

% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% 将误差值赋值到状态量
X(1,1) = x_error; 
X(2,1) = y_error;  
X(3,1) = yaw_error;

% 由状态方程矩阵系数，计算K
A = [1,  0,  -currentSpd*dt*sin(heading_r);
     0,  1,  currentSpd * dt * cos(heading_r);
     0,  0,  1];
B = [dt * cos(heading_r),    0;
     dt * sin(heading_r),    0;
     dt * tan(heading_r)/L,  currentSpd*dt/(L * cos(delta_r)^2)];


K = calcu_K(A,B,Q,R);

% 获得速度误差量、前轮转角误差量两个控制量
u = -K * X;  % 2行1列
v_delta = u(1);      
Delta_delta = u(2);

end


%% 计算增益
function K = calcu_K (A,B,Q,R)

% 终止条件定义
iter_max = 500;
epsilon = 0.01;

% 循环
P_old = Q;
for i = 1:iter_max
    P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q;
    if abs(P_new - P_old) <= epsilon
        break
    else
        P_old = P_new; 
    end
end

P = P_new;
K = (B' * P * B + R) \ (B' * P * A);  % 2行3列
end

%% 更新状态
function [pos_new, currentHeading, currentSpd, Delta] = update(pos_old, currentHeading, currentSpd, currentSpd_delta,Delta_delta,dt,L,refSpeed,refDelta)
Delta = refDelta + Delta_delta;
currentHeading = currentHeading + currentSpd / L * tan(Delta) * dt;
currentSpd = refSpeed + currentSpd_delta;
pos_new(1) = pos_old(1) + currentSpd*cos(currentHeading)*dt;
pos_new(2) =  pos_old(2) + currentSpd*sin(currentHeading)*dt;
end
