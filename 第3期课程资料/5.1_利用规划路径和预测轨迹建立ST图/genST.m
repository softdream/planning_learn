clc
clear
close all
load('scenario.mat')
load('optPath.mat')

%% 参数初始化
% 车辆参数
vehLength = 5;
vehWidth = 1.8;
vehHalfWidth = vehWidth/2;

% 获取障碍车轨迹
startIdx = 135;
endIdx = 210;
obsPath = [];
for i = startIdx:endIdx
    obsPath(end+1,:) = tjty(i).pos;
end

% 画场景图
plot(scenario)
hold on
plot(path_opt(:,1),path_opt(:,2),'r')
scatter(obsPath(:,1),obsPath(:,2),'b.');

% 计算障碍车每个轨迹点的航向角
diffX = diff(obsPath(:,1));
diffY = diff(obsPath(:,2));
ptNum = size(obsPath,1);
heading =  atan2(diffY, diffX);
heading(end+1) = heading(end);

%% 将障碍车轨迹映射到ST图
S_lb = [];
S_ub = [];
for i = 1:ptNum-1
    t = i*0.1;

    % 车尾中心点位置和车辆航向角
    rearPosX = obsPath(i,1);
    rearPosY = obsPath(i,2);
    
    % 计算航向角
    diffX = obsPath(i+1,1) - obsPath(i,1);
    diffY =  obsPath(i+1,2) - obsPath(i,2);
    heading = atan2(diffY, diffX);
    
    % 根据车尾中心点位置和航向角计算车头中心点位置
    frontPosX = rearPosX + vehLength * cos(heading);
    frontPosY = rearPosY + vehLength * sin(heading);
    
    % 根据前后中心点、航向角计算目障碍车四个轮廓点位置（顺时针编号）
    % 每一行代表一个车辆一个角的纵向和横向位置
    VehPoints(1,1) = rearPosX - vehHalfWidth * sin(heading);
    VehPoints(1,2) = rearPosY + vehHalfWidth * cos(heading);
    VehPoints(2,1) = frontPosX - vehHalfWidth * sin(heading);
    VehPoints(2,2) = frontPosY + vehHalfWidth * cos(heading);
    VehPoints(3,1) = frontPosX + vehHalfWidth * sin(heading);
    VehPoints(3,2) = frontPosY - vehHalfWidth * cos(heading);
    VehPoints(4,1) = rearPosX + vehHalfWidth * sin(heading);
    VehPoints(4,2) = rearPosY - vehHalfWidth * cos(heading);
    
    % 依次计算四个点位置距离自车轨迹序列的最小距离的索引
    for j = 1:4
        distX = path_opt(:,1) - VehPoints(j,1);
        distY = path_opt(:,2) - VehPoints(j,2);
        dist = sqrt((distX.^2 + distY.^2));
        [minDistTemp(j), minIndexTemp(j)] =  min(dist);
    end
    
    % 然后在四个距离中再次选择最小值，表征障碍车辆轮廓与本车轨迹的最小距离
    minDist = min(minDistTemp);

    % 若最小距离值小于阈值，表明障碍车在本时刻将会占据本车轨迹
    if minDist < 1
        % 计算minIndexTemp的最小索引和最大索引
        minIdx = min(minIndexTemp);
        maxIdx = max(minIndexTemp);
        
        % 然后获得S的下边界和上边界
        S_lb(end+1,:) = [t, cumLength_opt(minIdx)];
        S_ub(end+1,:) = [t, cumLength_opt(maxIdx)];
    end
end

%% 画图
obsZone = [S_lb; S_ub(end:-1:1,:); S_lb(1,:)];
figure
fill(obsZone(:,1), obsZone(:,2), 'b')
grid on
xlim([0,8]);
ylim([0,60]);
xlabel('时间/s')
ylabel('距离/m')

%% 保存
save stGragh.mat S_lb S_ub obsZone totalLength_opt