clc
clear
close all

%% 构建道路场景
% 场景初始化
scenario = drivingScenario;   % 初始化场景
scenario.SampleTime = 0.1;    % 采样时间

% 道路中心坐标    
roadCenters = [-50 50 0; -50 -50 0; 0 0 0; 50 50 0; 50 -50 0];  

% 建立道路
marking = [laneMarking('Solid') ...
    laneMarking('Dashed') laneMarking('Solid')];             % 分界线线型
laneSpecification = lanespec(2, 'Marking', marking);         % 道路规范
road(scenario, roadCenters, 'Lanes', laneSpecification);     % 生成道路

% 获得道路边界,先把边界点位置重新整理次序
rdbdy = roadBoundaries(scenario);
rdbdy = rdbdy{1,1}(1:end-1,:);
rdbdy = [rdbdy(3:end,:); rdbdy(1:2,:)];
rdbdy(1,3) = 0;

%% 构建车辆信息
% 根据单向两车道约定，分别计算左车道和右车道中心线坐标，以生成车辆航迹点
ptNums = size(rdbdy,1);
for j = 1:ptNums/2
    rightBdyPt = rdbdy(j,:);
    leftBdyPt = rdbdy(ptNums-j+1,:);
    leftWaypoints(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*0.25;
    rightWaypoints(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*0.75;
end

% 本车信息
ego.vehicle = [];
ego.waypoints = rightWaypoints;
ego.pos = ego.waypoints(1,:);
ego.speed = 15;

% 1号交通车信息
obs = struct;
obs(1).vehicle = [];
obs(1).waypoints = rightWaypoints(50:end,:);
obs(1).pos = obs(1).waypoints(1,:);
obs(1).speed = 10;

% 2号交通车信息
obs(2).vehicle = [];
obs(2).waypoints = leftWaypoints(30:end,:);
obs(2).pos = obs(2).waypoints(1,:);
obs(2).speed = 8;

%% 在场景增加车辆
% 根据自车信息增加车辆
ego.vehicle = vehicle(scenario,'Position',ego.pos,'PlotColor','b');
trajectory(ego.vehicle, ego.waypoints,ego.speed);

% 根据交通车信息增加车辆
for i = 1:2
    obs(i).vehicle = vehicle(scenario,'Position',obs(i).pos,'PlotColor','k');
    trajectory(obs(i).vehicle, obs(i).waypoints,obs(i).speed);
end

%% 启动仿真
plot(scenario)
chasePlot(ego.vehicle)
tjty = [];
step = 1;
MakeGif('仿真动图.gif',step);
while advance(scenario)
    pause(0.05)
    tjty(step).time = scenario.SimulationTime;
    tjty(step).pos = scenario.Actors(1, 3).Position(1:2);
    step = step + 1;
    MakeGif('仿真动图.gif',step)
end

%% 保存
save Scenario.mat scenario leftWaypoints rightWaypoints tjty