clc
clear
close all
load dataSet.mat
load obsPre.mat
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
obs(1).waypoints = rightWaypoints(40:end,:);
obs(1).pos = obs(1).waypoints(1,:);
obs(1).speed = 10;

% 2号交通车信息
obs(2).vehicle = [];
obs(2).waypoints = leftWaypoints(25:end,:);
obs(2).pos = obs(2).waypoints(1,:);
obs(2).speed = 8;

%% 在场景增加车辆
% 根据交通车信息增加车辆
for i = 1:2
    obs(i).vehicle = vehicle(scenario,'Position',obs(i).pos,'PlotColor','k');
    trajectory(obs(i).vehicle, obs(i).waypoints,obs(i).speed);
end
% 根据自车信息增加车辆
ego.vehicle = vehicle(scenario,'Position',ego.pos,'PlotColor','b');
trajectory(ego.vehicle, ego.waypoints,ego.speed);

%% 启动仿真
tjty = [];
step = 1;
time = [];
while advance(scenario)
    refInfo_obs1(step,1:2) = scenario.Actors(1,1).Position(1:2);
    refInfo_obs1(step,3) = scenario.Actors(1,1).Yaw;
    refInfo_obs1(step,4:5) = scenario.Actors(1,1).Velocity(1:2);
    refInfo_obs2(step,1:2) = scenario.Actors(1,2).Position(1:2);
    refInfo_obs2(step,3) = scenario.Actors(1,2).Yaw;
    refInfo_obs2(step,4:5) = scenario.Actors(1,2).Velocity(1:2);
    
    egoInfo(step,1:2) = scenario.Actors(1,3).Position(1:2);
    egoInfo(step,3) = scenario.Actors(1,3).Yaw;
    egoInfo(step,4:5) = scenario.Actors(1,3).Velocity(1:2);
    time(end+1) = scenario.SimulationTime;
    step = step + 1;
end

%% Simulink的初始常参数
egoTargetSpd_kph = 60;
iniEgoPos = egoInfo(1,1:2);
iniEgoHeading = egoInfo(1,3);
iniEgoSpd = egoInfo(1,4:5);

iniObsPos1 = refInfo_obs1(1,1:2);
iniObsHeading1 = refInfo_obs1(1,3);
iniObsSpd1 = refInfo_obs1(1,4:5);

iniObsPos2 = refInfo_obs2(1,1:2);
iniObsHeading2 = refInfo_obs2(1,3);
iniObsSpd2 = refInfo_obs2(1,4:5);


%% 转为时序变量
egoPos = timeseries(egoInfo(:,1:2),time);
egoSpd = timeseries(egoInfo(:,4:5),time);
obsPos1 = timeseries(refInfo_obs1(:,1:2),time);
obsSpd1 = timeseries(refInfo_obs1(:,4:5),time);
