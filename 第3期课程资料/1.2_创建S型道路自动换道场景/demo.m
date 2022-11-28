clc
clear
close all

%% 构建道路场景
% 场景初始化
scenario = drivingScenario;   % 初始化场景
scenario.SampleTime = 0.1;    % 采样时间

% 道路中心坐标    
roadCenters = [0 0 0; 40 0 0];  

% 建立道路
marking = [laneMarking('Solid') ...
    laneMarking('Dashed') laneMarking('Solid')];             % 分界线线型
laneSpecification = lanespec(2, 'Marking', marking);         % 道路规范
road(scenario, roadCenters, 'Lanes', laneSpecification);     % 生成道路

% 增加车辆
vehicle(scenario,'Position',[2 0 0]);

% 获得道路边界,先把边界点位置重新整理次序
rdbdy = roadBoundaries(scenario);
rdbdy = rdbdy{1,1}(1:end-1,:);
rdbdy = [rdbdy(3:end,:); rdbdy(1:2,:)];
rdbdy(1,3) = 0;

% 画图
plot(scenario)
xlim([0,40])
ylim([-5,5])
