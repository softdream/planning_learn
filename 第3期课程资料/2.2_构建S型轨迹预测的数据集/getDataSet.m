clc
clear
close all

%% 初始参数定义
path = cell(40,1);    % 数据集一共40条路径
obsSpeed = 10;        % 交通车速度
for i = 1:40
    % 创建场景
    scenario = drivingScenario;
    scenario.SampleTime = 0.1;
    
    % 道路中心坐标：根据pos的变化，生成不同曲率半径的S型道路
    pos = i+19;  
    roadCenters = [-pos pos 0; -pos -pos 0; 0 0 0; pos pos 0; pos -pos 0];
  
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
    
    % 根据单向两车道约定，计算左车道中心线坐标，以生成2号交通车路径点
    ptNums = size(rdbdy,1);
    waypoints = [];
    for j = 1:ptNums/2
        leftPt = rdbdy(j,:);
        rightPt = rdbdy(ptNums-j+1,:);
        waypoints(j,:) = leftPt + (rightPt - leftPt)/4;
    end
    
    % 生成车辆行驶轨迹
    iniPos = waypoints(1,:);
    myVehicle = vehicle(scenario,'Position',iniPos);
    trajectory(myVehicle, waypoints, obsSpeed);
    
    while advance(scenario)
        currentPos = scenario.Actors.Position;
        path{i}(:,end+1) = currentPos(1:2)';
    end

    % 清空变量
    clear scenario
end

%% 标准化处理
% 混合成两行数据，用于标准化（才能获取数据集最大值和最小值）
mixData = [];
for i = 1:length(path)
    mixData = [mixData, path{i}];
end

% 统一进行标准化为具有零均值和单位方差的数据集
for i = 1:2  
    mu(i) = mean(mixData(i,:));
    sig(i) = std(mixData(i,:));
    mixData_std(i,:) = (mixData(i,:) - mu(i)) / sig(i);
end

% 再把标准化之后的数据归类到原来的元胞里面
path_std = cell(0);
flag = 1;
for i = 1:length(path)
    len = length(path{i});
    path_std{i,1} = mixData_std(:,flag:flag+len-1);
    flag = flag + len;
end

%% 保存
save dataSet.mat path path_std mu sig