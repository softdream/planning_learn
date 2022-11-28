% 利用LSTM网络预测
clc
clear
close all
load trainResult.mat     
load dataSet.mat     
load Scenario.mat
%% 参数定义
% 根据测试集的索引号构造输入量，且不取最后一个变量；初始化输出量
input = input_test{1,1}(:,1:end-1);
output = [];
target = target_test{1,1};

% 预测时域长度及起始预测帧等定义
predLen = 10;  
frameFlag = 50;                                       % 从frameFlag开始向后预测
endFlag = mod(size(input,2)-frameFlag, predLen);      % 用于去掉余数那一小截数据
steps = floor( (size(input,2)-frameFlag) / predLen ); % 滚动预测轨迹的总次数

%% 预测轨迹
% 根据起始预测帧初始化预测变量
YPred =  input(:,1:frameFlag);

% 循环滚动预测
for i = 1:steps    
    for j = 1:predLen       
        % 滚动预测
        [net,YPred_temp] = predictAndUpdateState(net,YPred);
        YPred = [YPred,YPred_temp(:,end)];
    end
    
    % 追加到输出量
    output = [output,YPred(:,frameFlag+(i-1)*predLen+1:frameFlag+i*predLen)];

    %一个预测时域结束后，将本时域新增的实际轨迹用来下一次更新网络
    YPred = input(:,1:frameFlag+i*predLen);
end   

% 根据标准化的sig和mu，获得反标准化的输出值
for i = 1:2 
    output(i,:) = sig(i)*output(i,:) + mu(i);
    target(i,:) = sig(i)*target(i,:) + mu(i);
end

% 赋值
actualPos = target(:,frameFlag+1 : end-endFlag-1);
predictedPos = output;

%% 计算误差
% 横纵向的绝对误差
error_x = abs(actualPos(1,:) - predictedPos(1,:));
error_y = abs(actualPos(2,:) - predictedPos(2,:));

% 横纵向的误差最大值
error_x_max = max(error_x);
error_y_max = max(error_y);

% 误差平均值
error_x_mean = mean(error_x);
error_y_mean = mean(error_y);

% 均方根误差的平均值
RMSE_x = sqrt(mean((actualPos(1,:) - predictedPos(1,:)).^2));
RMSE_y = sqrt(mean((actualPos(2,:) - predictedPos(2,:)).^2 ));

%% 画图
% X误差
figure
hold on
plot(1:length(error_x), error_x,'r');
title('X误差')
xlabel('序列索引')
ylabel('误差/m')

% Y误差
figure
hold on
plot(1:length(error_y), error_y,'r');
title('Y误差')
xlabel('序列索引')
ylabel('误差/m')

% 实际轨迹与预测轨迹对比
figure
hold on
plot(actualPos(1,:), actualPos(2,:),'r');
plot(predictedPos(1,:), predictedPos(2,:),'b--');
title('实际轨迹与预测轨迹对比')
legend('实际轨迹','预测轨迹')
xlabel('X坐标/m')
ylabel('Y坐标/m')
%% 保存实际轨迹和预测轨迹数据
save predictResult.mat actualPos predictedPos
