function [RMSE_x, RMSE_y] = preTraj(net, input_test, target_test,sig ,mu)

% 预测时域长度及起始预测帧等定义
predLen = 10;  
frameFlag = 50;                                            % 从frameFlag开始向后预测
endFlag = mod(size(input_test,2)-frameFlag, predLen);      % 用于去掉余数那一小截数据
steps = floor( (size(input_test,2)-frameFlag) / predLen ); % 滚动预测轨迹的总次数

%% 预测轨迹
% 根据起始预测帧初始化预测变量
YPred =  input_test(:,1:frameFlag);
output_test = [];
% 循环滚动预测
for i = 1:steps    
    for j = 1:predLen       
        [net,YPred_temp] = predictAndUpdateState(net,YPred);
        YPred = [YPred,YPred_temp(:,end)];
    end
    
    % 追加到输出量
    output_test = [output_test,YPred(:,frameFlag+(i-1)*predLen+1:frameFlag+i*predLen)];

    %一个预测时域结束后，将本时域新增的实际轨迹用来下一次更新网络
    YPred = input_test(:,1:frameFlag+i*predLen);
end   

% 根据标准化的sig和mu，获得反标准化的输出值
for i = 1:2 
    output_test(i,:) = sig(i)*output_test(i,:) + mu(i);
    target_test(i,:) = sig(i)*target_test(i,:) + mu(i);
end

% 预测位置和实际位置
actualPos = target_test(:,frameFlag+1:end - endFlag);
predictedPos = output_test;

%% 计算误差
RMSE_x = sqrt(mean((actualPos(1,:) - predictedPos(1,:)).^2));
RMSE_y = sqrt(mean((actualPos(2,:) - predictedPos(2,:)).^2 ));
