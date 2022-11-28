clc
clear
load dataSet.mat

%% 将40组轨迹，分成10份，每份4组数据
% 采用10折交叉验证，随机生成索引值序列
indices = crossvalind('Kfold', length(path_std), 10);

% 将40组数据按照indices分成10份，存放在元胞数组
path_std_crossvalid = cell(10,4);
for i = 1:length(indices)
    for j = 1:4
        row = indices(i);
        if isempty(path_std_crossvalid{row,j})
            path_std_crossvalid{row,j} = path_std{i,1};
            break
        end
    end
end

%% LSTM网络训练
parSet = {'sgdm','rmsprop','adam'};  % 超参数集
for i = 1:length(parSet)
    RMSE_x_all = zeros(10,4);
    RMSE_y_all = zeros(10,4);
    
    % 第j份数据作为验证集，其余9份作为训练集
    for j = 1:10
        
        % 构造训练集和验证集的输入、输出数据
        input_train = cell(0);
        output_train = cell(0);
        input_test = cell(0);
        target_test = cell(0);
        idx = 1;
        for k = 1:10
            if k == j
                for n = 1:4
                    input_test{1,n} =  path_std_crossvalid{k,n}(:,1:end-1);
                    target_test{1,n} = path_std_crossvalid{k,n}(:,2:end);
                end
            else
                for n = 1:4
                    input_train{idx} =  path_std_crossvalid{k,n}(:,1:end-1);
                    output_train{idx} =  path_std_crossvalid{k,n}(:,2:end);
                    idx = idx + 1;
                end
            end
        end

        % 创建LSTM回归网络，指定LSTM层的输入、输出、隐含单元个数
        numFeatures = 2;
        numResponses = 2;
        numHiddenUnits = 250;
        layers = [ sequenceInputLayer(numFeatures)
            lstmLayer(numHiddenUnits)
            fullyConnectedLayer(numResponses)
            regressionLayer];
        
        % 指定训练选项
        options = trainingOptions(parSet(i), ...
            'MaxEpochs',500, ...
            'GradientThreshold',1, ...
            'InitialLearnRate',0.005, ...
            'LearnRateSchedule','piecewise', ...
            'LearnRateDropPeriod',125, ...
            'LearnRateDropFactor',0.2, ...
            'Verbose',0, ...
            'Plots','training-progress');
        
        %训练LSTM
        net = trainNetwork(input_train,output_train,layers,options);
        
        % 利用第j份数据的4组轨迹验证所训练的网络的有效性,并记录RMSE
        for k = 1:4
            [RMSE_x, RMSE_y] = preTraj(net, input_test{k}, target_test{k},sig, mu);
            RMSE_x_all(j,k) = RMSE_x;
            RMSE_y_all(j,k) = RMSE_y;
        end
    end
    
    % 计算第i个超参数所有验证结果的RMSE
    RMSE_x_mean(i) = mean(mean(RMSE_x_all));
    RMSE_y_mean(i) = mean(mean(RMSE_y_all));
end

[~,idx] = min(RMSE_x_mean);
disp(strcat('-----交叉验证结束，第',num2str(idx),'个超参数最优！-----'))

%% 保存
save trainResult.mat net path_std_crossvalid RMSE_x_mean RMSE_y_mean input_test target_test
