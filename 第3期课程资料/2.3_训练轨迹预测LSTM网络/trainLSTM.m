clc
clear
load dataSet.mat

%% ��40��켣���ֳ�10�ݣ�ÿ��4������
% ����10�۽�����֤�������������ֵ����
indices = crossvalind('Kfold', length(path_std), 10);

% ��40�����ݰ���indices�ֳ�10�ݣ������Ԫ������
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

%% LSTM����ѵ��
parSet = {'sgdm','rmsprop','adam'};  % ��������
for i = 1:length(parSet)
    RMSE_x_all = zeros(10,4);
    RMSE_y_all = zeros(10,4);
    
    % ��j��������Ϊ��֤��������9����Ϊѵ����
    for j = 1:10
        
        % ����ѵ��������֤�������롢�������
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

        % ����LSTM�ع����磬ָ��LSTM������롢�����������Ԫ����
        numFeatures = 2;
        numResponses = 2;
        numHiddenUnits = 250;
        layers = [ sequenceInputLayer(numFeatures)
            lstmLayer(numHiddenUnits)
            fullyConnectedLayer(numResponses)
            regressionLayer];
        
        % ָ��ѵ��ѡ��
        options = trainingOptions(parSet(i), ...
            'MaxEpochs',500, ...
            'GradientThreshold',1, ...
            'InitialLearnRate',0.005, ...
            'LearnRateSchedule','piecewise', ...
            'LearnRateDropPeriod',125, ...
            'LearnRateDropFactor',0.2, ...
            'Verbose',0, ...
            'Plots','training-progress');
        
        %ѵ��LSTM
        net = trainNetwork(input_train,output_train,layers,options);
        
        % ���õ�j�����ݵ�4��켣��֤��ѵ�����������Ч��,����¼RMSE
        for k = 1:4
            [RMSE_x, RMSE_y] = preTraj(net, input_test{k}, target_test{k},sig, mu);
            RMSE_x_all(j,k) = RMSE_x;
            RMSE_y_all(j,k) = RMSE_y;
        end
    end
    
    % �����i��������������֤�����RMSE
    RMSE_x_mean(i) = mean(mean(RMSE_x_all));
    RMSE_y_mean(i) = mean(mean(RMSE_y_all));
end

[~,idx] = min(RMSE_x_mean);
disp(strcat('-----������֤��������',num2str(idx),'�����������ţ�-----'))

%% ����
save trainResult.mat net path_std_crossvalid RMSE_x_mean RMSE_y_mean input_test target_test
