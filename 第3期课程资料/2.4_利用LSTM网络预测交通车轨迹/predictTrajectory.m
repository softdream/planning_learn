% ����LSTM����Ԥ��
clc
clear
close all
load trainResult.mat     
load dataSet.mat     
load Scenario.mat
%% ��������
% ���ݲ��Լ��������Ź������������Ҳ�ȡ���һ����������ʼ�������
input = input_test{1,1}(:,1:end-1);
output = [];
target = target_test{1,1};

% Ԥ��ʱ�򳤶ȼ���ʼԤ��֡�ȶ���
predLen = 10;  
frameFlag = 50;                                       % ��frameFlag��ʼ���Ԥ��
endFlag = mod(size(input,2)-frameFlag, predLen);      % ����ȥ��������һС������
steps = floor( (size(input,2)-frameFlag) / predLen ); % ����Ԥ��켣���ܴ���

%% Ԥ��켣
% ������ʼԤ��֡��ʼ��Ԥ�����
YPred =  input(:,1:frameFlag);

% ѭ������Ԥ��
for i = 1:steps    
    for j = 1:predLen       
        % ����Ԥ��
        [net,YPred_temp] = predictAndUpdateState(net,YPred);
        YPred = [YPred,YPred_temp(:,end)];
    end
    
    % ׷�ӵ������
    output = [output,YPred(:,frameFlag+(i-1)*predLen+1:frameFlag+i*predLen)];

    %һ��Ԥ��ʱ������󣬽���ʱ��������ʵ�ʹ켣������һ�θ�������
    YPred = input(:,1:frameFlag+i*predLen);
end   

% ���ݱ�׼����sig��mu����÷���׼�������ֵ
for i = 1:2 
    output(i,:) = sig(i)*output(i,:) + mu(i);
    target(i,:) = sig(i)*target(i,:) + mu(i);
end

% ��ֵ
actualPos = target(:,frameFlag+1 : end-endFlag-1);
predictedPos = output;

%% �������
% ������ľ������
error_x = abs(actualPos(1,:) - predictedPos(1,:));
error_y = abs(actualPos(2,:) - predictedPos(2,:));

% �������������ֵ
error_x_max = max(error_x);
error_y_max = max(error_y);

% ���ƽ��ֵ
error_x_mean = mean(error_x);
error_y_mean = mean(error_y);

% ����������ƽ��ֵ
RMSE_x = sqrt(mean((actualPos(1,:) - predictedPos(1,:)).^2));
RMSE_y = sqrt(mean((actualPos(2,:) - predictedPos(2,:)).^2 ));

%% ��ͼ
% X���
figure
hold on
plot(1:length(error_x), error_x,'r');
title('X���')
xlabel('��������')
ylabel('���/m')

% Y���
figure
hold on
plot(1:length(error_y), error_y,'r');
title('Y���')
xlabel('��������')
ylabel('���/m')

% ʵ�ʹ켣��Ԥ��켣�Ա�
figure
hold on
plot(actualPos(1,:), actualPos(2,:),'r');
plot(predictedPos(1,:), predictedPos(2,:),'b--');
title('ʵ�ʹ켣��Ԥ��켣�Ա�')
legend('ʵ�ʹ켣','Ԥ��켣')
xlabel('X����/m')
ylabel('Y����/m')
%% ����ʵ�ʹ켣��Ԥ��켣����
save predictResult.mat actualPos predictedPos
