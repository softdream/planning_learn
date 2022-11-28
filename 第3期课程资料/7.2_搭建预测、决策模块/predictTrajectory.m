% ���߻��ÿһ��ʱ�䲽��Ԥ��켣
clc
clear
close all
load refInfo.mat     
load Scenario.mat
load dataSet.mat

%% �����ҳ���������·���㣨��ͨ����ʻ·�������б�׼������
for i = 1:2
    refInfo_obs1_std(:,i) = (refInfo_obs1(:,i) - mu(i)) / sig(i);
    refInfo_obs2_std(:,i) = (refInfo_obs2(:,i) - mu(i)) / sig(i);
end

%% ��������
% ���ݲ��Լ��������Ź������������Ҳ�ȡ���һ����������ʼ�������
input1 = refInfo_obs1_std(1:end-1,:)';
input2 = refInfo_obs2_std(1:end-1,:)';

% Ԥ��ʱ�򳤶ȼ���ʼԤ��֡�ȶ���
predLen = 80;  
frameFlag1 = 80;                                       % ��frameFlag��ʼ���Ԥ��
frameFlag2 = 60;                                     

%% Ԥ��켣
load trainResult.mat  net
% ������ʼԤ��֡��ʼ��Ԥ�����
YPred1 =  input1(:,1:frameFlag1);
for i = 1:size(input1,2)-frameFlag1-1
% ѭ������Ԥ��
    for j = 1:predLen       
        % ����Ԥ��
        [net,YPred_temp] = predictAndUpdateState(net,YPred1);
        YPred1 = [YPred1,YPred_temp(:,end)];
    end
    
    % ׷�ӵ������
    output1_x(i,:) = YPred1(1,frameFlag1+1:end);
    output1_y(i,:) = YPred1(2,frameFlag1+1:end);

    %һ��Ԥ��ʱ������󣬽���ʱ��������ʵ�ʹ켣������һ�θ�������
    YPred1 = input1(:,i+1:frameFlag1+i);
end   

load trainResult.mat  net
% ������ʼԤ��֡��ʼ��Ԥ�����
YPred2 =  input2(:,1:frameFlag2);
for i = 1:size(input2,2)-frameFlag2-1
% ѭ������Ԥ��
    for j = 1:predLen       
        % ����Ԥ��
        [net,YPred_temp] = predictAndUpdateState(net,YPred2);
        YPred2 = [YPred2,YPred_temp(:,end)];
    end
    
    % ׷�ӵ������
    output2_x(i,:) = YPred2(1,frameFlag2+1:end);
    output2_y(i,:) = YPred2(2,frameFlag2+1:end);

    %һ��Ԥ��ʱ������󣬽���ʱ��������ʵ�ʹ켣������һ�θ�������
    YPred2 = input2(:,i+1:frameFlag2+i);
end   

%% ����
% ���ݱ�׼����sig��mu����÷���׼�������ֵ
for i = 1:size(output1_x,1)
    output1_x(i,:) = sig(1)*output1_x(i,:) + mu(1); 
    output2_x(i,:) = sig(1)*output2_x(i,:) + mu(1);
end
for i = 1:size(output1_y,1)
    output1_y(i,:) = sig(2)*output1_y(i,:) + mu(2);
    output2_y(i,:) = sig(2)*output2_y(i,:) + mu(2);
end
% ת��ʱ�����
time1 = 0:0.1:(size(output1_x,1)-1) / 10;
time2 = 0:0.1:(size(output2_x,1)-1) / 10;
obsPrePos1_x = timeseries(output1_x, time1);
obsPrePos1_y = timeseries(output1_y, time1);
obsPrePos2_x = timeseries(output2_x, time2);
obsPrePos2_y = timeseries(output2_y, time2);

%% ����ʵ�ʹ켣��Ԥ��켣����
save obsPre.mat obsPrePos1_x obsPrePos1_y obsPrePos2_x obsPrePos2_y
