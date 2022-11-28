function [RMSE_x, RMSE_y] = preTraj(net, input_test, target_test,sig ,mu)

% Ԥ��ʱ�򳤶ȼ���ʼԤ��֡�ȶ���
predLen = 10;  
frameFlag = 50;                                            % ��frameFlag��ʼ���Ԥ��
endFlag = mod(size(input_test,2)-frameFlag, predLen);      % ����ȥ��������һС������
steps = floor( (size(input_test,2)-frameFlag) / predLen ); % ����Ԥ��켣���ܴ���

%% Ԥ��켣
% ������ʼԤ��֡��ʼ��Ԥ�����
YPred =  input_test(:,1:frameFlag);
output_test = [];
% ѭ������Ԥ��
for i = 1:steps    
    for j = 1:predLen       
        [net,YPred_temp] = predictAndUpdateState(net,YPred);
        YPred = [YPred,YPred_temp(:,end)];
    end
    
    % ׷�ӵ������
    output_test = [output_test,YPred(:,frameFlag+(i-1)*predLen+1:frameFlag+i*predLen)];

    %һ��Ԥ��ʱ������󣬽���ʱ��������ʵ�ʹ켣������һ�θ�������
    YPred = input_test(:,1:frameFlag+i*predLen);
end   

% ���ݱ�׼����sig��mu����÷���׼�������ֵ
for i = 1:2 
    output_test(i,:) = sig(i)*output_test(i,:) + mu(i);
    target_test(i,:) = sig(i)*target_test(i,:) + mu(i);
end

% Ԥ��λ�ú�ʵ��λ��
actualPos = target_test(:,frameFlag+1:end - endFlag);
predictedPos = output_test;

%% �������
RMSE_x = sqrt(mean((actualPos(1,:) - predictedPos(1,:)).^2));
RMSE_y = sqrt(mean((actualPos(2,:) - predictedPos(2,:)).^2 ));
