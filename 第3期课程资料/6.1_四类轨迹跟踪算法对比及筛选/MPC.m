clc
clear
close all
load  refPath.mat

%% ��ʼ����
dt = 0.1;   % ʱ�䲽��
L = 2.9;    % ���

%% �ο��켣����ز���
% ����ο��켣
refPos = refPath_sin;
refSpeed = 10;    % �ο��ٶ�

% ���㺽���
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % �����
refHeading(end+1) = refHeading(end);

% ����
for i = 1:size(refPos,1)-2
    x = refPos(i:i+2,1);
    y = refPos(i:i+2,2);
    refCur(i) = getCur(x,y);
end
refCur(i+1) = refCur(i);
refCur(i+2) = refCur(i);

% ���ݰ�����ת��ԭ������ο�ǰ��ת��
refDelta = atan(L*refCur);

% ������ʼ״̬����
iniPos = refPos(1,:);
iniSpd = 0;
iniHeading = refHeading(1);
%% ������
% ��ǰ������Ϣ
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;

% ����ֵ
Delta = 0;
U = [0.01;0.01];

% �켣����ʵ����
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_MPC = [];

% ѭ��
while true
    % Ѱ��Ԥ����뷶Χ�����·����
    dist = sqrt((refPos(:,1)-currentPos(1)).^2 + (refPos(:,2)-currentPos(2)).^2);
    [~,targetIdx] = min(dist); 
    
    % ����MPC������
    [Delta,currentSpd,idx,latErr,U] = MPC_controller(targetIdx,currentPos,currentHeading,refPos,refHeading,refDelta,dt,L,U,refSpeed);
    
    % �ж��Ƿ񳬳�����
    if targetIdx >= size(refPos,1)-2
        break
    end
    
    
    % ���̫���˳�����
    if abs(latErr) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ����״̬��
    [currentPos,currentHeading] = updateState(currentPos,currentHeading, currentSpd, Delta, dt,L); 
    
    % ����ÿһ����ʵ����
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_MPC(end+1,:) = [cumLength(targetIdx),latErr];
    
end

% ��ͼ
figure
plot(refPos(:,1), refPos(:,2), 'b');
xlabel('�������� / m');
ylabel('�������� / m');
hold on 
for i = 1:size(posSet,1)
    scatter(posSet(i,1), posSet(i,2),150, '.r');
    pause(0.01)
end
legend('�滮�����켣', 'ʵ����ʻ�켣')

% ����
path_MPC = posSet;
save MPC.mat path_MPC latErr_MPC

%%
function [currentPos,currentHeading] = updateState(currentPos,currentHeading, currentSpd, Delta, dt,L)
max_steer = 60 * pi/180; 
Delta = max(min(max_steer, Delta), -max_steer);
currentPos(1) = currentPos(1) + currentSpd * cos(currentHeading) * dt;
currentPos(2) = currentPos(2) + currentSpd * sin(currentHeading) * dt;
currentHeading = currentHeading + currentSpd / L * tan(Delta) * dt ;
end