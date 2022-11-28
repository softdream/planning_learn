clc
clear
close all
load  refPath.mat

%% ��ز�������
% �ο��켣
refPos = refPath_sin;            

% ����켣�Ĳο������
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % �����
refHeading(end+1) = refHeading(end);

% ������������
targetSpeed = 10;           % Ŀ���ٶȣ���λ�� m /s
k = 10;                      % �������
Kp = 1;                     % �ٶ�P������ϵ��
dt = 0.1;                   % ʱ��������λ��s
L = 2;                      % ������࣬��λ��m

% ������ʼ״̬����
iniPos = refPos(1,:);
iniSpd = 0;
iniHeading = refHeading(1);
%% ������
% ������ʼ״̬����
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;

% ����ʼ״̬����ʵ��״̬������
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_Stanley = [];

% ѭ��
while true
    % Ѱ��Ԥ����뷶Χ�����·����
    dist = sqrt((refPos(:,1)-currentPos(1)).^2 + (refPos(:,2)-currentPos(2)).^2);
    [~,targetIdx] = min(dist);
    
    % �ж��Ƿ񳬳�����
    if targetIdx >= size(refPos,1)-1
        break
    end
       
    % ����ǰ��ת��
    [delta,latErr] = stanleyControler(targetIdx,currentPos, currentHeading, currentSpd,refPos,refHeading,k);
    
    % ����������˳�ѭ��
    if abs(latErr) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ������ٶ�
    currentAcc = Kp* (targetSpeed-currentSpd);
    
    % ����״̬��
    [currentPos, currentHeading, currentSpd] = updateState(currentAcc,currentPos, currentHeading, currentSpd,delta,L, dt);
    
    % ����ÿһ����ʵ����
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_Stanley(end+1,:) = [cumLength(targetIdx),latErr];
    
    
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
path_Stanley = posSet;
save stanley.mat path_Stanley latErr_Stanley
%% �����ڲο��켣�������뵱ǰλ������ĵ�
function target_idx = findTargetIdx(state,refPos)
for i = 1:size(refPos,1)
    d(i,1) = norm(refPos(i,:) - state(1:2));
end
[~,target_idx] = min(d);  % �ҵ����뵱ǰλ�������һ���ο��켣������
end

%% ��ÿ�����
function [delta,latError] = stanleyControler(targetIdx,currentPos, currentHeading, currentSpd,refPos,refHeading,k);
% ���ݰٶ�Apolo������������
dx = currentPos(1) - refPos(targetIdx,1);
dy = currentPos(2) - refPos(targetIdx,2);
phi_r = refHeading(targetIdx);
latError = dy*cos(phi_r) - dx*sin(phi_r);

% �ֱ����ֻ���Ǻ�������theta��ֻ���Ǻ�������theta
theta_fai =  refHeading(targetIdx)- currentHeading;
theta_y = atan2(-k*latError,currentSpd);

% �������ǶȺϲ���Ϊǰ��ת��
delta = theta_fai + theta_y;
end

%% ����״̬��
function [pos_new, heading_new, v_new] = updateState(acc_old,pos_old, heading_old, v_old,delta,wheelbase, dt)
v_new =  v_old + acc_old*dt;
heading_new=  heading_old + v_new*dt*tan(delta)/wheelbase;
pos_new(1) = pos_old(1) + v_new*cos(heading_new)*dt;
pos_new(2) =  pos_old(2) + v_new*sin(heading_new)*dt;
end
