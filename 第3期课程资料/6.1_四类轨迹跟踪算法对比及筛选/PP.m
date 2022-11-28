clc
clear
close all
load  refPath.mat

%% ��ز�������
refPos = refPath_8;
targetSpeed = 10;      % m/s
Kv = 0.1;              % ǰ�Ӿ���ϵ��
Kp = 0.8;              % �ٶ�P������ϵ��
Ld0 = 2;               % Ld0��Ԥ����������ֵ
dt = 0.1;              % ʱ��������λ��s
L = 2.9;               % ������࣬��λ��m

% ���������ۼƾ���Ͳο������
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % �����
refHeading(end+1) = refHeading(end);

% ������ʼ״̬����
iniPos = refPos(1,:);
iniSpd = 0;
iniHeading = refHeading(1);

%% ������
% ��ǰ������Ϣ
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;
 
% ����ʼ״̬����ʵ��״̬������
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_PP = [];

% ѭ�������켣��
while true
    % Ѱ��Ԥ����뷶Χ�����·����
    targetIdx = findLookaheadPoint(currentPos, currentSpd, refPos, Kv, Ld0);
    
    % �ж��Ƿ񳬳�����
    if targetIdx >= size(refPos,1)
        break
    end
   
    % ���������
    [delta,latErr]  = PP_controller(targetIdx,currentPos, currentHeading, currentSpd, refPos,refHeading, Kv, Ld0,L);
    
    % ����������˳�ѭ��
    if abs(latErr) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ������ٶ�
    currentAcc = Kp* (targetSpeed-currentSpd)/dt;
    
    % ����״̬��
    [currentPos, currentHeading, currentSpd] = updateState(currentAcc,currentPos, currentHeading, currentSpd,delta,L, dt);
    
    % ����ÿһ����ʵ����
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_PP(end+1,:) = [cumLength(targetIdx),latErr];    
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
path_PP = posSet;
save PP.mat path_PP latErr_PP

%% �����ڲο��켣�������뵱ǰ����λ������ĵ�
function  targetIdx = findLookaheadPoint(currentPos, currentSpd, refPos, Kv, Ld0)
% �ҵ����뵱ǰλ�������һ���ο��켣������
deltX = refPos(:,1)-currentPos(1);
deltY = refPos(:,2)-currentPos(2);
dist = sqrt(deltX.^2 + deltY.^2);
[~,idx] = min(dist); 
Ld = Kv*currentSpd + Ld0;       % Ld0��Ԥ����������ֵ��


% �Ӹõ㿪ʼ��켣ǰ���������ҵ���Ԥ������������һ���켣��
L_steps = 0;           % �ο��켣�ϼ������ڵ���ۼƾ���
while L_steps < Ld && idx < size(refPos,1)
    L_steps = L_steps + norm(refPos(idx + 1,:) - refPos(idx,:));
    idx = idx+1;
end
targetIdx = idx;
end


%% ��ÿ�������ǰ��ת��
function [delta,latError] = PP_controller(targetIdx,currentPos, currentHeading, currentSpd,refPos,refHeading, Kv, Ld0, L)
lookaheadPoint = refPos(targetIdx,:);
alpha = atan2(lookaheadPoint(2) - currentPos(2), lookaheadPoint(1) - currentPos(1))  - currentHeading;
Ld = Kv*currentSpd + Ld0;

% ��λ�á�����ǵ����
x_error  = currentPos(1) - refPos(targetIdx,1);
y_error = currentPos(2) - refPos(targetIdx,2);
heading_r = refHeading(targetIdx);

% ���ݰٶ�Apolo������������
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% ǰ��ת��
delta = atan2(2*L*sin(alpha), Ld);

end

%% ����״̬��
function [pos_new, heading_new, v_new] = updateState(acc_old,pos_old, heading_old, v_old,delta,wheelbase, dt)
v_new =  v_old + acc_old*dt;
heading_new=  heading_old + v_new*dt*tan(delta)/wheelbase;
pos_new(1) = pos_old(1) + v_new*cos(heading_new)*dt;
pos_new(2) =  pos_old(2) + v_new*sin(heading_new)*dt;
end

