clc
clear
close all
load  refPath.mat

%% ��ز�������
dt = 0.1;
L = 2.9 ;
Q = 10*eye(3);
R = eye(2)* 2;

%% �켣����
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
    refCur(i) = abs(getCur(x,y));
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

% �켣����ʵ����
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErr_LQR = [];

% ѭ��
while true
     % Ѱ��Ԥ����뷶Χ�����·����
    dist = sqrt((refPos(:,1)-currentPos(1)).^2 + (refPos(:,2)-currentPos(2)).^2);
    [~,targetIdx] = min(dist); 
    
    % �ж��Ƿ񳬳�����
    if targetIdx >= size(refPos,1)-2
        break
    end
    
    % LQR������
    [v_delta,delta,delta_r,latErr] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refCur,L,Q,R,dt);    
    
    % ����������˳�ѭ��
    if abs(latErr) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ����״̬
    [currentPos,currentHeading,currentSpd,Delta] = update(currentPos,currentHeading,currentSpd, v_delta,delta, dt,L, refSpeed,delta_r);
    
    % ����ÿһ����ʵ����
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErr_LQR(end+1,:) = [cumLength(targetIdx),latErr];
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
path_LQR = posSet;
save LQR.mat path_LQR latErr_LQR


%% LQR����
function [v_delta,Delta_delta,delta_r,latError] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refCur,L,Q,R,dt)
% ��λ�á�����ǲο���
x_r = refPos(targetIdx,1);
y_r = refPos(targetIdx,2);
heading_r = refHeading(targetIdx);
delta_r = refDelta(targetIdx);

% ��λ�á�����ǵ����
x_error  = currentPos(1) - x_r;
y_error = currentPos(2) - y_r;
yaw_error =  currentHeading - heading_r;

% ���ݰٶ�Apolo������������
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% �����ֵ��ֵ��״̬��
X(1,1) = x_error; 
X(2,1) = y_error;  
X(3,1) = yaw_error;

% ��״̬���̾���ϵ��������K
A = [1,  0,  -currentSpd*dt*sin(heading_r);
     0,  1,  currentSpd * dt * cos(heading_r);
     0,  0,  1];
B = [dt * cos(heading_r),    0;
     dt * sin(heading_r),    0;
     dt * tan(heading_r)/L,  currentSpd*dt/(L * cos(delta_r)^2)];


K = calcu_K(A,B,Q,R);

% ����ٶ��������ǰ��ת�����������������
u = -K * X;  % 2��1��
v_delta = u(1);      
Delta_delta = u(2);

end


%% ��������
function K = calcu_K (A,B,Q,R)

% ��ֹ��������
iter_max = 500;
epsilon = 0.01;

% ѭ��
P_old = Q;
for i = 1:iter_max
    P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q;
    if abs(P_new - P_old) <= epsilon
        break
    else
        P_old = P_new; 
    end
end

P = P_new;
K = (B' * P * B + R) \ (B' * P * A);  % 2��3��
end

%% ����״̬
function [pos_new, currentHeading, currentSpd, Delta] = update(pos_old, currentHeading, currentSpd, currentSpd_delta,Delta_delta,dt,L,refSpeed,refDelta)
Delta = refDelta + Delta_delta;
currentHeading = currentHeading + currentSpd / L * tan(Delta) * dt;
currentSpd = refSpeed + currentSpd_delta;
pos_new(1) = pos_old(1) + currentSpd*cos(currentHeading)*dt;
pos_new(2) =  pos_old(2) + currentSpd*sin(currentHeading)*dt;
end
