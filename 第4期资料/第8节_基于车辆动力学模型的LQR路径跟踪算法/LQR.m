clc
clear
close all
load  refPath.mat

%% ��ز�������
dt = 0.1;
L = 2.9;
Q = 1*eye(4);
R = 1*eye(1);

%% �켣����
% ����ο��켣
refPos = refPath_line;
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
iniSpd = 1;
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
latErrSet = [];

latError_last = 0;
yaw_error_last = 0;
omega = 0;
actualCur = 0.0001;
targetIdx = 1;
% ѭ��
while true
     % Ѱ��Ԥ����뷶Χ�����·����
     if size(refPos,1) - targetIdx> 20
         dist = sqrt((refPos(targetIdx:targetIdx+20,1)-currentPos(1)).^2 + ...
             (refPos(targetIdx:targetIdx+20,2)-currentPos(2)).^2);
     else
         dist = sqrt((refPos(targetIdx:end,1)-currentPos(1)).^2 + ...
             (refPos(targetIdx:end,2)-currentPos(2)).^2);
     end
     [~,targetIdx_temp] = min(dist);
     targetIdx = targetIdx + targetIdx_temp - 1;

    
    % �ж��Ƿ񳬳�����
    if targetIdx >= size(refPos,1)
        break
    end
    
    % �ٶ�P������
    a = 0.8* (refSpeed-currentSpd)/dt;
    
    % LQR������
    [Delta,delta_r,latError,yaw_error] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refSpeed,refCur,L,Q,R,dt,latError_last,yaw_error_last,actualCur,omega);    
   
    % ����������˳�ѭ��
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ����״̬
    [currentPos,currentHeading,currentSpd,omega] = update(currentPos,currentHeading,currentSpd, a,Delta, dt,L);
    
    % ����ÿһ����ʵ����
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErrSet(end+1,:) = [cumLength(targetIdx),latError];
    
    %
    latError_last = latError;
    yaw_error_last = yaw_error;
    
    % ����ʵ��·������
    if size(posSet,1) >= 3
        actualCur = getCur(posSet(end-2:end,1), posSet(end-2:end,2));
    else
        actualCur = 0.00001;
    end

end
latErr_mean = mean(abs(latErrSet(:,2)));

%%  ��ͼ
% ��·������ͼ
figure
hold on
grid on
plot(refPos(:,1), refPos(:,2),  'Color', 'b');
scatter(posSet(:,1),posSet(:,2),10,'MarkerEdgeColor','r',...
    'MarkerFaceColor','r')   % ������ɢ��
plot(posSet(:,1), posSet(:,2),  'Color', 'r');
xlabel('�������� / m');
ylabel('�������� / m');
legend('�滮�����켣', 'ʵ����ʻ�켣');


% �����ͼ
figure
hold on
grid on
plot(latErrSet(1:end,1), latErrSet(1:end,2),'LineWidth', 3,  'Color', 'b');
plot([0,latErrSet(end,1)], [latErr_mean,latErr_mean], 'r:','LineWidth', 3);
xlabel('���� / m');
ylabel('������� / m');
legend('�������', 'ƽ���������');

%% LQR����
function [Delta,delta_r,latError,yaw_error] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refSpd,refCur,L,Q,R,dt,latError_last,yaw_error_last,actualCur,omega)
% ��������
m = 1723;          % ����
lf = 1.232;        % ���ĵ�ǰ��ľ���
lr = 1.468;        % ���ĵ�����ľ���
C_af = -66900;     % ǰ�ֲ�ƫ�ն�
C_ar = -62700;     % ���ֲ�ƫ�ն�
Iz = 4175;         % ��Z���ת������

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

% ����仯��
latError_dot = currentSpd * sin(yaw_error); 
yawError_dot = omega - currentSpd*refCur(targetIdx)*cos(yaw_error)/(1+refCur(targetIdx)*latError);

% �����ֵ��ֵ��״̬��
X(1,1) = latError; 
X(2,1) = latError_dot;  
X(3,1) = yaw_error;
X(4,1) = yawError_dot;

% ��״̬���̾���ϵ��������K
A = [0,  1,  0, 0;
    0,  2*(C_af + C_ar)/(m*currentSpd), -2*(C_af + C_ar)/m, 2*(lf*C_af - lr*C_ar)/(m*currentSpd);
     0,  0,  0, 1;
     0,  2*(lf*C_af - lr*C_ar)/(Iz*currentSpd), -2*(lf*C_af - lr*C_ar)/Iz, 2*(lf^2*C_af + lr^2*C_ar)/(Iz*currentSpd)];
B = [0; -2*C_af/m; 0; -2*lf*C_af/Iz];

K = lqr(A,B,Q,R);

% ��ÿ�����
u = -K * X; 
Delta = max(u,-0.44);
Delta = min(Delta,0.44);

end

%% ����״̬
function [pos_new, currentHeading_new, currentSpd, omega] = update(currentPos,currentHeading,currentSpd, a,Delta, dt,L)
currentSpd = currentSpd + a*dt;
omega =  currentSpd / L * tan(Delta);
currentHeading_new = currentHeading + currentSpd / L * tan(Delta) * dt;
pos_new(1) = currentPos(1) + currentSpd*cos(currentHeading_new)*dt;
pos_new(2) =  currentPos(2) + currentSpd*sin(currentHeading_new)*dt;
end
