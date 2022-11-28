clc
clear
close all

%% ֱ�߹켣
x = 1:0.1:50;
y = ones(1,length(x));
refPath_line = [x', y'];
figure
plot(refPath_line(:,1),refPath_line(:,2))

%% Բ�ι켣
t = pi/2:0.01:pi/2+2*pi-0.1;
x = cos(t)*20+20;
y = sin(t)*20+20;
refPath_circle = [x',y'];
figure
plot(refPath_circle(:,1),refPath_circle(:,2))

%% S�ι켣
% ��һ����
x1 = 1:0.1:21;
y1 = ones(1,length(x1));

% �ڶ�����
t = 3*pi/2:0.01:2*pi;
t = [t,0.01:0.01:pi/2];
x2 = cos(t)*10+21;
y2 = sin(t)*10+11;

% ��������
x3 = 21:-0.1:1;
y3 = 21*ones(1,length(x3));

% ���Ĳ���
t = 3*pi/2:-0.01:pi/2;
x4 = cos(t)*10+1;
y4 = sin(t)*10+31;

% ���岿��
x5 = 1:0.1:21;
y5 = 41*ones(1,length(x1));

refPath_S = [x1,x2,x3,x4,x5; y1,y2,y3,y4,y5];
refPath_S = refPath_S';

figure
plot(refPath_S(:,1),refPath_S(:,2))

%% path_4,8���ι켣
% ��һ����
t = 3*pi/2-0.2 : -0.01 : 3*pi/2 - 2*pi;
x = cos(t)*20;
y = sin(t)*20;

% �ڶ�����
t = pi/2+0.01:0.01:pi/2+2*pi-0.2;
x = [x,cos(t)*20];
y = [y,sin(t)*20-40];

refPath_8 = [x',y'];
figure
plot(refPath_8(:,1),refPath_8(:,2))

%% �������߹켣
x = 0:1:40*pi;
y = 20*sin(x/20);
refPath_sin = [x',y'];
figure
plot(refPath_sin(:,1),refPath_sin(:,2))

%% ����
save refPath.mat refPath_8 refPath_circle refPath_line refPath_S refPath_sin