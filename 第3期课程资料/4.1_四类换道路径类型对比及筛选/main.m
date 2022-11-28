clc
clear
close all
%% ��ʼ������
d = 3.5;                  % ��·��׼���
startPt = [0,-d/2];       % ������
endPt = [50,d/2];         % ������

%% Dubins����
d_temp = 10;
A = startPt;
F = endPt;
path_dubins = dubins(A,F,d_temp);
for i = 1:size(path_dubins,1)-2
    x = path_dubins(i:i+2,1);
    y = path_dubins(i:i+2,2);
    cur_dubins(i) = cur(x,y);
end
diffX = diff(path_dubins(:,1));
diffY = diff(path_dubins(:,2));
len_dubins = cumsum(sqrt(diffX.^2 + diffY.^2));

%% �����ҹ켣����
omega = pi/(endPt(1) - startPt(1));
x = (startPt(1):endPt(1))';
y = d/2*sin(omega*(x-(startPt(1)+endPt(1))/2));
path_sin = [x,y];
for i = 1:size(path_sin,1)-2
    x = path_sin(i:i+2,1);
    y = path_sin(i:i+2,2);
    cur_sin(i) = cur(x,y);
end
diffX = diff(path_sin(:,1));
diffY = diff(path_sin(:,2));
len_sin = cumsum(sqrt(diffX.^2 + diffY.^2));

%% ����������
P0 = startPt;
P1 = [25, -d/2];
P2 = [25, d/2];
P3 = endPt;
path_bessel = getBesselCurve(P0, P1, P2, P3);
for i = 1:size(path_bessel,1)-2
    x = path_bessel(i:i+2,1);
    y = path_bessel(i:i+2,2);
    cur_bessel(i) = cur(x,y);
end
diffX = diff(path_bessel(:,1));
diffY = diff(path_bessel(:,2));
len_bessel = cumsum(sqrt(diffX.^2 + diffY.^2));

%% B��������
P1 = startPt;
P6 = endPt;
P2 = [P1(1)+5,-d/2];
P3 = [P1(1)+10,-d/2];
P4 = [P6(1)-10,d/2];
P5 = [P6(1)-5,d/2];
P = [P1;P2;P3;P4;P5;P6]';          
n = size(P,2)-1;                  
k = 3;                                % k��B����
path_Bspline = B_spline_func(P,n,k);  % ����B����������������
for i = 1:size(path_Bspline,1)-2
    x = path_Bspline(i:i+2,1);
    y = path_Bspline(i:i+2,2);
    cur_Bspline(i) = cur(x,y);
end
diffX = diff(path_Bspline(:,1));
diffY = diff(path_Bspline(:,2));
len_Bspline = cumsum(sqrt(diffX.^2 + diffY.^2));

%% ��ͼ
% ��·��
figure
hold on
grid on
plot(path_dubins(:,1),path_dubins(:,2),'b');
plot(path_sin(:,1),path_sin(:,2),'c');
plot(path_bessel(:,1),path_bessel(:,2),'k');
plot(path_Bspline(:,1),path_Bspline(:,2),'r');
legend('Dubins','Sin','Bessel','B-spline')

% ������ͼ
figure
hold on
grid on
plot(len_dubins(1:end-1),cur_dubins,'b');
plot(len_sin(1:end-1),cur_sin,'c');
plot(len_bessel(1:end-1),cur_bessel,'k');
plot(len_Bspline(1:end-1),cur_Bspline,'r');
legend('Dubins','Sin','Bessel','B-spline')