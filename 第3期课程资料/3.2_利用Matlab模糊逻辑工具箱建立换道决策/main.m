clc
clear
close all

%% �������롢���
%�����µ�ģ������ϵͳ
fis = mamfis; 

% ����1:fai_d
fis = addInput(fis,[0 1],"Name", "fai_d");
fis = addMF(fis,"fai_d","trimf",[-0.25, 0, 0.25],'Name',"С");
fis = addMF(fis,"fai_d","trimf",[0, 0.25, 0.5],'Name',"��С");
fis = addMF(fis,"fai_d","trimf",[0.25, 0.5, 0.75],'Name',"��");
fis = addMF(fis,"fai_d","trimf",[0.5,0.75,1],'Name',"�ϴ�");
fis = addMF(fis,"fai_d","trimf",[0.75, 1, 1.25],'Name',"��");

% ����2:fai_v
fis = addInput(fis,[0 1],"Name", "fai_v");
fis = addMF(fis,"fai_v","trimf",[-0.25, 0, 0.25],'Name',"С");
fis = addMF(fis,"fai_v","trimf",[0, 0.25, 0.5],'Name',"��С");
fis = addMF(fis,"fai_v","trimf",[0.25, 0.5, 0.75],'Name',"��");
fis = addMF(fis,"fai_v","trimf",[0.5,0.75,1],'Name',"�ϴ�");
fis = addMF(fis,"fai_v","trimf",[0.75, 1, 1.25],'Name',"��");

% ���
fis = addOutput(fis,[0 1],"Name", "fai_h");
fis = addMF(fis,"fai_h","trimf",[-0.25, 0, 0.25],'Name',"��");
fis = addMF(fis,"fai_h","trimf",[0, 0.25, 0.5],'Name',"����");
fis = addMF(fis,"fai_h","trimf",[0.25, 0.5, 0.75],'Name',"��");
fis = addMF(fis,"fai_h","trimf",[0.5,0.75,1],'Name',"��ǿ");
fis = addMF(fis,"fai_h","trimf",[0.75, 1, 1.25],'Name',"ǿ");
%% �����
%��1~5�зֱ��ǣ�fai_d��fai_v��fai_h, ����Ȩ��, AND/ORѡ��
rulelist=[1 1 2 1 1;             
    1 2 3 1 1;
    1 3 4 1 1;
    1 4 5 1 1;
    1 5 5 1 1;
    
    2 1 2 1 1;
    2 2 3 1 1;
    2 3 4 1 1;
    2 4 5 1 1;
    2 5 5 1 1;
    
    3 1 1 1 1;
    3 2 2 1 1;
    3 3 3 1 1;
    3 4 4 1 1;
    3 5 4 1 1;
    
    4 1 1 1 1;
    4 2 1 1 1;
    4 3 2 1 1;
    4 4 3 1 1;
    4 5 4 1 1;
    
    5 1 1 1 1;
    5 2 1 1 1;
    5 3 1 1 1;
    5 4 2 1 1;
    5 5 3 1 1;];

fis = addRule(fis,rulelist);            % ���ģ��������
showrule(fis)                           % ��ʾģ��������

%% �������룬�õ����  
% �����ý�ģ������
fis.DefuzzMethod = "centroid";

% �����һ��fai_d��fai_v�����������Ըfai_h
fai_d = 0.4;
fai_v = 0.6;
fai_h = evalfis(fis,[fai_d,fai_v]);  

% ���ݻ�����Ըfai_h�Ĵ�С��������������
if fai_h > 0 && fai_h <= 0.5
    decision = 1;
    disp('��������Ϊ��������ʻ')
elseif fai_h > 0.5 && fai_h <= 0.6
    decision = 2;
    disp('��������Ϊ���ȴ�����')
elseif fai_h > 0.6 && fai_h <= 1
    decision = 3;
    disp('��������Ϊ��ִ�л���')
end

%% ����ģ��ϵͳ
figure(1); plotfis(fis);            % ģ��ϵͳ���ӽṹͼ
figure(2); plotmf(fis,'input',1);   % ģ��ϵͳ��һ��������������ͼ
figure(3); plotmf(fis,'input',2);   % ģ��ϵͳ�ڶ���������������ͼ
figure(4); plotmf(fis,'output',1);  % ģ��ϵͳ�����������ͼ
figure(5); gensurf(fis);            % ģ����������

%% ����
writeFIS(fis,'LCD');
