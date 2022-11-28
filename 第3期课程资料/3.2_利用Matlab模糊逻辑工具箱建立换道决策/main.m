clc
clear
close all

%% 构造输入、输出
%创建新的模糊推理系统
fis = mamfis; 

% 输入1:fai_d
fis = addInput(fis,[0 1],"Name", "fai_d");
fis = addMF(fis,"fai_d","trimf",[-0.25, 0, 0.25],'Name',"小");
fis = addMF(fis,"fai_d","trimf",[0, 0.25, 0.5],'Name',"较小");
fis = addMF(fis,"fai_d","trimf",[0.25, 0.5, 0.75],'Name',"中");
fis = addMF(fis,"fai_d","trimf",[0.5,0.75,1],'Name',"较大");
fis = addMF(fis,"fai_d","trimf",[0.75, 1, 1.25],'Name',"大");

% 输入2:fai_v
fis = addInput(fis,[0 1],"Name", "fai_v");
fis = addMF(fis,"fai_v","trimf",[-0.25, 0, 0.25],'Name',"小");
fis = addMF(fis,"fai_v","trimf",[0, 0.25, 0.5],'Name',"较小");
fis = addMF(fis,"fai_v","trimf",[0.25, 0.5, 0.75],'Name',"中");
fis = addMF(fis,"fai_v","trimf",[0.5,0.75,1],'Name',"较大");
fis = addMF(fis,"fai_v","trimf",[0.75, 1, 1.25],'Name',"大");

% 输出
fis = addOutput(fis,[0 1],"Name", "fai_h");
fis = addMF(fis,"fai_h","trimf",[-0.25, 0, 0.25],'Name',"弱");
fis = addMF(fis,"fai_h","trimf",[0, 0.25, 0.5],'Name',"较弱");
fis = addMF(fis,"fai_h","trimf",[0.25, 0.5, 0.75],'Name',"中");
fis = addMF(fis,"fai_h","trimf",[0.5,0.75,1],'Name',"较强");
fis = addMF(fis,"fai_h","trimf",[0.75, 1, 1.25],'Name',"强");
%% 规则库
%第1~5列分别是：fai_d，fai_v，fai_h, 规则权重, AND/OR选项
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

fis = addRule(fis,rulelist);            % 添加模糊规则函数
showrule(fis)                           % 显示模糊规则函数

%% 根据输入，得到输出  
% 先设置解模糊方法
fis.DefuzzMethod = "centroid";

% 任意给一个fai_d，fai_v，输出换道意愿fai_h
fai_d = 0.4;
fai_v = 0.6;
fai_h = evalfis(fis,[fai_d,fai_v]);  

% 根据换道意愿fai_h的大小，给出车辆决策
if fai_h > 0 && fai_h <= 0.5
    decision = 1;
    disp('车辆决策为：跟驰行驶')
elseif fai_h > 0.5 && fai_h <= 0.6
    decision = 2;
    disp('车辆决策为：等待换道')
elseif fai_h > 0.6 && fai_h <= 1
    decision = 3;
    disp('车辆决策为：执行换道')
end

%% 画出模糊系统
figure(1); plotfis(fis);            % 模糊系统连接结构图
figure(2); plotmf(fis,'input',1);   % 模糊系统第一个输入隶属函数图
figure(3); plotmf(fis,'input',2);   % 模糊系统第二个输入隶属函数图
figure(4); plotmf(fis,'output',1);  % 模糊系统输出隶属函数图
figure(5); gensurf(fis);            % 模糊规则曲面

%% 保存
writeFIS(fis,'LCD');
