clc
clear
close all
load  PP.mat
load  stanley.mat
load  LQR.mat
load  MPC.mat
%%
figure
plot(latErr_PP(:,1),latErr_PP(:,2),'c','linewidth',1.5);
hold on
plot(latErr_Stanley(:,1),latErr_Stanley(:,2),'m','linewidth',1.5);
plot(latErr_LQR(:,1),latErr_LQR(:,2),'b','linewidth',1.5);
plot(latErr_MPC(:,1),latErr_MPC(:,2),'r','linewidth',1.5);
legend('PP','Stanley','LQR','MPC');
grid on
xlabel('距离/m');
ylabel('横向误差/m');

