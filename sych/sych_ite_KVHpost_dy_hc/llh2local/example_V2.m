
clear 
% close all

addpath('att_q_function20210707')
load('STM32_20210630203806.mat')



%%
[xyz_local] = llh2local([RawData_table.Latitude_Fusion_Double, RawData_table.Longitude_Fusion_Double, RawData_table.ins_Height]);

%%
figure(1)
hold on
plot(xyz_local(:,1),xyz_local(:,2))
xlabel('x')
ylabel('y')
grid on
legend
%%
[xyz_local] = llh2local_V2([RawData_table.Latitude_Fusion_Double, RawData_table.Longitude_Fusion_Double, RawData_table.ins_Height],[RawData_table.Latitude_Fusion_Double(1)-1e-5, RawData_table.Longitude_Fusion_Double(1), RawData_table.ins_Height(1)]);
%%
figure(1)
hold on
plot(xyz_local(:,1),xyz_local(:,2))
xlabel('x')
ylabel('y')
grid on
legend

% figure
% hold on
% plot3(xyz_local(:,1),xyz_local(:,2),xyz_local(:,3))
% xlabel('x')
% ylabel('y')
% zlabel('z')
% 
 

