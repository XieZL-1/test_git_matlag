% 使用注意事项
%在matlab 命令行窗口运行RumMain(filename,Endian,Kws).

% 参数设置如下：
% filename：文件路径
% Endian：大小端模式。       【注意事项】：INS550D系列为1 ；INS570D系列为0.
% Kws：轮速比例系数，空  

%使用方法，将各方法RunMain 将引号内的绝对地址 改成需要解析的数据地址和文件名（含后缀）即可。  
%如  RunMain('F:\asensing test01\com1_20200101.txt');


%% 解析INS550D 脚本方法：
% clc;%清除命令窗口的内容
% close all;%关闭所有的Figure窗口
% clear all;%清除工作空间的所有变量，函数，和MEX文件
% RunMain('F:\asensing test01\com1_20200101.txt',1);     %将引号内的绝对地址 改成需要解析的数据地址和文件名（含后缀）即可。  



% %%  解析INS570D 脚本方法：
% clc;%清除命令窗口的内容
% close all;%关闭所有的Figure窗口
% clear all;%清除工作空间的所有变量，函数，和MEX文件
% RunMain('F:\asensing test01\com1_20200101.txt',0);    %将引号内的绝对地址 改成需要解析的数据地址和文件名（含后缀）即可。
% 
% 

%%  解析INS570D 脚本方法：
% clc;%清除命令窗口的内容
close all;%关闭所有的Figure窗口
clear;%清除工作空间的所有变量，函数，和MEX文件
%RunMain('E:Asensing_test.txt',0);    %将引号内的绝对地址 改成需要解析的数据地址和文件名（含后缀）即可。
% RunMain('E:\3.解包脚本相关\客户解包脚本\客户解包_读取txt版\客户版解包程序（20200318注解版)\解包示例数据.txt',0);   
% RunMain('daoyuan\0831\083105.txt',0);   
% RunMain('C:\Users\xie_z\Desktop\daoyuan20220119\2.txt',0);   
% RunMain('G:\20220119204410.bin',0);   


FileName = 'G:\data\UART_Data\20220303\Daoyuan\r20220303002.txt';

[FileName2, RawData_table_DY, Time_DY] = DY_Analyze_V3(FileName);   %按照协议自行解析
FileName = FileName2;

tic
RunMain(FileName, 0);   
toc
data_daoyuan = ans;


SaveEnable = 1;
idx_dash =  find(FileName=='\',1,'last');
SaveFileFolder = FileName(1:idx_dash);

%% 从figure中提取导远标志位数据
f = figure(12);
lf=findall(f.Children(4),'type','line');
% xf=get(lf,'xdata');
yf=get(lf,'ydata');

% a = [cell2mat(yf(1))', cell2mat(yf(2))', cell2mat(yf(3))', cell2mat(yf(4))'];

data_daoyuan.SelfDefine.V_R = cell2mat(yf(4))';
data_daoyuan.SelfDefine.P_R = cell2mat(yf(3))';
data_daoyuan.SelfDefine.R2_R = cell2mat(yf(2))';
data_daoyuan.SelfDefine.H2_R = cell2mat(yf(1))';

%% 存储数据

if SaveEnable
    if exist(SaveFileFolder(1:end-1),'dir')==0
        mkdir(SaveFileFolder(1:end-1));
    end
end

if SaveEnable
    data_num = FileName(idx_dash+1:length(FileName)-4);
    save_name = strcat(SaveFileFolder, data_num);
end
if SaveEnable
    save(save_name,'data_daoyuan','RawData_table_DY','Time_DY');
    disp('将数据存放在如下位置：')
    disp(save_name)
else
    disp('这组解析后的数据没有保存')
end

%%


tt = data_daoyuan.INSData.ts - data_daoyuan.INSData.ts(1);
g = 9.78;
figure(101)
subplot(3,1,1)
plot(tt,ans.INSData.a(:,1)*g)
xlabel('x')
title('加速度m/s^2')
subplot(3,1,2)
plot(tt,ans.INSData.a(:,2)*g)
xlabel('y')
subplot(3,1,3)
plot(tt,ans.INSData.a(:,3)*g)
xlabel('z')

figure(102)
subplot(3,1,1)
plot(tt,ans.INSData.g(:,1))
xlabel('x')
title('角速度°/s')
subplot(3,1,2)
plot(tt,ans.INSData.g(:,2))
xlabel('y')
subplot(3,1,3)
plot(tt,ans.INSData.g(:,3))
xlabel('z')

%     f_figure(101,'daoyuan')
%     hold on 
%     grid on
%     plot3(tra_GNSS(:,1),tra_GNSS(:,2),tra_GNSS(:,3),'.','DisplayName','tra_GNSS');
%     hold on;plot3(tra_INS(:,1),tra_INS(:,2),tra_INS(:,3),'.','DisplayName','tra_INS');
%     legend
%     xlabel('Longitude deg')
%     ylabel('Latitude deg')
