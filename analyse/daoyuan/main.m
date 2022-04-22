% ʹ��ע������
%��matlab �����д�������RumMain(filename,Endian,Kws).

% �����������£�
% filename���ļ�·��
% Endian����С��ģʽ��       ��ע�������INS550Dϵ��Ϊ1 ��INS570Dϵ��Ϊ0.
% Kws�����ٱ���ϵ������  

%ʹ�÷�������������RunMain �������ڵľ��Ե�ַ �ĳ���Ҫ���������ݵ�ַ���ļ���������׺�����ɡ�  
%��  RunMain('F:\asensing test01\com1_20200101.txt');


%% ����INS550D �ű�������
% clc;%�������ڵ�����
% close all;%�ر����е�Figure����
% clear all;%��������ռ�����б�������������MEX�ļ�
% RunMain('F:\asensing test01\com1_20200101.txt',1);     %�������ڵľ��Ե�ַ �ĳ���Ҫ���������ݵ�ַ���ļ���������׺�����ɡ�  



% %%  ����INS570D �ű�������
% clc;%�������ڵ�����
% close all;%�ر����е�Figure����
% clear all;%��������ռ�����б�������������MEX�ļ�
% RunMain('F:\asensing test01\com1_20200101.txt',0);    %�������ڵľ��Ե�ַ �ĳ���Ҫ���������ݵ�ַ���ļ���������׺�����ɡ�
% 
% 

%%  ����INS570D �ű�������
% clc;%�������ڵ�����
close all;%�ر����е�Figure����
clear;%��������ռ�����б�������������MEX�ļ�
%RunMain('E:Asensing_test.txt',0);    %�������ڵľ��Ե�ַ �ĳ���Ҫ���������ݵ�ַ���ļ���������׺�����ɡ�
% RunMain('E:\3.����ű����\�ͻ�����ű�\�ͻ����_��ȡtxt��\�ͻ���������20200318ע���)\���ʾ������.txt',0);   
% RunMain('daoyuan\0831\083105.txt',0);   
% RunMain('C:\Users\xie_z\Desktop\daoyuan20220119\2.txt',0);   
% RunMain('G:\20220119204410.bin',0);   


FileName = 'G:\data\UART_Data\20220303\Daoyuan\r20220303002.txt';

[FileName2, RawData_table_DY, Time_DY] = DY_Analyze_V3(FileName);   %����Э�����н���
FileName = FileName2;

tic
RunMain(FileName, 0);   
toc
data_daoyuan = ans;


SaveEnable = 1;
idx_dash =  find(FileName=='\',1,'last');
SaveFileFolder = FileName(1:idx_dash);

%% ��figure����ȡ��Զ��־λ����
f = figure(12);
lf=findall(f.Children(4),'type','line');
% xf=get(lf,'xdata');
yf=get(lf,'ydata');

% a = [cell2mat(yf(1))', cell2mat(yf(2))', cell2mat(yf(3))', cell2mat(yf(4))'];

data_daoyuan.SelfDefine.V_R = cell2mat(yf(4))';
data_daoyuan.SelfDefine.P_R = cell2mat(yf(3))';
data_daoyuan.SelfDefine.R2_R = cell2mat(yf(2))';
data_daoyuan.SelfDefine.H2_R = cell2mat(yf(1))';

%% �洢����

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
    disp('�����ݴ��������λ�ã�')
    disp(save_name)
else
    disp('��������������û�б���')
end

%%


tt = data_daoyuan.INSData.ts - data_daoyuan.INSData.ts(1);
g = 9.78;
figure(101)
subplot(3,1,1)
plot(tt,ans.INSData.a(:,1)*g)
xlabel('x')
title('���ٶ�m/s^2')
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
title('���ٶȡ�/s')
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
