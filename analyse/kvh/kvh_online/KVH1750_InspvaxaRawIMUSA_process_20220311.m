% clc;
clear
close all
addpath(genpath('Functions'))
addpath(genpath('PlotFiles'))
set(groot,'defaultLineLineWidth',2)
%%

FileFolder = 'G:\data\KVH1750\Rover\20220111\';
FileName = 'BMAW18200029H_18_20220111_055056';  

SaveFileFolder = FileFolder;
if exist(SaveFileFolder(1:end-1),'dir')==0
   mkdir(SaveFileFolder(1:end-1));
end

save_name = strcat(SaveFileFolder,'Data_',FileName,'_new','.mat');
File2Open = strcat(FileFolder,FileName,'.ASC');                 %要读取的文档所在的路径  
fpn = fopen (File2Open, 'rt');           %打开文档  
delimiter = {',',';','*'};
str_INSPVAXA = '#INSPVAXA'; 
str_RAWIMU = '%RAWIMUSA';
i_INSPVAXA = 1;
i_RAWIMU = 1;
Gyro_Factor = 0.1/(3600.0*256.0) / 0.005; % rad/LSB
Acc_Factor = 0.05/2^15 / 0.005 ; % m/s/LSB

Data_max_Lenth = 1e6; % 超过数组所能存储最长的数据 停止存储，如果需要处理更长的数据，请增大Data_max_Lenth,目前9e7能处理8h的数据

time_INSPVAXA = zeros(Data_max_Lenth,1);
INS_Status_Fusion_KVH1750 = cell(Data_max_Lenth,1);
Pos_Type_Fusion_KVH1750 = cell(Data_max_Lenth,1);
Latitude_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Longitude_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Height_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Undulation_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Vel_N_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Vel_E_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Vel_U_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Roll_KVH1750 = zeros(Data_max_Lenth,1);
Pitch_KVH1750 = zeros(Data_max_Lenth,1);
Azimuth_KVH1750 = zeros(Data_max_Lenth,1);
Lat_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Long_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Height_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Vel_N_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Vel_E_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Vel_U_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Roll_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Pitch_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);
Azimuth_sigma_Fusion_KVH1750 = zeros(Data_max_Lenth,1);

time_RAWIMU = zeros(Data_max_Lenth,1);
Z_Acc_KVH1750_temp = zeros(Data_max_Lenth,1);
Y_Acc_KVH1750_temp = zeros(Data_max_Lenth,1);
X_Acc_KVH1750_temp = zeros(Data_max_Lenth,1);
Z_Gyro_KVH1750_temp = zeros(Data_max_Lenth,1);
Y_Gyro_KVH1750_temp = zeros(Data_max_Lenth,1);
X_Gyro_KVH1750_temp = zeros(Data_max_Lenth,1);

tic;
i = 0;
while feof(fpn) ~= 1                %用于判断文件指针p在其所指的文件中的位置，如果到文件末，函数返回1，否则返回0  
    file = fgetl(fpn);            %获取文档第一行  
    new_str = file;
    new_str(find(isspace(new_str))) = [];
    s = textscan(new_str,'%s','delimiter',delimiter);

    switch (s{1,1}{1,1})
        case str_INSPVAXA 
            if(length(s{1,1})~=34)
            else
                time_INSPVAXA(i_INSPVAXA,1) = str2double(s{1,1}{7,1});
                INS_Status_Fusion_KVH1750{i_INSPVAXA,1} = (s{1,1}{1+10,1});
                Pos_Type_Fusion_KVH1750{i_INSPVAXA,1} = (s{1,1}{2+10,1});
                Latitude_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{3+10,1});
                Longitude_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{4+10,1});
                Height_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{5+10,1});
                Undulation_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{6+10,1});
                Vel_N_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{7+10,1});
                Vel_E_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{8+10,1});
                Vel_U_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{9+10,1});
                Roll_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{10+10,1});
                Pitch_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{11+10,1});
                Azimuth_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{12+10,1});
                Lat_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{13+10,1});
                Long_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{14+10,1});
                Height_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{15+10,1});
                Vel_N_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{16+10,1});
                Vel_E_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{17+10,1});
                Vel_U_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{18+10,1});
                Roll_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{19+10,1});
                Pitch_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{20+10,1});
                Azimuth_sigma_Fusion_KVH1750(i_INSPVAXA,1) = str2double(s{1,1}{21+10,1});
                i_INSPVAXA = i_INSPVAXA + 1;
            end

       case str_RAWIMU 
            if(length(s{1,1})~=13)
            else
                time_RAWIMU(i_RAWIMU,1) = str2double(s{1,1}{5,1});
                Z_Acc_KVH1750_temp(i_RAWIMU,1) = str2double(s{1,1}{7,1});
                Y_Acc_KVH1750_temp(i_RAWIMU,1) = -str2double(s{1,1}{8,1});
                X_Acc_KVH1750_temp(i_RAWIMU,1) = str2double(s{1,1}{9,1});
                Z_Gyro_KVH1750_temp(i_RAWIMU,1) = str2double(s{1,1}{10,1});
                Y_Gyro_KVH1750_temp(i_RAWIMU,1) = -str2double(s{1,1}{11,1});
                X_Gyro_KVH1750_temp(i_RAWIMU,1) = str2double(s{1,1}{12,1});

                i_RAWIMU = i_RAWIMU + 1;
            end
            
        otherwise
    end
     
    i = i + 1;
 end  
fclose(fpn); 
time_INSPVAXA(i_INSPVAXA:end) = [];
INS_Status_Fusion_KVH1750(i_INSPVAXA:end) = [];
Pos_Type_Fusion_KVH1750(i_INSPVAXA:end) = [];
Pos_Type = Pos_Type_Cell2num(Pos_Type_Fusion_KVH1750);
INS_Status = INS_Status_Cell2num(INS_Status_Fusion_KVH1750);

Latitude_Fusion_KVH1750(i_INSPVAXA:end) = [];
Longitude_Fusion_KVH1750(i_INSPVAXA:end) = [];
Height_Fusion_KVH1750(i_INSPVAXA:end) = [];
Undulation_Fusion_KVH1750(i_INSPVAXA:end) = [];
Vel_N_Fusion_KVH1750(i_INSPVAXA:end) = [];
Vel_E_Fusion_KVH1750(i_INSPVAXA:end) = [];
Vel_U_Fusion_KVH1750(i_INSPVAXA:end) = [];
Roll_KVH1750(i_INSPVAXA:end) = [];
Pitch_KVH1750(i_INSPVAXA:end) = [];
Azimuth_KVH1750(i_INSPVAXA:end) = [];
Lat_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Long_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Height_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Vel_N_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Vel_E_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Vel_U_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Roll_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Pitch_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];
Azimuth_sigma_Fusion_KVH1750(i_INSPVAXA:end) = [];

time_RAWIMU(i_RAWIMU:end) = [];
Z_Acc_KVH1750_temp(i_RAWIMU:end) = [];
Y_Acc_KVH1750_temp(i_RAWIMU:end) = [];
X_Acc_KVH1750_temp(i_RAWIMU:end) = [];
Z_Gyro_KVH1750_temp(i_RAWIMU:end) = [];
Y_Gyro_KVH1750_temp(i_RAWIMU:end) = [];
X_Gyro_KVH1750_temp(i_RAWIMU:end) = [];

toc;


temp = sortrows([time_RAWIMU,Z_Acc_KVH1750_temp,Y_Acc_KVH1750_temp,X_Acc_KVH1750_temp,Z_Gyro_KVH1750_temp,Y_Gyro_KVH1750_temp,X_Gyro_KVH1750_temp],1);
time_RAWIMU = temp(:,1);
Z_Acc_KVH1750_temp = temp(:,2);
Y_Acc_KVH1750_temp = temp(:,3);
X_Acc_KVH1750_temp = temp(:,4);
Z_Gyro_KVH1750_temp = temp(:,5);
Y_Gyro_KVH1750_temp = temp(:,6);
X_Gyro_KVH1750_temp = temp(:,7);

temp = sortrows([time_INSPVAXA,Latitude_Fusion_KVH1750,Longitude_Fusion_KVH1750,Height_Fusion_KVH1750,Undulation_Fusion_KVH1750,...
                 Vel_N_Fusion_KVH1750,Vel_E_Fusion_KVH1750,Vel_U_Fusion_KVH1750,Roll_KVH1750,Pitch_KVH1750,Azimuth_KVH1750,...
                 Lat_sigma_Fusion_KVH1750,Long_sigma_Fusion_KVH1750,Height_sigma_Fusion_KVH1750,Vel_N_sigma_Fusion_KVH1750,...
                 Vel_E_sigma_Fusion_KVH1750,Vel_U_sigma_Fusion_KVH1750,Roll_sigma_Fusion_KVH1750,Pitch_sigma_Fusion_KVH1750,...
                 Azimuth_sigma_Fusion_KVH1750,Pos_Type,INS_Status],1);
             
time_INSPVAXA = temp(:,1);
Latitude_Fusion_KVH1750 = temp(:,2);
Longitude_Fusion_KVH1750 = temp(:,3);
Height_Fusion_KVH1750 = temp(:,4);
Undulation_Fusion_KVH1750 = temp(:,5);
Vel_N_Fusion_KVH1750 = temp(:,6);
Vel_E_Fusion_KVH1750 = temp(:,7);
Vel_U_Fusion_KVH1750 = temp(:,8);
Roll_KVH1750 = temp(:,9);
Pitch_KVH1750 = temp(:,10);
Azimuth_KVH1750 = temp(:,11);
Lat_sigma_Fusion_KVH1750 = temp(:,12);
Long_sigma_Fusion_KVH1750 = temp(:,13);
Height_sigma_Fusion_KVH1750 = temp(:,14);
Vel_N_sigma_Fusion_KVH1750 = temp(:,15);
Vel_E_sigma_Fusion_KVH1750 = temp(:,16);
Vel_U_sigma_Fusion_KVH1750 = temp(:,17);
Roll_sigma_Fusion_KVH1750 = temp(:,18);
Pitch_sigma_Fusion_KVH1750 = temp(:,19);
Azimuth_sigma_Fusion_KVH1750 = temp(:,20);
Pos_Type = temp(:,21);
INS_Status = temp(:,22);

   %%
   Z_Acc_KVH1750 = Z_Acc_KVH1750_temp * Acc_Factor;
   Y_Acc_KVH1750 = Y_Acc_KVH1750_temp * Acc_Factor;
   X_Acc_KVH1750 = X_Acc_KVH1750_temp * Acc_Factor;
   
   Z_Gyro_KVH1750 = Z_Gyro_KVH1750_temp * Gyro_Factor;
   Y_Gyro_KVH1750 = Y_Gyro_KVH1750_temp * Gyro_Factor;
   X_Gyro_KVH1750 = X_Gyro_KVH1750_temp * Gyro_Factor;
   
   t_min = min(time_INSPVAXA(1),time_RAWIMU(1));
   T_RAWIMU = time_RAWIMU - t_min;
   T_INSPVAXA = time_INSPVAXA - t_min;

%    T_RAWIMU = time_RAWIMU;
%    T_RAWIMU = time_RAWIMU - t_min_compare;
%    T_INSPVAXA = time_INSPVAXA - t_min_compare;
%     T_INSPVAXA = time_INSPVAXA;

    %% KVH1750.INSPVA
KVH1750.INSPVA = array2table([time_INSPVAXA,T_INSPVAXA,Latitude_Fusion_KVH1750,Longitude_Fusion_KVH1750,Height_Fusion_KVH1750,Undulation_Fusion_KVH1750,...
                 Vel_N_Fusion_KVH1750,Vel_E_Fusion_KVH1750,Vel_U_Fusion_KVH1750,Roll_KVH1750,Pitch_KVH1750,Azimuth_KVH1750,...
                 Lat_sigma_Fusion_KVH1750,Long_sigma_Fusion_KVH1750,Height_sigma_Fusion_KVH1750,Vel_N_sigma_Fusion_KVH1750,...
                 Vel_E_sigma_Fusion_KVH1750,Vel_U_sigma_Fusion_KVH1750,Roll_sigma_Fusion_KVH1750,Pitch_sigma_Fusion_KVH1750,...
                 Azimuth_sigma_Fusion_KVH1750,Pos_Type,INS_Status]);
KVH1750.INSPVA.Properties.VariableNames =...
    {'time_INSPVAXA','time_INSPVAXA_0','Latitude_Fusion','Longitude_Fusion','Height_Fusion','Undulation_Fusion',...
     'Vel_N_Fusion','Vel_E_Fusion','Vel_U_Fusion','Roll','Pitch','Azimuth',...
     'Lat_sigma_Fusion','Long_sigma_Fusion','Height_sigma_Fusion','Vel_N_sigma_Fusion',...
     'Vel_E_sigma_Fusion','Vel_U_sigma_Fusion','Roll_sigma_Fusion','Pitch_sigma_Fusion',...
     'Azimuth_sigma_Fusion','Pos_Type','INS_Status'};
     %% KVH1750.RAWIMU
KVH1750.RAWIMU = array2table([time_RAWIMU,T_RAWIMU,Z_Acc_KVH1750,Y_Acc_KVH1750,X_Acc_KVH1750,Z_Gyro_KVH1750,Y_Gyro_KVH1750,X_Gyro_KVH1750]);
KVH1750.RAWIMU.Properties.VariableNames = {'time_RAWIMU','T_RAWIMU_0','Z_Acc','Y_Acc','X_Acc','Z_Gyro','Y_Gyro','X_Gyro'};
 
%%
figure
    hold on 
    plot3(KVH1750.INSPVA.Longitude_Fusion,KVH1750.INSPVA.Latitude_Fusion,KVH1750.INSPVA.time_INSPVAXA_0,'.')    
    legend('FusionKVH')
    xlabel('经度');
    ylabel('纬度')
[INS_xyz_local] = llh2local_V2([KVH1750.INSPVA.Latitude_Fusion,KVH1750.INSPVA.Longitude_Fusion,KVH1750.INSPVA.Height_Fusion],[31.28,121.2,0]);

figure
plot(INS_xyz_local(:,1),INS_xyz_local(:,2),'.')
grid on
    legend('FusionKVH')
    xlabel('x');
    ylabel('y')

figure
    hold on
    plot(KVH1750.INSPVA.time_INSPVAXA_0,KVH1750.INSPVA.Lat_sigma_Fusion)
    plot(KVH1750.INSPVA.time_INSPVAXA_0,KVH1750.INSPVA.Long_sigma_Fusion)
    legend('Lat sigma Fusion','Long sigma Fusion')
    xlabel('时间 s')
    ylabel('误差 m')

figure
subplot(2,1,1)
    hold on
    plot(KVH1750.RAWIMU.T_RAWIMU_0,KVH1750.RAWIMU.X_Acc)
    plot(KVH1750.RAWIMU.T_RAWIMU_0,KVH1750.RAWIMU.Y_Acc)
    plot(KVH1750.RAWIMU.T_RAWIMU_0,KVH1750.RAWIMU.Z_Acc)
    legend('X Acc','Y Acc','Z Acc')
    xlabel('时间 s')
    ylabel('加速度 m/s^2')
subplot(2,1,2)
    hold on
    plot(KVH1750.RAWIMU.T_RAWIMU_0,KVH1750.RAWIMU.X_Gyro*180/pi)
    plot(KVH1750.RAWIMU.T_RAWIMU_0,KVH1750.RAWIMU.Y_Gyro*180/pi)
    plot(KVH1750.RAWIMU.T_RAWIMU_0,KVH1750.RAWIMU.Z_Gyro*180/pi)
    legend('X Gyro','Y Gyro','Z Gyro')
    xlabel('时间 s')
    ylabel('角速度 deg/s')
    
    
figure
    hold on
    TINSPVAXA_error = [0:length(KVH1750.INSPVA.time_INSPVAXA_0)-1]'*0.1 - KVH1750.INSPVA.time_INSPVAXA_0 - mean([0:length(KVH1750.INSPVA.time_INSPVAXA_0)-1]'*0.1 - KVH1750.INSPVA.time_INSPVAXA_0);
    plot(KVH1750.RAWIMU.T_RAWIMU_0, [0:length(KVH1750.RAWIMU.T_RAWIMU_0)-1]'*0.005 - KVH1750.RAWIMU.T_RAWIMU_0,'DisplayName','T RAWIMU error')
    plot(KVH1750.INSPVA.time_INSPVAXA_0, TINSPVAXA_error,'DisplayName','T INSPVAXA error')
    legend
    xlabel('时间 s')
    ylabel('时间误差 s')
    
figure
subplot(2,1,1)
    hold on
    plot(KVH1750.INSPVA.time_INSPVAXA_0,KVH1750.INSPVA.Pos_Type,'DisplayName','Pos\_Type')
    legend(['0 NONE No solution',newline,'16 single',newline, '17 PSRDIFF',newline,'19 PROPAGATED',newline,'49 NARROW\_FLOAT',newline,'50 NARROW\_INT',newline,'56:INS\_RTKFIXED'])
    xlabel('时间 s')
    ylabel('Pos\_Type')
subplot(2,1,2)
    hold on
    plot(KVH1750.INSPVA.time_INSPVAXA_0,KVH1750.INSPVA.INS_Status,'DisplayName','INS\_Status')
    legend(['0 INS\_INACTIVE',newline, '1 INS\_ALIGNING',newline,'2 INS\_ALIGNING',newline,'3 INS\_SOLUTION\_GOOD',newline,'6 INS\_SOLUTION\_FREE',...
        newline,'7 INS\_ALIGNMENT\_COMPLETE',newline,'8 DETERMINING\_ORIENTATION',newline,'9 WAITING\_INITIALPOS',...
        '10 WAITING\_AZIMUTH','11 INITIALIZING\_BIASES','12 MOTION_DETECT'])
    xlabel('时间 s')
    ylabel('INS\_Status')
   
%%
save(save_name,'KVH1750')
disp(save_name)
%% 清除变量 先用 who 获取变量
clear  Acc_Factor Longitude_Fusion_KVH1750 Vel_U_sigma_Fusion_KVH1750 fpn ...                    
Azimuth_KVH1750 Pitch_KVH1750 X_Acc_KVH1750 i ...
Azimuth_sigma_Fusion_KVH1750  Pitch_sigma_Fusion_KVH1750    X_Acc_KVH1750_temp            i_INSPVAXA   ...                
Data_max_Lenth  Pos_Type_Fusion_KVH1750       X_Gyro_KVH1750                i_RAWIMU         ...             
File2Open   Roll_KVH1750                  X_Gyro_KVH1750_temp           new_str         ...              
FileFolder   Roll_sigma_Fusion_KVH1750     Y_Acc_KVH1750                 s              ...               
FileName      SaveFileFolder                Y_Acc_KVH1750_temp            save_name     ...                
Gyro_Factor    TINSPVAXA_error               Y_Gyro_KVH1750                str_INSPVAXA    ...              
Height_Fusion_KVH1750  T_INSPVAXA                    Y_Gyro_KVH1750_temp           str_RAWIMU   ...                 
Height_sigma_Fusion_KVH1750  T_RAWIMU                      Z_Acc_KVH1750                 t_min  ...                       
INS_Status_Fusion_KVH1750     Undulation_Fusion_KVH1750     Z_Acc_KVH1750_temp            temp ...                         
INS_xyz_local   Vel_E_Fusion_KVH1750          Z_Gyro_KVH1750                time_INSPVAXA     ...            
Vel_E_sigma_Fusion_KVH1750    Z_Gyro_KVH1750_temp           time_RAWIMU       ...            
Lat_sigma_Fusion_KVH1750      Vel_N_Fusion_KVH1750          ans                         ...  
Latitude_Fusion_KVH1750       Vel_N_sigma_Fusion_KVH1750    delimiter                  ...   
Long_sigma_Fusion_KVH1750     Vel_U_Fusion_KVH1750          file  INS_Status Pos_Type