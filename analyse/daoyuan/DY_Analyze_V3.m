function [FileName2, RawData_table_DY, Time_DY] = DY_Analyze_V3(FileName)
% clear;
% %clc;
% close all;
% addpath('Functions')
%% 需要修改的部分


% FileName = 'G:\data\UART_Data\20220325\Daoyuan\r20220325012.txt';

% SaveEnable = 1;
% idx_dash =  find(FileName=='\',1,'last');
% SaveFileFolder = FileName(1:idx_dash);

% Start_T = 1;
% End_T = inf;
%% 以下无需修改
tic
disp(FileName);
fid = fopen(FileName,'rb');
data = fread(fid,'uint8');
fclose(fid);

% 找出INS数据头，进行数据筛选
idx0 = strfind(reshape(data,1,[]), uint8([189, 219, 11]));
if idx0(end) > length(data)-62
    idx0(end) = [];
end
datanum = zeros(length(idx0),1);
count = 0;
for i = 1:length(idx0)   %协议长度要求
        xor_temp = 0;
        for j=1:62   %协议长度要求
            xor_temp = bitxor(xor_temp,data(idx0(i)+j-1));
        end
        if xor_temp==data(idx0(i)+62)
            count = count+1;
            datanum(count) = idx0(i);
        end
end
datanum(count:end)=[];
    


data_DY = zeros(count-1,60);
for i = 1:count-1
    data_DY(i,:) = data((datanum(i)+3):(datanum(i)+62))';
end


toc

% fid = fopen(FileName2,'rb');
%% 数据解析

data_DY_INS = zeros(count-1,33);
data_DY_INS_temp = zeros(count-1,22);

% 数据拼接
% 横滚角\俯仰角\方位角 % 陀螺仪  % 加速度
for i=1:9
    data_DY_INS_temp(:,i) = data_DY(:,i*2-1) + data_DY(:,i*2) * 2^8;
end

% 经纬高
data_DY_INS_temp(:,10) = data_DY(:,19) + data_DY(:,20) * 2^8 + data_DY(:,21) * 2^16 +data_DY(:,22) * 2^24;
data_DY_INS_temp(:,11) = data_DY(:,23) + data_DY(:,24) * 2^8 + data_DY(:,25) * 2^16 +data_DY(:,26) * 2^24;
data_DY_INS_temp(:,12) = data_DY(:,27) + data_DY(:,28) * 2^8 + data_DY(:,29) * 2^16 +data_DY(:,30) * 2^24;

% 速度
for i=13:15
    data_DY_INS_temp(:,i) = data_DY(:,i*2+5) + data_DY(:,i*2+6) * 2^8;
end

% 状态
data_DY_INS_temp(:,16) = data_DY(:,37);

% 轮询数据
for i=17:19
    data_DY_INS_temp(:,i) = data_DY(:,i*2+10) + data_DY(:,i*2+11) * 2^8;
end

% 时间
data_DY_INS_temp(:,20) = data_DY(:,50)+ data_DY(:,51) * 2^8 + data_DY(:,52) * 2^16 +data_DY(:,53) * 2^24;

% Type
data_DY_INS_temp(:,21) = data_DY(:,54);

% GPS周
data_DY_INS_temp(:,22) = data_DY(:,56) + data_DY(:,57) * 2^8 + data_DY(:,58) * 2^16 +data_DY(:,59) * 2^24;


% 类型转换
% 姿态角
data_DY_INS(:,1) = double(typecast(uint16(data_DY_INS_temp(:,1)), 'int16'))* 360/32768;
data_DY_INS(:,2) = double(typecast(uint16(data_DY_INS_temp(:,2)), 'int16'))* 360/32768;
data_DY_INS(:,3) = double(typecast(uint16(data_DY_INS_temp(:,3)), 'int16'))* 360/32768;

% 陀螺仪
data_DY_INS(:,4) = double(typecast(uint16(data_DY_INS_temp(:,4)), 'int16'))* 300/32768;
data_DY_INS(:,5) = double(typecast(uint16(data_DY_INS_temp(:,5)), 'int16'))* 300/32768;
data_DY_INS(:,6) = double(typecast(uint16(data_DY_INS_temp(:,6)), 'int16'))* 300/32768;

% 加速度计
data_DY_INS(:,7) = double(typecast(uint16(data_DY_INS_temp(:,7)),'int16'))* 12/32768;
data_DY_INS(:,8) = double(typecast(uint16(data_DY_INS_temp(:,8)),'int16'))* 12/32768;
data_DY_INS(:,9) = double(typecast(uint16(data_DY_INS_temp(:,9)),'int16'))* 12/32768;

% 经纬高度
data_DY_INS(:,10) = double(typecast(uint32(data_DY_INS_temp(:,10)),'int32'))* 1.00E-07;
data_DY_INS(:,11) = double(typecast(uint32(data_DY_INS_temp(:,11)),'int32'))* 1.00E-07;
data_DY_INS(:,12) = double(typecast(uint32(data_DY_INS_temp(:,12)),'int32'))* 1.00E-03;

% 速度
data_DY_INS(:,13) = double(typecast(uint16(data_DY_INS_temp(:,13)),'int16'))* 1e2/32768;
data_DY_INS(:,14) = double(typecast(uint16(data_DY_INS_temp(:,14)),'int16'))* 1e2/32768;
data_DY_INS(:,15) = double(typecast(uint16(data_DY_INS_temp(:,15)),'int16'))* 1e2/32768;

% 初始化状态
data_DY_INS(:,16) = data_DY_INS_temp(:,16);

% 时间
data_DY_INS(:,17) = data_DY_INS_temp(:,20)*0.25;

% GPS周
data_DY_INS(:,18) = data_DY_INS_temp(:,22);

% 轮询数据Type
data_DY_INS(:,19) = data_DY_INS_temp(:,21);

% 轮询数据Data1-3
data_DY_INS_temp(:,17) = double(typecast(uint16(data_DY_INS_temp(:,17)),'int16'));
data_DY_INS_temp(:,18) = double(typecast(uint16(data_DY_INS_temp(:,18)),'int16'));
data_DY_INS_temp(:,19) = double(typecast(uint16(data_DY_INS_temp(:,19)),'int16'));

pos_std = zeros(count-1,3);
v_std = zeros(count-1,3);
att_std = zeros(count-1,3);
TEMP = zeros(count-1,1);
GPS_Type = zeros(count-1,3);
Wheelspeed_Type = zeros(count-1,1);

for j=1:count-1-60
    switch data_DY_INS_temp(j,21)
        case 0
            pos_std(j:j+59,1:3) = ones(60,1)*exp(data_DY_INS_temp(j,17:19)/100);
        case 1
            v_std(j:j+59,1:3) = ones(60,1)*exp(data_DY_INS_temp(j,17:19)/100);
        case 2
            att_std(j:j+59,1:3) = ones(60,1)*exp(data_DY_INS_temp(j,17:19)/100);
        case 22
            TEMP(j:j+59) = ones(60,1)*data_DY_INS_temp(j,17)*200/32768;
        case 32
            GPS_Type(j:j+59,1:3) = ones(60,1)*data_DY_INS_temp(j,17:19);
        case 33
            Wheelspeed_Type(j:j+59) = ones(60,1)*data_DY_INS_temp(j,18);
    end
end

data_DY_INS(:,20:33) = [pos_std,v_std,att_std,TEMP,GPS_Type,Wheelspeed_Type];


% RawData_table 写入数据
% End_T_num = min(End_T*100+1,length(data_DY_INS));
% RawData_table_DY=array2table(data_DY_INS(Start_T*100+1:End_T_num,:));
RawData_table_DY=array2table(data_DY_INS );

RawData_table_DY.Properties.VariableNames ={'Roll_deg', 'Pitch_deg', 'Azimuth_deg', 'Gyro_x', 'Gyro_y', ...%每行五个，便于计数
    'Gyro_z', 'Acc_x', 'Acc_y', 'Acc_z', 'Latitude',...
    'Longitude', 'Height','VN','VE','VD',  ...
    'INS_Status', 'GNSS_Time','GNSS_Week','Type','lat_std',...
    'long_std','h_std','vn_std','ve_std','vd_std',...
    'roll_std','pitch_std','yaw_std','TEMP','Pos_Type',...
    'Num_SV','Heading_Type','Wheelspeed_Type'};


Flag = dec2bin(RawData_table_DY.INS_Status);
start_flag = length(data_DY_INS);
for i = 1:size(data_DY_INS,1)
    if sum(Flag(i,end-3:end) == '1') ==4
        start_flag = i;
        break
    end
end
clear Flag

% Time_DY = [0:length(data_DY_INS)-1]'*0.01  - (start_flag-1)*0.01;
Time_DY = RawData_table_DY.GNSS_Time(start_flag:end) + (RawData_table_DY.GNSS_Week(start_flag:end) - RawData_table_DY.GNSS_Week(start_flag))*7*24*3600 - RawData_table_DY.GNSS_Time(start_flag);
Time_DY = [([1:start_flag-1]' - start_flag)*0.01; Time_DY*0.001];
%% 数据存储

%剔除非初始化段数据，否则导远解析代码容易报错
ra = start_flag/length(data_DY_INS);
data = data(ceil(length(data)*ra):end);

% %把洗过的数据重新存为bin文件
FileName2 = [FileName(1:end-4),'_1',FileName(end-3:end)]; %导出的临时文件名
disp(FileName2);
fid = fopen(FileName2,'wb');
fwrite(fid,data,'uint8');
fclose(fid);


% if SaveEnable
%     if exist(SaveFileFolder(1:end-1),'dir')==0
%         mkdir(SaveFileFolder(1:end-1));
%     end
% end
% 
% % 存储名字需要根据数据修改
% if SaveEnable
% %     data_num = FileName(length(FileName)-11:length(FileName)-4);
%     name_temp = FileName(idx_dash+1:length(FileName)-4);
%     save_name = strcat(SaveFileFolder,'DY_',name_temp);
% end
% 
% if SaveEnable
%     save(save_name,'RawData_table_DY','Time_DY');
%     disp(save_name)
% end

%% 作图
set(groot,'defaultLineLineWidth',1.5);

Pos_ratio_longi = 5.519e6*deg2rad(1);
Pos_ratio_lati = 6.3515e6*deg2rad(1);
pos_local_x0 = 1.212088647000000e+02;%Pos_long_9250_4.signals.values(1,1);
pos_local_y0 = 31.289691800000000;%Pos_lat_9250_4.signals.values(1,1);

Latitude = RawData_table_DY.Latitude;
Longitude = RawData_table_DY.Longitude;

Local_x = (Longitude- pos_local_x0)*Pos_ratio_longi;
Local_y = ( Latitude- pos_local_y0)*Pos_ratio_lati;

f_figure(301,'横滚角')
hold on
grid on
plot(Time_DY,RawData_table_DY.Roll_deg,'DisplayName','Roll');
legend
xlabel('时间 s')
ylabel('横滚角 deg')

f_figure(302,'俯仰角')
hold on
grid on
plot(Time_DY,RawData_table_DY.Pitch_deg,'DisplayName','Pitch');
legend
xlabel('时间 s')
ylabel('俯仰角 deg')

f_figure(303,'方位角')
hold on
grid on
plot(Time_DY,RawData_table_DY.Azimuth_deg,'DisplayName','Azimuth');
legend
xlabel('时间 s')
ylabel('方位角 deg')

f_figure(304,'角速度 加速度')
axe1 = subplot(2,1,1);
hold on
grid on
plot(Time_DY,RawData_table_DY.Gyro_x,'DisplayName','x');
plot(Time_DY,RawData_table_DY.Gyro_y,'DisplayName','y');
plot(Time_DY,RawData_table_DY.Gyro_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
axe2 = subplot(2,1,2);
hold on
grid on
plot(Time_DY,RawData_table_DY.Acc_x,'DisplayName','x');
plot(Time_DY,RawData_table_DY.Acc_y,'DisplayName','y');
plot(Time_DY,RawData_table_DY.Acc_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('加速度 m/s^2')
linkaxes([axe1,axe2],'x')

f_figure(305,'位置-局部坐标系') %画平面坐标m图
hold on
grid on
plot3(Local_x,Local_y,Time_DY,'.','DisplayName','INS local');
legend
xlabel('局部坐标 x m')
ylabel('局部坐标 y m')

f_figure(306,'经纬度') %画经纬度图
hold on
grid on
plot3(Longitude,Latitude,Time_DY,'.','DisplayName','INS');
legend
xlabel('经度 deg')
ylabel('纬度 deg')

f_figure(307,'高度')
hold on
grid on
plot(Time_DY,RawData_table_DY.Height,'DisplayName','Height');
legend
xlabel('时间 s')
ylabel('高度 m')

f_figure(308,'速度')
axe1 = subplot(3,1,1);
hold on
grid on
plot(Time_DY,RawData_table_DY.VN,'DisplayName','VN');
legend
xlabel('时间 s')
ylabel('速度 m/s')
axe2 = subplot(3,1,2);
hold on
grid on
plot(Time_DY,RawData_table_DY.VE,'DisplayName','VE');
legend
xlabel('时间 s')
ylabel('速度 m/s')
axe3 = subplot(3,1,3);
hold on
grid on
plot(Time_DY,RawData_table_DY.VD,'DisplayName','VD');
legend
xlabel('时间 s')
ylabel('速度 m/s')
linkaxes([axe1,axe2,axe3],'x')

% 定位状态分析
Flag = dec2bin(RawData_table_DY.INS_Status);
pos_flag = Flag(:,end)>'0';
vel_flag = Flag(:,end-1)>'0';
att_flag = Flag(:,end-2)>'0';
heading_flag = Flag(:,end-3)>'0';
clear Flag
f_figure(309,'定位状态')
hold on
grid on
plot(Time_DY,pos_flag,'DisplayName','Pos_Flag');
plot(Time_DY,vel_flag,'DisplayName','Vel_Flag');
plot(Time_DY,att_flag,'DisplayName','Att_Flag');
plot(Time_DY,heading_flag,'DisplayName','Heading Flag');
legend
xlabel('时间 s')
ylabel('定位状态')

f_figure(3010,'GPS时间')
hold on
grid on
plot(Time_DY,RawData_table_DY.GNSS_Time,'DisplayName','GNSS Time');
legend
xlabel('时间 s')
ylabel('GPS时间 ms')

f_figure(3110,'diff GPS时间')
hold on
grid on
plot(Time_DY(1:end-1),diff(RawData_table_DY.GNSS_Time),'DisplayName','diff GNSS Time');
legend
xlabel('时间 s')
ylabel('GPS时间 ms')

f_figure(3011,'GPS周')
hold on
grid on
plot(Time_DY,RawData_table_DY.GNSS_Week,'DisplayName','GNSS Week');
legend
xlabel('时间 s')
ylabel('GPS周数')

f_figure(3012,'Pos std')
axe1 = subplot(3,1,1);
hold on
grid on
plot(Time_DY,RawData_table_DY.lat_std,'DisplayName','lat std');
plot(Time_DY,RawData_table_DY.long_std,'DisplayName','long std');
plot(Time_DY,RawData_table_DY.h_std,'DisplayName','h std');
axis([ 0, Time_DY(end), 0, 10,])
legend
xlabel('时间 s')
ylabel('Pos std')

axe2 = subplot(3,1,2);
hold on
grid on
plot(Time_DY,pos_flag,'DisplayName','pos flag');
legend
xlabel('时间 s')
ylabel('Pos flag')

axe3 = subplot(3,1,3);
hold on
grid on
plot(Time_DY,RawData_table_DY.Pos_Type,'DisplayName','pos type');
legend
xlabel('时间 s')
ylabel('Pos Type')
linkaxes([axe1,axe2,axe3],'x')

f_figure(3013,'V std')
axe1 = subplot(2,1,1);
hold on
grid on
plot(Time_DY,RawData_table_DY.vn_std,'DisplayName','vn std');
plot(Time_DY,RawData_table_DY.ve_std,'DisplayName','ve std');
plot(Time_DY,RawData_table_DY.vd_std,'DisplayName','vd std');
legend
xlabel('时间 s')
ylabel('V std')

axe2 = subplot(2,1,2);
hold on
grid on
plot(Time_DY,vel_flag,'DisplayName','vel flag');
legend
xlabel('时间 s')
ylabel('Vel flag')
linkaxes([axe1,axe2],'x')

f_figure(3014,'Att std')
axe1 = subplot(3,1,1);
hold on
grid on
plot(Time_DY,RawData_table_DY.roll_std,'DisplayName','roll std');
plot(Time_DY,RawData_table_DY.pitch_std,'DisplayName','pitch std');
plot(Time_DY,RawData_table_DY.yaw_std,'DisplayName','yaw std');
legend
xlabel('时间 s')
ylabel('Att std')

axe2 = subplot(3,1,2);
hold on
grid on
plot(Time_DY,att_flag,'DisplayName','att flag');
legend
xlabel('时间 s')
ylabel('Att flag')

axe3 = subplot(3,1,3);
hold on
grid on
plot(Time_DY,RawData_table_DY.Heading_Type,'DisplayName','Heading Type');
legend
xlabel('时间 s')
ylabel('Heading Type')
linkaxes([axe1,axe2,axe3],'x')

f_figure(3015,'温度')
hold on
grid on
plot(Time_DY,RawData_table_DY.TEMP,'DisplayName','TEMP');
legend
xlabel('时间 s')
ylabel('Temp °C')

f_figure(3016,'定位状态')
axe1 = subplot(3,1,1);
hold on
grid on
plot(Time_DY,RawData_table_DY.Pos_Type,'DisplayName','Pos Type');
plot(Time_DY,RawData_table_DY.Heading_Type,'DisplayName','Heading Type');
legend
xlabel('时间 s')
ylabel('Pos&Heading Type')

axe2 = subplot(3,1,2);
hold on
grid on
plot(Time_DY,RawData_table_DY.Num_SV,'DisplayName','卫星个数');
legend
xlabel('时间 s')
ylabel('卫星个数')

axe3 = subplot(3,1,3);
hold on
grid on
plot(Time_DY,pos_flag,'DisplayName','Pos flag');
legend
xlabel('时间 s')
ylabel('位置初始化状态')
linkaxes([axe1,axe2,axe3],'x')


%% test
% gx = RawData_table_DY.Gyro_x(2.75e4:3.08e4);
% gy = RawData_table_DY.Gyro_y(2.75e4:3.08e4);
% gz = RawData_table_DY.Gyro_z(2.75e4:3.08e4);
% ax = RawData_table_DY.Acc_x(2.75e4:3.08e4);
% ay = RawData_table_DY.Acc_y(2.75e4:3.08e4);
% az = RawData_table_DY.Acc_z(2.75e4:3.08e4);
% tt = Time_DY(2.75e4:3.08e4);
% 
% dt = 0.5 *100;
% 
% std_gx = zeros(floor(length(gx)/dt),1);
% std_gy = zeros(floor(length(gx)/dt),1);
% std_gz = zeros(floor(length(gx)/dt),1);
% std_g_all = zeros(floor(length(gx)/dt),1);
% 
% std_ax = zeros(floor(length(gx)/dt),1);
% std_ay = zeros(floor(length(gx)/dt),1);
% std_az = zeros(floor(length(gx)/dt),1);
% std_a_all = zeros(floor(length(gx)/dt),1);
% 
% tt_std = tt(1:dt:end-1);
% 
% for i = 1:length(std_gx)
%     std_gx(i) = std(gx((i-1)*dt+1 : i*dt));
%     std_gy(i) = std(gy((i-1)*dt+1 : i*dt));
%     std_gz(i) = std(gz((i-1)*dt+1 : i*dt));
%     std_g_all(i) = std( (gx((i-1)*dt+1 : i*dt).^2 + gy((i-1)*dt+1 : i*dt).^2 + gz((i-1)*dt+1 : i*dt).^2).^0.5 );
%     
%     std_ax(i) = std(ax((i-1)*dt+1 : i*dt));
%     std_ay(i) = std(ay((i-1)*dt+1 : i*dt));
%     std_az(i) = std(az((i-1)*dt+1 : i*dt));
%     std_a_all(i) = std( (ax((i-1)*dt+1 : i*dt).^2 + ay((i-1)*dt+1 : i*dt).^2 + az((i-1)*dt+1 : i*dt).^2).^0.5 );
% end
% 
% figure
% hold on
% plot(tt_std, std_gx);
% plot(tt_std, std_gy);
% plot(tt_std, std_gz);
% plot(tt_std, std_g_all);
% 
% figure
% hold on
% plot(tt_std, std_ax);
% plot(tt_std, std_ay);
% plot(tt_std, std_az);
% plot(tt_std, std_a_all);
