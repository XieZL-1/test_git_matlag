% clc;
clear;
close all;

%需要删除文件最后一行
filename = 'G:\data\UART_Data\20220303\Huace\20220303093833.hua';


SaveEnable = 1;

%% 数据解析
tic;
str_GPCHC = '$GPCHC';
str_GPGGA = '$GPGGA';
RM = 6382306;
RN = 6383783;
% i_GPCHC = 1;
% i_GPGGA = 1;
% i = 1;

fpn = fopen (filename, 'rt');           %打开文档
lines = 0;
while ~feof(fpn)
    fgetl(fpn);
    lines = lines +1;
end
fclose(fpn);

fpn = fopen (filename, 'rt');           %打开文档
s = textscan(fpn,['%s ',repmat('%f ',1,18), repmat('%d ',1,4), '%1d','%*1c', '%2s',],'delimiter',',');

switch (s{1,1}{1,1})
    case str_GPGGA
        
    case str_GPCHC
        time_GPCHC0 = s{1,3};
        Heading = (s{1,4});
        Pitch = (s{1,5});
        Roll = (s{1,6});
        gyro_x = (s{1,7});    %deg/s
        gyro_y = (s{1,8});
        gyro_z = (s{1,9});
        acc_x = (s{1,10});    %g
        acc_y = (s{1,11});
        acc_z = (s{1,12});
        Lattitude = (s{1,13});
        Longitude = (s{1,14});
        Altitude = (s{1,15});
        VE = (s{1,16});
        VN = (s{1,17});
        VU = (s{1,18});
        vehicle_speed = (s{1,19});
        SV = (s{1,20});
        SV2 = (s{1,21});
        
        temp = uint8((s{1,22}));
%         temp11 = [bitget(temp,8),bitget(temp,7),bitget(temp,6),bitget(temp,5),....
%                 bitget(temp,4),bitget(temp,3),bitget(temp,2),bitget(temp,1)];
%         Status_sys = bitget(temp,4) .* uint8(2.^3) + bitget(temp,3) .* uint8(2.^2) + bitget(temp,2) .* uint8(2.^1) + bitget(temp,1) .* uint8(2.^0);   %低半字节
%         Status_GNSS = bitget(temp,8) .* uint8(2.^3) + bitget(temp,7) .* uint8(2.^2) + bitget(temp,6) .* uint8(2.^1) + bitget(temp,5) .* uint8(2.^0);   %高半字节
        Status_sys = uint8(round(mod(temp,10)));
        Status_GNSS = (uint8(round(temp)) - Status_sys) / uint8(10);
        
        Age = (s{1,23});
        
        warning  = s{1,24};
        warning_GNSS = bitget(uint8(warning), 1*ones(size(warning)));
        warning_vehicle = bitget(uint8(warning), 2*ones(size(warning)));
        warning_gyro = bitget(uint8(warning), 3*ones(size(warning)));
        warning_acc = bitget(uint8(warning), 4*ones(size(warning)));
        
        CS = cell2mat(s{1,25});
    otherwise
end


if(lines> min([length(s{1,1}), length(s{1,end})])) 
    error('there is error in file line %d',min([length(s{1,1}), length(s{1,end})]));
end

fclose(fpn);
toc;



%%
time_GPCHC = time_GPCHC0 - time_GPCHC0(1);
%    time_GPGGA = time_GPGGA - time_GPGGA(1);

temp = [time_GPCHC;time_GPCHC(end)] - [0;time_GPCHC];
temp_id = find(temp>100);
for i = 1:length(temp_id)
    time_GPCHC(temp_id(i)-1) = time_GPCHC(temp_id(i)-2) + 0.1;
end
%% 数据提取
data_huace.time_GPCHC0 = time_GPCHC0;
data_huace.Heading = Heading;
data_huace.Pitch = Pitch;
data_huace.Roll = Roll;
data_huace.gyro_x = gyro_x;
data_huace.gyro_y = gyro_y;
data_huace.gyro_z = gyro_z;
data_huace.acc_x = acc_x;
data_huace.acc_y = acc_y;
data_huace.acc_z = acc_z;
data_huace.Lattitude = Lattitude;
data_huace.Longitude = Longitude;
data_huace.Altitude = Altitude;
data_huace.VE = VE;
data_huace.VN = VN;
data_huace.VU = VU;
data_huace.vehicle_speed = vehicle_speed;
data_huace.SV = double(SV);
data_huace.SV2 = double(SV2);
data_huace.Status_sys = double(Status_sys);
data_huace.Status_GNSS = double(Status_GNSS);
data_huace.Age = double(Age);
data_huace.warning_GNSS = double(warning_GNSS);
data_huace.warning_vehicle = double(warning_vehicle);
data_huace.warning_gyro = double(warning_gyro);
data_huace.warning_acc = double(warning_acc);

%% plot figures
figure(1)
hold on
plot(time_GPCHC,acc_x)
plot(time_GPCHC,acc_y,'r')
plot(time_GPCHC,acc_z,'g')
legend('x','y','z')
%     xlim([0,max(time_RAWIMUA)])
xlabel('时间 s');
ylabel('加速度 m/s^2')

figure(2)
hold on
plot(time_GPCHC,gyro_x)
plot(time_GPCHC,gyro_y,'r')
plot(time_GPCHC,gyro_z,'g')
legend('x','y','z')
%     xlim([0,max(time_RAWIMUA)])
xlabel('时间 s');
ylabel('角速度 deg/s')

figure(30)
hold on
plot3(Longitude,Lattitude,time_GPCHC,'.')
%     xlim([121.20,121.22])
%     ylim([31.290,31.292])
%     plot(Longitude_GPS,Latitude_GPS,'r .')
%     plot(Long_deg_GPRMC,Lat_deg_GPRMC,'g .')
%     plot(Long_deg_GPGGA,Lat_deg_GPGGA,'m .')
%     legend('Fusion','GPS','GPS_RMC','GPS_GGA')

%     legend('Fusion','GPS')
%     xlim([time(1),time(end)])
xlabel('经度 Longitude');
ylabel('纬度')

figure(4)
hold on
plot(time_GPCHC,VE)
plot(time_GPCHC,VN,'r')
xlabel('时间 s');
ylabel('速度 m/s')
legend('VE','VN')
%     ylim([-30,30]);
figure(5)
hold on
plot(time_GPCHC,Roll)
plot(time_GPCHC,Pitch,'r')
xlabel('时间 s');
ylabel('姿态角 deg/s')
legend('Roll','Pitch')
ylim([-10 10])
figure(6)
hold on
plot(time_GPCHC,Heading)
xlabel('时间 s');
ylabel('航向角 deg')
legend('航向角')
ylim([0 360])

% temp =  isnan(Status);
% Status(temp) = 0;
% [Status_H,Status_L] = bitand_array(Status);
% figure(7)
% subplot(2,1,1)
% plot(time_GPCHC,Status_H/16)
% subplot(2,1,2)
% plot(time_GPCHC,Status_L)
% xlabel('时间 s');
% ylabel('Status')
% legend('Status')

figure(7)
hold on
subplot(2,1,1)
plot(time_GPCHC,Status_GNSS)
xlabel('时间 s');
ylabel('Status_GNSS')
subplot(2,1,2)
plot(time_GPCHC,Status_sys)
xlabel('时间 s');
ylabel('Status_sys')


figure(9)
hold on
plot(time_GPCHC,Age)
xlabel('时间 s');
ylabel('Age')
legend('Age')


figure(10)
hold on
plot(time_GPCHC,warning_GNSS)
plot(time_GPCHC,warning_vehicle)
plot(time_GPCHC,warning_gyro)
plot(time_GPCHC,warning_acc)
xlabel('时间 s');
ylabel('warning')
legend('warning_GNSS','warning_vehicle','warning_gyro','warning_acc')

figure(11)
hold on
plot(time_GPCHC,SV)
plot(time_GPCHC,SV2)
xlabel('时间 s');
ylabel('卫星')
legend('SV','SV2')

%     %%
% figure(8)
%     hold on
%     plot(Longitude*RN*deg2rad(1),Lattitude*RM*deg2rad(1),'.')
% %      plot(Longitude(5635:5648)*RN*deg2rad(1),Lattitude(5635:5648)*RM*deg2rad(1),'m .')
%     xlabel('经度 Longitude');
%     ylabel('纬度')
%     grid minor
%     xscope = [min(Longitude*RN*deg2rad(1)):0.1:max(Longitude*RN*deg2rad(1))];
%     yscope = [min(Lattitude*RM*deg2rad(1)):0.1:max(Lattitude*RM*deg2rad(1))];
%     plot([xscope(1),xscope(end)],[yscope(end),yscope(end)],'r');
%     for i = 1:10
%         plot([xscope(1),xscope(end)],[yscope(end-i),yscope(end-i)],'r');
%     end

%     angle1 = atan((yscope(end) - yscope(1))/(xscope(end) - xscope(1)));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     angle1 = 0;
%     delta_angle = 2.25/57.3;
%     angle2 = angle1 - delta_angle;
%     for i = 20:30
%         L = ((xscope(end) - xscope(1))^2 + (yscope(end) - yscope(1))^2)^0.5;
%         plot([xscope(1),xscope(1)+L*cos(angle2)],[yscope(i),yscope(i)+L*sin(angle2)],'r');
%     end
%
%     k = 0;
%     for i = length(yscope)-10:length(yscope)
%         k = k + 1;
%
%         L = ((xscope(end) - xscope(1))^2 + (yscope(end) - yscope(1))^2)^0.5;
%         x1 = xscope(1);
%         xi(k) = xscope(1)+L*cos(angle2);
%
%         y1 = yscope(i)+4.8;
%         yi(k) = yscope(i)+L*sin(angle2)+4.8;
%         plot([x1,xi(k)],[y1,yi(k)],'r');
%     end
%
%
%
%
% %     plot([xscope(end),xscope(end)],[yscope(1),yscope(end)],'r');
% %     for i = 1:50
% %         plot([xscope(end-i),xscope(end-i)],[yscope(1),yscope(end)]);
% %     end

%% 保存
if SaveEnable
    save_name = [filename(1:end-4),'_Huace'];
    save(save_name, 'data_huace')
    disp('将数据存放在如下位置：')
    disp(save_name)
else
    disp('这组解析后的数据没有保存')
end