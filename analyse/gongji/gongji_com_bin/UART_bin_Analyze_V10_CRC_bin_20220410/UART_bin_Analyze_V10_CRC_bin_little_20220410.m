%% 采用全新的基于二进制的接口协议
clear;
% % clc;
close all;
set(groot,'defaultLineLineWidth',2)

%% 需要修改的部分

% FileName= 'G:\data\temp\data20220311\data_20220316_122254_00.bin';%
% FileName= 'G:\data\temp\20220401\20220401191732.TXT';%
FileName= 'G:\data\temp\20220408\20220413152824.bin';%

leverfile_enable = 1;
CRC_test = 1;   %是否进行CRC校验 1是需要 0不校验
SaveEnable = 1;
idx_dash =  find(FileName=='\',1,'last');
SaveFileFolder = FileName(1:idx_dash);
g = 9.7964;     %上海重力加速度


%% com协议设置
bytelenth = 164;    %com口协议字节长度


byte_order0 = [1, 5, 9, 13, 17, 19, 21, 23, 25, 27, 29, 30, 32, 36, 37, 38, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, ...
                71, 72, 74, 76, 78, 80, 82, 84, 86, 90, 91, 92, 93, 94, 95, 96, 97, 101, 102, 106, 107, 111, 115, 117, 119, 121, 123, 125, ...
                127, 129, 131, 133, 134, 135, 136, 140, 141, 142, 143, 145, 147, 149, 151, 153, 155, 157, 159, 160, 161, 162, 163, 164, ]; 
byte_order1_1 = [ 91     93  95 ];   %轮询数据表
byte_order1_2 = [ 91     93  95 ];   %轮询数据表
byte_order1_3 = [ 91     93  95 ];   %轮询数据表
byte_order1_4 = [ 91     93  95 ];   %轮询数据表
byte_order1_5 = [ 91     93  95 ];   %轮询数据表
byte_order1_6 = [ 91     93  95 ];   %轮询数据表
byte_order1_7 = [ 91     93  95 ];   %轮询数据表
        
byte_num0 = [4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 1, 2, 4, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 4, 1, ...
            1, 1, 1, 1, 1, 1, 4, 1, 4, 1, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 4, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, ];
byte_num1_1 = [2    2   2 ];   %轮询数据表
byte_num1_2 = [2    2   2 ];   %轮询数据表
byte_num1_3 = [2    2   2 ];   %轮询数据表
byte_num1_4 = [2    2   2 ];   %轮询数据表
byte_num1_5 = [2    2   2 ];   %轮询数据表
byte_num1_6 = [2    2   2 ];   %轮询数据表 
byte_num1_7 = [2    2   2 ];   %轮询数据表

%字头放在uint32里        
byte_type0 =  {'uint32', 'int32', 'int32', 'int32', 'int16', 'int16', 'int16', 'int16', 'int16', 'int16', 'uint8', 'uint16', 'uint32', 'uint8', ...
            'uint8', 'uint8', 'int16', 'int16', 'int16', 'int16', 'int16', 'int16', 'int16', 'uint16', 'uint16', 'uint16', 'uint16', 'uint16', ...
            'uint16', 'uint16', 'uint16', 'uint16', 'uint8', 'int16', 'int16', 'int16', 'int16', 'int16', 'int16', 'int16', 'uint32', 'uint8', ...
            'uint8', 'uint8', 'uint8', 'uint8', 'uint8', 'uint8', 'uint32', 'int8', 'int32', 'int8', 'int32', 'int32', 'int16', 'int16', 'int16', ...
            'int16', 'int16', 'uint16', 'uint16', 'uint16', 'uint16', 'uint8', 'uint8', 'uint8', 'uint32', 'uint8', 'uint8', 'uint8', 'uint16', ...
            'uint16', 'uint16', 'uint16', 'uint16', 'uint16', 'uint16', 'uint16', 'uint8', 'uint8', 'uint8', 'uint8', 'uint8', 'uint8', };
byte_type1_1 =  {'int16','int16','int16' };     %轮询数据表       
byte_type1_2 =  {'int16','int16','int16' };     %轮询数据表       
byte_type1_3 =  {'int16','int16','int16' };     %轮询数据表       
byte_type1_4 =  {'int16','int16','int16' };     %轮询数据表       
byte_type1_5 =  {'uint16','uint16','uint16' };     %轮询数据表       
byte_type1_6 =  {'uint16','uint16','uint16' };     %轮询数据表      
byte_type1_7 =  {'uint16','uint16','uint16' };     %轮询数据表    

byte_factor0 = [1, 1e-7, 1e-7, 1e-3, 1e2/2^15, 1e2/2^15, 1e2/2^15, 180/2^15, 180/2^15, 180/2^15, 1, 1, 1e-3, 1, 1, 1, 8/2^15, 8/2^15, 8/2^15, ...
            1e2/2^15, 1e2/2^15, 1e2/2^15, 150/2^15, 100/2^16, 100/2^16, 100/2^16, 100/2^16, 100/2^16, 100/2^16, 100/2^16, 100/2^16, 100/2^16, ...
            1, 150/2^15, 8/2^15, 8/2^15, 8/2^15, 1e2/2^15, 1e2/2^15, 1e2/2^15, 1, 1, 1, 1, 1, 1, 1, 1, 1, 10, 10/2^31, 10, 10/2^31, 1e-3, ...
            1e2/2^15, 1e2/2^15, 1e2/2^15, 180/2^15, 180/2^15, 32/2^16, 32/2^16, 32/2^16, 32/2^16, 1, 1, 1, 1e-3, 1, 1, 1, 1e2/2^16, 1e2/2^16, ...
            1e2/2^16, 1e2/2^16, 1e2/2^16, 720/2^16, 720/2^16, 1024/2^16, 1/2^8, 1, 1, 1, 1, 1, ];
byte_factor1_1 = [180/2^15, 180/2^15, 180/2^15];     %轮询数据表  
byte_factor1_2 = [20/2^15, 20/2^15, 20/2^15];     %轮询数据表  
byte_factor1_3 = [20/2^15, 20/2^15, 20/2^15];     %轮询数据表  
byte_factor1_4 = [180/2^15, 180/2^15, 180/2^15];     %轮询数据表  
byte_factor1_5 = [1, 1, 1];          %轮询数据表  
byte_factor1_6 = [1/2^15, 1/2^15, 1/2^15];     %轮询数据表  
byte_factor1_7 = [10/2^15, 10/2^15, 10/2^15];     %轮询数据表  
        
% 协议设置检查
if length(byte_order0) ~= length(byte_num0) || length(byte_order0) ~= length(byte_type0) || length(byte_order0) ~= length(byte_factor0) 
    error('Error occurred in COM protocol setting.');
end

%% 以下无需修改
disp(FileName);
fid = fopen(FileName,'rb');
data_temp=uint8(fread(fid,'uint8'));
fclose(fid);

% 上位机保存标志位
% i =1;
% if ( data_temp(i)==67 && data_temp(i+1)==79 && data_temp(i+2)==77 && data_temp(i+3)==80 && data_temp(i+4)==85 && data_temp(i+5)==84 && data_temp(i+6)==69 && data_temp(i+7)==82)
if ( sum(double(reshape(data_temp(1:8),1,[]))-double('COMPUTER')) ==0)
    data_temp(1:8)=[];
    FLAG_computer = 1;
else
    FLAG_computer = 0;
end

if FLAG_computer == 0
%% 按消息头去除错误数据
tic
    idx0 = strfind(reshape(data_temp,1,[]), uint8([hex2dec('AA'),hex2dec('44'),hex2dec('AA'),hex2dec('45')]));
    error0 = sum(uint16(diff(idx0))~=uint16(bytelenth));
    idx0 = idx0(uint16(diff(idx0))>=uint16(bytelenth));
    idx1 = zeros(bytelenth,size(idx0,2));
    for i = 1:bytelenth
        idx1(i,:) = idx0+i-1;
    end
    data_temp = data_temp(idx1);
    data_temp=data_temp';
    toc
%% CRC校验
    if CRC_test == 1
        crc_1 = uint8(data_temp(:,71));%CRC存储
        crc_2 = uint8(data_temp(:,164));%CRC存储
        data1 = uint8(data_temp(:,1:70));%数据存储
        data2 = uint8(data_temp(:,72:163));%数据存储
        crc_1_1 = cellstr(dec2hex(crc_1,2));
        crc_2_2 = cellstr(dec2hex(crc_2,2));
        
        bool_1 =  CRC_mexupdate_0109(data1,crc_1_1,70);
        all_1 = find(bool_1==0);
        bool_2 =  CRC_mexupdate_0109(data2,crc_2_2,92);
        all_2 = find(bool_2==0);  
        
        wrong = 0;
        if isempty(all_1)
            disp('非用户数据全部错误，请检查数据格式重新解析')
            all_error_1 = bool_1(1:end); error_1 = length(all_error_1); 
        else
            all_error_1 = bool_1(1:all_1(1)-1);error_1 = length(all_error_1);
        end
        if isempty(all_2)
            disp('用户数据全部错误，请检查数据格式重新解析')
            all_error_2 = bool_2(1:end);error_2 = length(all_error_2);
        else
            all_error_2 = bool_2(1:all_2(1)-1);error_2 = length(all_error_2);
        end
        
        alldata = length(data_temp(:,1));
        fprintf('Number of all data = %d\n',alldata);
        fprintf('Number of wrong non-users data = %d\n',error_1);
        fprintf('Number of wrong users'' data = %d\n',error_2);
        data_temp([all_error_1;all_error_2] , :) = [];
        fprintf('Number of all wrong data = %d\n',alldata - length(data_temp(:,1)));        
    end
    
    %% 字节数据解析
    data_temp1 = zeros(size(data_temp,1), length(byte_order0));
    for i = 1: length(byte_order0)
        data_temp1(:,i) = double(typecast(reshape(data_temp(:,byte_order0(i):byte_order0(i)+byte_num0(i)-1)',1,[]), byte_type0{i}))' * byte_factor0(i);
    end
    data_temp1_1 = zeros(length(find(data_temp1(:,42)==1)), length(byte_order1_1));
    data_temp1_2 = zeros(length(find(data_temp1(:,42)==2)), length(byte_order1_2));
    data_temp1_3 = zeros(length(find(data_temp1(:,42)==3)), length(byte_order1_3));
    data_temp1_4 = zeros(length(find(data_temp1(:,42)==4)), length(byte_order1_4));
    data_temp1_5 = zeros(length(find(data_temp1(:,42)==5)), length(byte_order1_5));
    data_temp1_6 = zeros(length(find(data_temp1(:,42)==6)), length(byte_order1_6));
    data_temp1_7 = zeros(length(find(data_temp1(:,42)==7)), length(byte_order1_7));
    for i = 1: length(byte_order1_1)
       	data_temp1_1(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==1,byte_order1_1(i):byte_order1_1(i)+byte_num1_1(i)-1)',1,[]), byte_type1_1{i}))' * byte_factor1_1(i);
    end
    for i = 1: length(byte_order1_2)
       	data_temp1_2(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==2,byte_order1_2(i):byte_order1_2(i)+byte_num1_2(i)-1)',1,[]), byte_type1_2{i}))' * byte_factor1_2(i);
    end
    for i = 1: length(byte_order1_3)
       	data_temp1_3(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==3,byte_order1_3(i):byte_order1_3(i)+byte_num1_3(i)-1)',1,[]), byte_type1_3{i}))' * byte_factor1_3(i);
    end
    for i = 1: length(byte_order1_4)
       	data_temp1_4(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==4,byte_order1_4(i):byte_order1_4(i)+byte_num1_4(i)-1)',1,[]), byte_type1_4{i}))' * byte_factor1_4(i);
    end
    for i = 1: length(byte_order1_5)
       	data_temp1_5(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==5,byte_order1_5(i):byte_order1_5(i)+byte_num1_5(i)-1)',1,[]), byte_type1_5{i}))' * byte_factor1_5(i);
    end
    for i = 1: length(byte_order1_6)
       	data_temp1_6(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==6,byte_order1_6(i):byte_order1_6(i)+byte_num1_6(i)-1)',1,[]), byte_type1_6{i}))' * byte_factor1_6(i);
    end
    for i = 1: length(byte_order1_7)
       	data_temp1_7(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==7,byte_order1_7(i):byte_order1_7(i)+byte_num1_7(i)-1)',1,[]), byte_type1_7{i}))' * byte_factor1_7(i);
    end

    %% 按协议设定的变量处理解析
    data_temp2 = zeros(size(data_temp1,1),33);
    %GNSS整数小数部分结合
    data_temp2(:,1) =  data_temp1(:,50) + data_temp1(:,51);    %GNSS纬度
    data_temp2(:,2) =  data_temp1(:,52) + data_temp1(:,53);    %GNSS经度
    
    temp =  (-1).^(bitget(data_temp1(:,70),1));
    data_temp2(:,3) =  temp.*data_temp1(:,68)/1000 + data_temp1(:,67);      %GNSS vel time
    temp = (-1).^(bitget(data_temp1(:,70),2));
    data_temp2(:,4) =  temp.*data_temp1(:,69)/1000 + data_temp1(:,67);    %GNSS heading time
    temp = (bitget(data_temp1(:,70),3));
    data_temp2(:,5) =  temp ;   %GNSS pos ava
    temp = (bitget(data_temp1(:,70),4));
    data_temp2(:,6) =  temp ;   %GNSS vel ava
    temp = (bitget(data_temp1(:,70),5));
    data_temp2(:,7) =  temp ;   %GNSS heading ava
    
    temp = bitget(data_temp1(:,80),2)*2 + bitget(data_temp1(:,80),1);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,8) =  temp.*data_temp1(:,71);  % FL wheel speed
    temp = bitget(data_temp1(:,80),4)*2 + bitget(data_temp1(:,80),3);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,9) =  temp.*data_temp1(:,72);  % FR wheel speed
    temp = bitget(data_temp1(:,80),6)*2 + bitget(data_temp1(:,80),5);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,10) =  temp.*data_temp1(:,73);  % RL wheel speed
    temp = bitget(data_temp1(:,80),8)*2 + bitget(data_temp1(:,80),7);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,11) =  temp.*data_temp1(:,74);  % RR wheel speed
    
    temp = bitget(data_temp1(:,81),2)*2 + bitget(data_temp1(:,81),1);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,12) =  temp.*data_temp1(:,75);  % vehicle speed
    temp = bitget(data_temp1(:,81),4)*2 + bitget(data_temp1(:,81),3);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,13) =  temp.*data_temp1(:,76);  %steering angle
    temp = bitget(data_temp1(:,81),6)*2 + bitget(data_temp1(:,81),5);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,14) =  temp.*data_temp1(:,77);  % steering angle vel
    temp = bitget(data_temp1(:,81),8)*2 + bitget(data_temp1(:,81),7);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,15) =  temp.*data_temp1(:,78);  % torque
    
    data_temp2(:,16) = bitget(data_temp1(:,80),2)*2 + bitget(data_temp1(:,80),1);  % FL wheel speed dir
    data_temp2(:,17) = bitget(data_temp1(:,80),4)*2 + bitget(data_temp1(:,80),3);  % FR wheel speed dir
    data_temp2(:,18) = bitget(data_temp1(:,80),6)*2 + bitget(data_temp1(:,80),5);  % RL wheel speed dir
    data_temp2(:,19) = bitget(data_temp1(:,80),8)*2 + bitget(data_temp1(:,80),7);  % RR wheel speed dir
    
    data_temp2(:,20) = bitget(data_temp1(:,81),2)*2 + bitget(data_temp1(:,81),1); % vehicle speed dir
    data_temp2(:,21) = bitget(data_temp1(:,81),4)*2 + bitget(data_temp1(:,81),3);  %steering angle dir
    data_temp2(:,22) = bitget(data_temp1(:,81),6)*2 + bitget(data_temp1(:,81),5);  % steering angle vel dir
    data_temp2(:,23) = bitget(data_temp1(:,81),8)*2 + bitget(data_temp1(:,81),7);  % torque dir
    
    data_temp2(:,24) = bitget(data_temp1(:,82),1); % FL wheel speed valid
    data_temp2(:,25) = bitget(data_temp1(:,82),2);  % FR wheel speed valid    
    data_temp2(:,26) = bitget(data_temp1(:,82),3);  % RL wheel speed valid   
    data_temp2(:,27) = bitget(data_temp1(:,82),4);  % RR wheel speed valid
    data_temp2(:,28) = bitget(data_temp1(:,82),5);  % vehicle speed valid
    data_temp2(:,29) = bitget(data_temp1(:,82),6);  % steering angle valid    
    data_temp2(:,30) = bitget(data_temp1(:,82),7);  % steering angle vel valid    
    data_temp2(:,31) = bitget(data_temp1(:,82),8);  % torque valid
    
    data_temp2(:,32) = bitget(data_temp1(:,83),2)*2 + bitget(data_temp1(:,83),1);  % gear
    data_temp2(:,33) = bitget(data_temp1(:,83),4)*2 + bitget(data_temp1(:,83),3);  % pedal
    
    
    %数据拼接
    data_temp1 = [data_temp1(:,2:32),  data_temp1(:,34:42),  data_temp1(:,49), data_temp2(:,1:2), data_temp1(:,54:67), data_temp2(:,3:15), data_temp1(:,79), data_temp2(:,16:33)];
    
%% 生成table，并做单位处理等
    RawData_table = array2table(data_temp1);    
    RawData_table.Properties.VariableNames =...     %每行6个，便于计数
        {'Latitude_Fusion_Double', 'Longitude_Fusion_Double', 'ins_Height', 'ins_VE', 'ins_VN', 'ins_VU', ...
         'ins_Pitch', 'ins_Roll', 'ins_Heading_fusion', 'ins_Status', 'GNSS_Week_u16', 'GNSS_Time_ms_g_u32', ...
         'Quality_GNSS', 'Soln_SVs_GNSS', 'Soln_SVs_Ante2_GNSS', 'IMU_ACC_x', 'IMU_ACC_y', 'IMU_ACC_z', ...
         'IMU_Gyro_x', 'IMU_Gyro_y', 'IMU_Gyro_z', 'IMU_TEMP', 'ins_Latitude_sigma', 'ins_Longitude_sigma', ...
         'ins_Height_sigma', 'ins_VE_sigma', 'ins_VN_sigma', 'ins_VU_sigma', 'ins_Pitch_sigma', 'ins_Roll_sigma', ...
         'ins_Heading_fusion_sigma', 'reserved34',  'reserved35', 'reserved36', 'reserved37', 'reserved38', ...
         'reserved39', 'reserved40', 'PPS_Count_u32', 'CircleDataIdx', 'main_10ms_Count','Latitude_GNSS_Double', ...
         'Longitude_GNSS_Double', 'GNSS_Height', 'GNSS_Vel_E', 'GNSS_Vel_N', 'GNSS_Vel_Vertical', 'HeadingAngle_GNSS', ...
         'Course_GNSS', 'Pos_RMS_G_x', 'Pos_RMS_G_y','GNSS_Height_RMS', 'Heading_RMS_GNSS', 'GNSS_Delay_ms_u16', ...
         'GNSS_Vel_Latency',  'GNSS_Heading_delay_ms', 'Pos_time', 'Vel_time', 'Heading_time', 'GNSS_Pos_Ava', ...
         'GNSS_Vel_Ava', 'GNSS_heading_Ava', 'FLwheelspeed', 'FRwheelspeed','RLwheelspeed','RRwheelspeed', ...
         'VehicleSpeed', 'SteeringAngle', 'SteeringAngleSpeed',  'Torque', 'Pedal',  'FLwheelspeedDir', ...
         'FRwheelspeedDir', 'RLwheelspeedDir', 'RRwheelspeedDir', 'VehicleSpeedDir', 'SteeringAngleDir', 'SteeringAngleSpeedDir', ...
         'TorqueDir', 'FLwheelspeed_Ava', 'FRwheelspeed_Ava', 'RLwheelspeed_Ava', 'RRwheelspeed_Ava', 'Vehicle_Speed_Ava', ...
         'SteeringAngleValid', 'SteeringAngleSpeed_Ava', 'Torque_Ava', 'Gear_Ava2', 'Pedal_Ava2'};      %这里的valid和ava均包含信号源自检和通信检测
     
    RawData_table.Vel_Level_GNSS = (RawData_table.GNSS_Vel_E.^2 + RawData_table.GNSS_Vel_N.^2).^0.5;    %补充模型接口变量 Vel_Level_GNSS
    
    RawData_table.GNSS_Time_ms_g_u32 = RawData_table.GNSS_Time_ms_g_u32 *1e3;
    RawData_table.ins_Pitch = RawData_table.ins_Pitch/180*pi;
    RawData_table.ins_Roll = RawData_table.ins_Roll/180*pi;
    RawData_table.ins_Heading_fusion = RawData_table.ins_Heading_fusion/180*pi;
    
    RawData_table.IMU_ACC_x = RawData_table.IMU_ACC_x * g;
    RawData_table.IMU_ACC_y = RawData_table.IMU_ACC_y * g;
    RawData_table.IMU_ACC_z = RawData_table.IMU_ACC_z * g;
    RawData_table.IMU_Gyro_x = RawData_table.IMU_Gyro_x /180*pi;
    RawData_table.IMU_Gyro_y = RawData_table.IMU_Gyro_y /180*pi;
    RawData_table.IMU_Gyro_z = RawData_table.IMU_Gyro_z /180*pi;
    
    RawData_table.ins_Pitch_sigma = RawData_table.ins_Pitch_sigma /180*pi;
    RawData_table.ins_Roll_sigma = RawData_table.ins_Roll_sigma /180*pi;
    RawData_table.ins_Heading_fusion_sigma = RawData_table.ins_Heading_fusion_sigma /180*pi;    
    
    RawData_table.HeadingAngle_GNSS = RawData_table.HeadingAngle_GNSS /180*pi;
    RawData_table.Course_GNSS = (RawData_table.Course_GNSS +360).*(RawData_table.Course_GNSS<0) + RawData_table.Course_GNSS.*(RawData_table.Course_GNSS>=0);
    RawData_table.Course_GNSS = RawData_table.Course_GNSS /180*pi;
    RawData_table.GNSS_Vel_Latency = RawData_table.GNSS_Vel_Latency /1000;

%% 轮询数据生成table
    RawData_table_circle1 = array2table(data_temp1_1); 
    RawData_table_circle1.Properties.VariableNames = {'Ins2Vehicle_Pitch', 'Ins2Vehicle_Roll', 'Ins2Vehicle_Yaw'};
    RawData_table_circle2 = array2table(data_temp1_2); 
    RawData_table_circle2.Properties.VariableNames = {'LeverOut_x', 'LeverOut_y', 'LeverOut_z'};
    RawData_table_circle3 = array2table(data_temp1_3); 
    RawData_table_circle3.Properties.VariableNames = {'Lever_x', 'Lever_y', 'Lever_z'};
    RawData_table_circle4 = array2table(data_temp1_4); 
    RawData_table_circle4.Properties.VariableNames = {'GNSS2Vehicle_Pitch', 'GNSS2Vehicle_Roll', 'GNSS2Vehicle_Yaw'};
    RawData_table_circle5 = array2table(data_temp1_5); 
    RawData_table_circle5.Properties.VariableNames = {'SoftwareVersion', 'HardwareVersion', 'reserved'};
    RawData_table_circle6 = array2table(data_temp1_6); 
    RawData_table_circle6.Properties.VariableNames = {'FactoryBias_ax', 'FactoryBias_ay', 'FactoryBias_az'};
    RawData_table_circle7 = array2table(data_temp1_7); 
    RawData_table_circle7.Properties.VariableNames = {'FactoryBias_gx', 'FactoryBias_gy', 'FactoryBias_gz'};
    
else
    byte_num1_localtime = [4,4,2];
    byte_type1_localtime =  {'uint32','uint32','uint16' }; 
    byte_order_localtime = [bytelenth+1, bytelenth+byte_num1_localtime(1)+1, bytelenth+sum(byte_num1_localtime(1:2))+1];
    byte_factor1_localtime = [1, 1, 1]; 
    
    byte_order0 = [byte_order0, byte_order_localtime];
    byte_num0 = [byte_num0, byte_num1_localtime];
    for i=1:length(byte_type1_localtime)
        byte_type0{end+1} = byte_type1_localtime{i};
    end
    byte_factor0 = [byte_factor0, byte_factor1_localtime];
    
    bytelenth = bytelenth + sum(byte_num1_localtime);%
%% 按消息头去除错误数据 
    tic
    idx0 = strfind(reshape(data_temp,1,[]), uint8([hex2dec('AA'),hex2dec('44'),hex2dec('AA'),hex2dec('45')]));
    error0 = sum(uint16(diff(idx0))~=uint16(bytelenth));
    idx0 = idx0(uint16(diff(idx0))>=uint16(bytelenth));
    idx1 = zeros(bytelenth,size(idx0,2));
    for i = 1:bytelenth
        idx1(i,:) = idx0+i-1;
    end
    data_temp = data_temp(idx1);
    data_temp=data_temp';
    toc
%% CRC校验
    if CRC_test == 1
        crc_1 = uint8(data_temp(:,71));%CRC存储
        crc_2 = uint8(data_temp(:,164));%CRC存储
        data1 = uint8(data_temp(:,1:70));%数据存储
        data2 = uint8(data_temp(:,72:163));%数据存储
        crc_1_1 = cellstr(dec2hex(crc_1,2));
        crc_2_2 = cellstr(dec2hex(crc_2,2));
        
        bool_1 =  CRC_mexupdate_0109(data1,crc_1_1,70);
        all_1 = find(bool_1==0);
        bool_2 =  CRC_mexupdate_0109(data2,crc_2_2,92);
        all_2 = find(bool_2==0);  
        
        wrong = 0;
        if isempty(all_1)
            disp('非用户数据全部错误，请检查数据格式重新解析')
            all_error_1 = bool_1(1:end); error_1 = length(all_error_1); 
        else
            all_error_1 = bool_1(1:all_1(1)-1);error_1 = length(all_error_1);
        end
        if isempty(all_2)
            disp('用户数据全部错误，请检查数据格式重新解析')
            all_error_2 = bool_2(1:end);error_2 = length(all_error_2);
        else
            all_error_2 = bool_2(1:all_2(1)-1);error_2 = length(all_error_2);
        end
        
        alldata = length(data_temp(:,1));
        fprintf('Number of all data = %d\n',alldata);
        fprintf('Number of wrong non-users data = %d\n',error_1);
        fprintf('Number of wrong users'' data = %d\n',error_2);
        data_temp([all_error_1;all_error_2] , :) = [];
        fprintf('Number of all wrong data = %d\n',alldata - length(data_temp(:,1)));        
    end
    
    %% 字节数据解析
    data_temp1 = zeros(size(data_temp,1), length(byte_order0));
    for i = 1: length(byte_order0)
        data_temp1(:,i) = double(typecast(reshape(data_temp(:,byte_order0(i):byte_order0(i)+byte_num0(i)-1)',1,[]), byte_type0{i}))' * byte_factor0(i);
    end
    data_temp1_1 = zeros(length(find(data_temp1(:,42)==1)), length(byte_order1_1));
    data_temp1_2 = zeros(length(find(data_temp1(:,42)==2)), length(byte_order1_2));
    data_temp1_3 = zeros(length(find(data_temp1(:,42)==3)), length(byte_order1_3));
    data_temp1_4 = zeros(length(find(data_temp1(:,42)==4)), length(byte_order1_4));
    data_temp1_5 = zeros(length(find(data_temp1(:,42)==5)), length(byte_order1_5));
    data_temp1_6 = zeros(length(find(data_temp1(:,42)==6)), length(byte_order1_6));
    data_temp1_7 = zeros(length(find(data_temp1(:,42)==7)), length(byte_order1_7));
    for i = 1: length(byte_order1_1)
       	data_temp1_1(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==1,byte_order1_1(i):byte_order1_1(i)+byte_num1_1(i)-1)',1,[]), byte_type1_1{i}))' * byte_factor1_1(i);
    end
    for i = 1: length(byte_order1_2)
       	data_temp1_2(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==2,byte_order1_2(i):byte_order1_2(i)+byte_num1_2(i)-1)',1,[]), byte_type1_2{i}))' * byte_factor1_2(i);
    end
    for i = 1: length(byte_order1_3)
       	data_temp1_3(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==3,byte_order1_3(i):byte_order1_3(i)+byte_num1_3(i)-1)',1,[]), byte_type1_3{i}))' * byte_factor1_3(i);
    end
    for i = 1: length(byte_order1_4)
       	data_temp1_4(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==4,byte_order1_4(i):byte_order1_4(i)+byte_num1_4(i)-1)',1,[]), byte_type1_4{i}))' * byte_factor1_4(i);
    end
    for i = 1: length(byte_order1_5)
       	data_temp1_5(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==5,byte_order1_5(i):byte_order1_5(i)+byte_num1_5(i)-1)',1,[]), byte_type1_5{i}))' * byte_factor1_5(i);
    end
    for i = 1: length(byte_order1_6)
       	data_temp1_6(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==6,byte_order1_6(i):byte_order1_6(i)+byte_num1_6(i)-1)',1,[]), byte_type1_6{i}))' * byte_factor1_6(i);
    end
    for i = 1: length(byte_order1_7)
       	data_temp1_7(:,i) = double(typecast(reshape(data_temp(data_temp1(:,42)==7,byte_order1_7(i):byte_order1_7(i)+byte_num1_7(i)-1)',1,[]), byte_type1_7{i}))' * byte_factor1_7(i);
    end

    %% 按协议设定的变量处理解析
    data_temp2 = zeros(size(data_temp1,1),33);
    %GNSS整数小数部分结合
    data_temp2(:,1) =  data_temp1(:,50) + data_temp1(:,51);    %GNSS纬度
    data_temp2(:,2) =  data_temp1(:,52) + data_temp1(:,53);    %GNSS经度
    
    temp =  (-1).^(bitget(data_temp1(:,70),1));
    data_temp2(:,3) =  temp.*data_temp1(:,68)/1000 + data_temp1(:,67);      %GNSS vel time
    temp = (-1).^(bitget(data_temp1(:,70),2));
    data_temp2(:,4) =  temp.*data_temp1(:,69)/1000 + data_temp1(:,67);    %GNSS heading time
    temp = (bitget(data_temp1(:,70),3));
    data_temp2(:,5) =  temp ;   %GNSS pos ava
    temp = (bitget(data_temp1(:,70),4));
    data_temp2(:,6) =  temp ;   %GNSS vel ava
    temp = (bitget(data_temp1(:,70),5));
    data_temp2(:,7) =  temp ;   %GNSS heading ava
    
    temp = bitget(data_temp1(:,80),2)*2 + bitget(data_temp1(:,80),1);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,8) =  temp.*data_temp1(:,71);  % FL wheel speed
    temp = bitget(data_temp1(:,80),4)*2 + bitget(data_temp1(:,80),3);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,9) =  temp.*data_temp1(:,72);  % FR wheel speed
    temp = bitget(data_temp1(:,80),6)*2 + bitget(data_temp1(:,80),5);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,10) =  temp.*data_temp1(:,73);  % RL wheel speed
    temp = bitget(data_temp1(:,80),8)*2 + bitget(data_temp1(:,80),7);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,11) =  temp.*data_temp1(:,74);  % RR wheel speed
    
    temp = bitget(data_temp1(:,81),2)*2 + bitget(data_temp1(:,81),1);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,12) =  temp.*data_temp1(:,75);  % vehicle speed
    temp = bitget(data_temp1(:,81),4)*2 + bitget(data_temp1(:,81),3);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,13) =  temp.*data_temp1(:,76);  %steering angle
    temp = bitget(data_temp1(:,81),6)*2 + bitget(data_temp1(:,81),5);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,14) =  temp.*data_temp1(:,77);  % steering angle vel
    temp = bitget(data_temp1(:,81),8)*2 + bitget(data_temp1(:,81),7);
    temp = 0.*(abs(temp-0)<0.5) + 1.*(abs(temp-1)<0.5) + (-1).*(abs(temp-2)<0.5) + 1.*(abs(temp-3)<0.5);
    data_temp2(:,15) =  temp.*data_temp1(:,78);  % torque
    
    data_temp2(:,16) = bitget(data_temp1(:,80),2)*2 + bitget(data_temp1(:,80),1);  % FL wheel speed dir
    data_temp2(:,17) = bitget(data_temp1(:,80),4)*2 + bitget(data_temp1(:,80),3);  % FR wheel speed dir
    data_temp2(:,18) = bitget(data_temp1(:,80),6)*2 + bitget(data_temp1(:,80),5);  % RL wheel speed dir
    data_temp2(:,19) = bitget(data_temp1(:,80),8)*2 + bitget(data_temp1(:,80),7);  % RR wheel speed dir
    
    data_temp2(:,20) = bitget(data_temp1(:,81),2)*2 + bitget(data_temp1(:,81),1); % vehicle speed dir
    data_temp2(:,21) = bitget(data_temp1(:,81),4)*2 + bitget(data_temp1(:,81),3);  %steering angle dir
    data_temp2(:,22) = bitget(data_temp1(:,81),6)*2 + bitget(data_temp1(:,81),5);  % steering angle vel dir
    data_temp2(:,23) = bitget(data_temp1(:,81),8)*2 + bitget(data_temp1(:,81),7);  % torque dir
    
    data_temp2(:,24) = bitget(data_temp1(:,82),1); % FL wheel speed valid
    data_temp2(:,25) = bitget(data_temp1(:,82),2);  % FR wheel speed valid    
    data_temp2(:,26) = bitget(data_temp1(:,82),3);  % RL wheel speed valid   
    data_temp2(:,27) = bitget(data_temp1(:,82),4);  % RR wheel speed valid
    data_temp2(:,28) = bitget(data_temp1(:,82),5);  % vehicle speed valid
    data_temp2(:,29) = bitget(data_temp1(:,82),6);  % steering angle valid    
    data_temp2(:,30) = bitget(data_temp1(:,82),7);  % steering angle vel valid    
    data_temp2(:,31) = bitget(data_temp1(:,82),8);  % torque valid
    
    data_temp2(:,32) = bitget(data_temp1(:,83),2)*2 + bitget(data_temp1(:,83),1);  % gear
    data_temp2(:,33) = bitget(data_temp1(:,83),4)*2 + bitget(data_temp1(:,83),3);  % pedal
    
    
    %数据拼接
    data_temp1 = [data_temp1(:,2:32),  data_temp1(:,34:42),  data_temp1(:,49), data_temp2(:,1:2), data_temp1(:,54:67), data_temp2(:,3:15), data_temp1(:,79), data_temp2(:,16:33), data_temp1(:,end-2:end)];
    
%% 生成table，并做单位处理等
    RawData_table = array2table(data_temp1);    
    RawData_table.Properties.VariableNames =...     %每行6个，便于计数
        {'Latitude_Fusion_Double', 'Longitude_Fusion_Double', 'ins_Height', 'ins_VE', 'ins_VN', 'ins_VU', ...
         'ins_Pitch', 'ins_Roll', 'ins_Heading_fusion', 'ins_Status', 'GNSS_Week_u16', 'GNSS_Time_ms_g_u32', ...
         'Quality_GNSS', 'Soln_SVs_GNSS', 'Soln_SVs_Ante2_GNSS', 'IMU_ACC_x', 'IMU_ACC_y', 'IMU_ACC_z', ...
         'IMU_Gyro_x', 'IMU_Gyro_y', 'IMU_Gyro_z', 'IMU_TEMP', 'ins_Latitude_sigma', 'ins_Longitude_sigma', ...
         'ins_Height_sigma', 'ins_VE_sigma', 'ins_VN_sigma', 'ins_VU_sigma', 'ins_Pitch_sigma', 'ins_Roll_sigma', ...
         'ins_Heading_fusion_sigma', 'reserved34',  'reserved35', 'reserved36', 'reserved37', 'reserved38', ...
         'reserved39', 'reserved40', 'PPS_Count_u32', 'CircleDataIdx', 'main_10ms_Count','Latitude_GNSS_Double', ...
         'Longitude_GNSS_Double', 'GNSS_Height', 'GNSS_Vel_E', 'GNSS_Vel_N', 'GNSS_Vel_Vertical', 'HeadingAngle_GNSS', ...
         'Course_GNSS', 'Pos_RMS_G_x', 'Pos_RMS_G_y','GNSS_Height_RMS', 'Heading_RMS_GNSS', 'GNSS_Delay_ms_u16', ...
         'GNSS_Vel_Latency',  'GNSS_Heading_delay_ms', 'Pos_time', 'Vel_time', 'Heading_time', 'GNSS_Pos_Ava', ...
         'GNSS_Vel_Ava', 'GNSS_heading_Ava', 'FLwheelspeed', 'FRwheelspeed','RLwheelspeed','RRwheelspeed', ...
         'VehicleSpeed', 'SteeringAngle', 'SteeringAngleSpeed',  'Torque', 'Pedal',  'FLwheelspeedDir', ...
         'FRwheelspeedDir', 'RLwheelspeedDir', 'RRwheelspeedDir', 'VehicleSpeedDir', 'SteeringAngleDir', 'SteeringAngleSpeedDir', ...
         'TorqueDir', 'FLwheelspeed_Ava', 'FRwheelspeed_Ava', 'RLwheelspeed_Ava', 'RRwheelspeed_Ava', 'Vehicle_Speed_Ava', ...
        'SteeringAngleValid', 'SteeringAngleSpeed_Ava', 'Torque_Ava', 'Gear_Ava2', 'Pedal_Ava2', 'Local_Date','Local_Time','Local_Msec'};      %这里的valid和ava均包含信号源自检和通信检测
     
    RawData_table.Vel_Level_GNSS = (RawData_table.GNSS_Vel_E.^2 + RawData_table.GNSS_Vel_N.^2).^0.5;    %补充模型接口变量 Vel_Level_GNSS
    
    RawData_table.GNSS_Time_ms_g_u32 = RawData_table.GNSS_Time_ms_g_u32 *1e3;
    RawData_table.ins_Pitch = RawData_table.ins_Pitch/180*pi;
    RawData_table.ins_Roll = RawData_table.ins_Roll/180*pi;
    RawData_table.ins_Heading_fusion = RawData_table.ins_Heading_fusion/180*pi;
    
    RawData_table.IMU_ACC_x = RawData_table.IMU_ACC_x * g;
    RawData_table.IMU_ACC_y = RawData_table.IMU_ACC_y * g;
    RawData_table.IMU_ACC_z = RawData_table.IMU_ACC_z * g;
    RawData_table.IMU_Gyro_x = RawData_table.IMU_Gyro_x /180*pi;
    RawData_table.IMU_Gyro_y = RawData_table.IMU_Gyro_y /180*pi;
    RawData_table.IMU_Gyro_z = RawData_table.IMU_Gyro_z /180*pi;
    
    RawData_table.ins_Pitch_sigma = RawData_table.ins_Pitch_sigma /180*pi;
    RawData_table.ins_Roll_sigma = RawData_table.ins_Roll_sigma /180*pi;
    RawData_table.ins_Heading_fusion_sigma = RawData_table.ins_Heading_fusion_sigma /180*pi;    
    
    RawData_table.HeadingAngle_GNSS = RawData_table.HeadingAngle_GNSS /180*pi;    
    RawData_table.Course_GNSS = (RawData_table.Course_GNSS +360).*(RawData_table.Course_GNSS<0) + RawData_table.Course_GNSS.*(RawData_table.Course_GNSS>=0);
    RawData_table.Course_GNSS = RawData_table.Course_GNSS /180*pi;
    RawData_table.GNSS_Vel_Latency = RawData_table.GNSS_Vel_Latency /1000;
    %% 轮询数据生成table
    RawData_table_circle1 = array2table(data_temp1_1); 
    RawData_table_circle1.Properties.VariableNames = {'Ins2Vehicle_Pitch', 'Ins2Vehicle_Roll', 'Ins2Vehicle_Yaw'};
    RawData_table_circle2 = array2table(data_temp1_2); 
    RawData_table_circle2.Properties.VariableNames = {'LeverOut_x', 'LeverOut_y', 'LeverOut_z'};
    RawData_table_circle3 = array2table(data_temp1_3); 
    RawData_table_circle3.Properties.VariableNames = {'Lever_x', 'Lever_y', 'Lever_z'};
    RawData_table_circle4 = array2table(data_temp1_4); 
    RawData_table_circle4.Properties.VariableNames = {'GNSS2Vehicle_Pitch', 'GNSS2Vehicle_Roll', 'GNSS2Vehicle_Yaw'};
    RawData_table_circle5 = array2table(data_temp1_5); 
    RawData_table_circle5.Properties.VariableNames = {'SoftwareVersion', 'HardwareVersion', 'reserved'};
    RawData_table_circle6 = array2table(data_temp1_6); 
    RawData_table_circle6.Properties.VariableNames = {'FactoryBias_ax', 'FactoryBias_ay', 'FactoryBias_az'};
    RawData_table_circle7 = array2table(data_temp1_7); 
    RawData_table_circle7.Properties.VariableNames = {'FactoryBias_gx', 'FactoryBias_gy', 'FactoryBias_gz'};
    
end

%% 数据处理b
RawData_table.ASM330_TEMP = RawData_table.reserved34;
RawData_table.IMU_ACC_x_ASM330 = RawData_table.reserved35 * g;
RawData_table.IMU_ACC_y_ASM330 = RawData_table.reserved36 * g;
RawData_table.IMU_ACC_z_ASM330 = RawData_table.reserved37 * g;
RawData_table.IMU_Gyro_x_ASM330 = RawData_table.reserved38 /180*pi;
RawData_table.IMU_Gyro_y_ASM330 = RawData_table.reserved39 /180*pi;
RawData_table.IMU_Gyro_z_ASM330 = RawData_table.reserved40 /180*pi;


%% 仿真时间生成
main_10ms_Count = RawData_table.main_10ms_Count;
Time = (0:1:length(main_10ms_Count)-1)'*0.01;


%% 处理杆臂数据
if leverfile_enable==1
    fileLever = fopen([ FileName(1:idx_dash),'Lever.txt']);
    C = textscan(fileLever,'%s %f %f %f','Delimiter',{'\n','\r'});
    fclose(fileLever);
    lever = single([C{2};C{3};C{4}]);
end

%% 存储数据

if SaveEnable
    if exist(SaveFileFolder(1:end-1),'dir')==0
        mkdir(SaveFileFolder(1:end-1));
    end
end

if SaveEnable
    data_num = FileName(idx_dash+1:length(FileName)-4);
    save_name = strcat(SaveFileFolder,'STM32_',data_num);
end
if SaveEnable
    if leverfile_enable==1
        save(save_name,'RawData_table','Time','lever', 'RawData_table_circle1', 'RawData_table_circle2', 'RawData_table_circle3', 'RawData_table_circle4', 'RawData_table_circle5', 'RawData_table_circle6', 'RawData_table_circle7');
    else
        save(save_name,'RawData_table','Time', 'RawData_table_circle1', 'RawData_table_circle2', 'RawData_table_circle3', 'RawData_table_circle4', 'RawData_table_circle5', 'RawData_table_circle6', 'RawData_table_circle7');
    end
    disp('将数据存放在如下位置：')
    disp(save_name)
else
    disp('这组解析后的数据没有保存')
end

%% 绘图

Pos_ratio_longi = 5.519e6*deg2rad(1);
Pos_ratio_lati = 6.3515e6*deg2rad(1);
pos_local_x0 = 1.212088647000000e+02;%Pos_long_9250_4.signals.values(1,1);
pos_local_y0 = 31.289691800000000;%Pos_lat_9250_4.signals.values(1,1);

GNSS_Lati = RawData_table.Latitude_GNSS_Double;
GNSS_Longi = RawData_table.Longitude_GNSS_Double;
GNSS_Lati_Fusion = RawData_table.Latitude_Fusion_Double;
GNSS_Longi_Fusion = RawData_table.Longitude_Fusion_Double;

% GNSS_local_x = (GNSS_Longi- pos_local_x0)*Pos_ratio_longi;
% GNSS_local_y = (GNSS_Lati - pos_local_y0)*Pos_ratio_lati;
% 
% GNSS_local_x_Fusion = (GNSS_Longi_Fusion- pos_local_x0)*Pos_ratio_longi;
% GNSS_local_y_Fusion = (GNSS_Lati_Fusion - pos_local_y0)*Pos_ratio_lati;

% lla = [GNSS_Lati_Fusion,GNSS_Longi_Fusion,RawData_table.GNSS_Height];
% ecef = lla2ecef(lla);
% f_figure(1,'位置-局部坐标系') %画平面坐标m图
%     hold on
%     grid on
%     plot3(GNSS_local_x,GNSS_local_y,Time,'.','DisplayName','GPS');
%     plot3(GNSS_local_x_Fusion,GNSS_local_y_Fusion,Time,'.','DisplayName','Fusion');
%     legend
%     xlabel('局部坐标 x m')
%     ylabel('局部坐标 y m')

f_figure(2,'经纬度') %画经纬度图
hold on
grid on
plot3(GNSS_Longi,GNSS_Lati,Time,'.','DisplayName','GPS');
plot3(GNSS_Longi_Fusion,GNSS_Lati_Fusion,Time,'.','DisplayName','Fusion');
legend
xlabel('经度 deg')
ylabel('纬度 deg')
% f_figure(2002,'ECEF') %画经纬度图
%     hold on
%     grid on
% %     plot3(GNSS_Longi,GNSS_Lati,Time,'.','DisplayName','GPS');
%     plot3(ecef(:,2),ecef(:,1),Time,'.','DisplayName','gnss-ecef');
%     legend
%     xlabel('x m')
%     ylabel('y m')
%     f_figure(2,'经纬度') %画经纬度图
%     hold on
%     grid on
%     plot3(jing,wei,Time1,'.','DisplayName','GPS');
% %     plot3(GNSS_Longi_Fusion,GNSS_Lati_Fusion,Time1,'.','DisplayName','Fusion');
%     legend
%     xlabel('经度 deg')
%     ylabel('纬度 deg')

f_figure(3,'定位状态')
hold on
grid on
plot(Time,RawData_table.Quality_GNSS,'DisplayName','Pos_Type');
legend
xlabel('时间 s')
ylabel('定位状态')
ylim([-1 6]);


%  ACC_SENSITIVITY=0.244/1000;
%  IMU_ACC_z = RawData_table.IMU_ACC_z/ACC_SENSITIVITY;
%  IMU_ACC_z_hex = fix(IMU_ACC_z);
f_figure(4,'加速度 角速度')
subplot(2,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_x,'DisplayName','x');
plot(Time,RawData_table.IMU_ACC_y,'DisplayName','y');
plot(Time,RawData_table.IMU_ACC_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(2,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_x*rad2deg(1),'DisplayName','x');
plot(Time,RawData_table.IMU_Gyro_y*rad2deg(1),'DisplayName','y');
plot(Time,RawData_table.IMU_Gyro_z*rad2deg(1),'DisplayName','z');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')

f_figure(5,'位置RMS')
hold on
grid on
plot(Time,RawData_table.Pos_RMS_G_x,'DisplayName','x');
plot(Time,RawData_table.Pos_RMS_G_y,'DisplayName','y');
legend
xlabel('时间 s')
ylabel('位置精度 m')

f_figure(6,'速度')
%     subplot(2,1,1)
hold on
grid on
plot(Time,RawData_table.Vel_Level_GNSS,'DisplayName','GNSS');
plot(Time,RawData_table.VehicleSpeed,'DisplayName','Vehicle');
legend
%     subplot(2,1,2)
%     hold on
%     grid on
%     plot(Time,RawData_table.ins_VE,'DisplayName','ins_VE');
%     plot(Time,RawData_table.ins_VN,'DisplayName','ins_VN');
%     plot(Time,RawData_table.ins_VU,'DisplayName','ins_VU');
%     legend
%     xlabel('时间 s')
%     ylabel('速度 m/s')

f_figure(7,'航向角')
hold on
grid on
plot(Time,RawData_table.Course_GNSS*rad2deg(1),'DisplayName','Course');
plot(Time,RawData_table.HeadingAngle_GNSS*rad2deg(1),'DisplayName','Heading');
plot(Time,RawData_table.ins_Heading_fusion*rad2deg(1),'DisplayName','Heading_fusion');
legend
xlabel('时间 s')
ylabel('角度 deg')
grid on
f_figure(71,'航向角 误差')
subplot(2,1,1)
hold on
grid on
plot(Time,RawData_table.HeadingAngle_GNSS*rad2deg(1),'DisplayName','GPS');
plot(Time,RawData_table.ins_Heading_fusion*rad2deg(1),'DisplayName','Heading_fusion');
legend
xlabel('时间 s')
ylabel('角度 deg')
grid on
subplot(2,1,2)
hold on
grid on
plot(Time,RawData_table.HeadingAngle_GNSS*rad2deg(1) - RawData_table.ins_Heading_fusion*rad2deg(1),'DisplayName','GPS - Fusion');
legend
xlabel('时间 s')
ylabel('角度 deg')
grid on

f_figure(8,'HeadingRMS')
hold on
plot(Time,RawData_table.Heading_RMS_GNSS,'DisplayName','Heading_RMS_GNSS');
ylim([0 10])
legend
xlabel('时间 s')
ylabel('角度 deg')
grid on

f_figure(9,'高度')
subplot(2,1,1)
hold on
grid on
plot(Time,RawData_table.GNSS_Height,'DisplayName','Height');
plot(Time,RawData_table.ins_Height,'DisplayName','ins_Height');
legend
xlabel('时间 s')
ylabel('高度 m')
subplot(2,1,2)
hold on
grid on
plot(Time,RawData_table.GNSS_Height_RMS,'DisplayName','RMS');
legend
xlabel('时间 s')
ylabel('误差 m')

f_figure(10,'卫星个数')
hold on
grid on
plot(Time,RawData_table.Soln_SVs_GNSS,'DisplayName','主天线');
plot(Time,RawData_table.Soln_SVs_Ante2_GNSS,'DisplayName','副天线');
legend
xlabel('时间 s')
ylabel('个数')
ylim([0 50])


%%
A = [main_10ms_Count;main_10ms_Count(end)]-[main_10ms_Count(1);main_10ms_Count];
% temp = A<-100;
% A(temp) = 1;
% temp = find(A>100);
% A(temp) = 1;
A(end) = 1;
% temp = find(A>15);
% disp(temp)

f_figure(10000,'main_10ms_Count Timegap')
subplot(2,1,1)
plot(Time,A(2:end),'DisplayName','Time gap 正常应该是1')
legend
grid
xlabel('时间 s')
subplot(2,1,2)
plot(Time,main_10ms_Count,'DisplayName','main_10ms_Count')
legend
grid
xlabel('时间 s')

%%
A = RawData_table.IMU_Gyro_y/0.00875*57.3;
AA = [A;A(end)]-[A(1);A];
temp_Ax = find(RawData_table.IMU_ACC_x == 0);
temp_Ay = find(RawData_table.IMU_ACC_y == 0);
temp_Az = find(RawData_table.IMU_ACC_z == 0);
temp_Gx = find(RawData_table.IMU_Gyro_x == 0);
temp_Gy = find(RawData_table.IMU_Gyro_y == 0);
temp_Gz = find(RawData_table.IMU_Gyro_z == 0);

f_figure(40,'加速度 角速度')
subplot(2,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_x,'DisplayName','x');
plot(Time,RawData_table.IMU_ACC_y,'DisplayName','y');
plot(Time,RawData_table.IMU_ACC_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(2,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_x*57.3,'DisplayName','x');
plot(Time,RawData_table.IMU_Gyro_y*57.3,'DisplayName','y');
plot(Time,RawData_table.IMU_Gyro_z*57.3,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')

% f_figure(41,'加速度 角速度')
%     hold on
%     grid on
%     plot(Time,RawData_table.ins_Pitch-main_10ms_Count,'DisplayName','ACC_Count');
%     plot(Time,RawData_table.ins_Roll-main_10ms_Count,'DisplayName','Gyro_Count');
%     legend
%     xlabel('时间 s')
%     ylabel('加速度m/s^2')
% figure;
% hold on
% plot(Time,RawData_table.Pos_time-RawData_table.Heading_time)
% plot(Time,RawData_table.Pos_time-RawData_table.Vel_time)
%%
f_figure(11,'GNSS Timegap')
hold on
subplot(3,1,1)
plot(Time,RawData_table.Pos_time-[RawData_table.Pos_time(1);RawData_table.Pos_time(1:end-1)],'DisplayName','Postime Gap')
legend
xlabel('时间 s')
ylabel('时间误差 s')
hold on
subplot(3,1,2)
plot(Time,RawData_table.Heading_time-[RawData_table.Heading_time(1);RawData_table.Heading_time(1:end-1)],'DisplayName','Headingtime Gap')
legend
xlabel('时间 s')
ylabel('时间误差 s')
hold on
subplot(3,1,3)
plot(Time,RawData_table.Vel_time-[RawData_table.Vel_time(1);RawData_table.Vel_time(1:end-1)],'DisplayName','Veltime Gap')
legend
xlabel('时间 s')
ylabel('时间误差 s')


%%
Veltime = RawData_table.Vel_time - RawData_table.Vel_time(1);

%%
%     f_figure(711,'姿态角')
%     hold on
%     grid on
% %     plot(Time,RawData_table.HeadingAngle_GNSS*rad2deg(1)-175,'DisplayName','Heading');
% %     plot(Time,RawData_table.ins_Heading_fusion*rad2deg(1)-175,'DisplayName','Heading_fusion');
%     plot(Time,RawData_table.ins_Roll*rad2deg(1),'DisplayName','ins_Roll');
%     plot(Time,RawData_table.ins_Pitch*rad2deg(1),'DisplayName','ins_Pitch');
%     legend
%     xlabel('时间 s')
%     ylabel('姿态角 deg')
% %     xlim([100 1.315e4])
%%
f_figure(12,'位置延迟时间')
hold on
grid on
%     plot(Time,f_s16_array(RawData_table.GNSS_Delay_ms_u16),'DisplayName','GNSS_Delay_ms_u16');
plot(Time,(RawData_table.GNSS_Delay_ms_u16),'DisplayName','GNSS_Delay_ms_u16');
legend
xlabel('时间 s')
ylabel('延迟时间 ms')

%%
f_figure(400,'加速度 角速度')
subplot(3,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_x,'DisplayName','x');
plot(Time,RawData_table.IMU_ACC_y,'DisplayName','y');
plot(Time,RawData_table.IMU_ACC_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(3,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_x*57.3,'DisplayName','x');
plot(Time,RawData_table.IMU_Gyro_y*57.3,'DisplayName','y');
plot(Time,RawData_table.IMU_Gyro_z*57.3,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
subplot(3,1,3)
hold on
grid on
%     plot(Time,RawData_table.IMU_ACC_x,'DisplayName','x');
%     plot(Time,RawData_table.IMU_ACC_y,'DisplayName','y');
plot(Time,RawData_table.IMU_ACC_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')


%%
f_figure(13,'PPS&时钟调节标志')
subplot(2,1,1)
hold on
grid on
plot(Time,RawData_table.PPS_Count_u32,'DisplayName','PPS_Count_u32');
xlabel('时间 s')
ylabel('PPS时间 s')
legend
subplot(2,1,2)
hold on
grid on
% plot(Time,RawData_table.Timmer_add_Count_u16,'DisplayName','Timmer_add_Count_u16');
%     plot(Time,RawData_table.Timmer_minus_Count_u16,'DisplayName','Timmer_minus_Count_u16');
legend
xlabel('时间 s')
ylabel('调整次数')
%%
% RawData_table = RawData_table(15*100:end,:);
% 随着时间的增长，y一直在减小，说明T一直在增大，即10ms的count频率过高了，
T = (RawData_table.main_10ms_Count - RawData_table.main_10ms_Count(1))*0.01;
y = RawData_table.Pos_time-RawData_table.Pos_time(1) - T;
Pos_time_PPS =RawData_table.PPS_Count_u32 - RawData_table.PPS_Count_u32(1);
y_Pos_time = Pos_time_PPS - T;
AAB = Pos_time_PPS(2:end) - Pos_time_PPS(1:end-1);
% figure
% plot(T(2:end),AAB)
f_figure(100,'GNSS MCU 时间误差')
hold on
plot(T,y_Pos_time,'DisplayName','GNSSTime PPS - MCUTime')
plot(T,y,'DisplayName','GNSSTime Bestpos - MCUTime')
legend
xlabel('时间 s')
ylabel('时间误差 s')
% if max(abs(y))>100
%     ylim([-1 1])
% end
%%
ASM330LHH_ACC_Read_Count = RawData_table.ins_Pitch;
AA = main_10ms_Count - ASM330LHH_ACC_Read_Count;
AAB = find(AA >3);
% disp(main_10ms_Count(AAB(1)));
%%
f_figure(14,'温度')
hold on
grid on
plot(Time,RawData_table.IMU_TEMP,'DisplayName','IMU TEMP');
% plot(Time,RawData_table.Array_63,'DisplayName','IMU _UNO acc y_gyro TEMP');
% plot(Time,RawData_table.Array_64,'DisplayName','IMU _DUE gyro TEMP');
plot(Time,RawData_table.reserved34,'DisplayName','asm330 TEMP');
legend
xlabel('时间 s')
ylabel('温度 ℃')
%
%     f_figure(15,'温度-加速度')
% subplot(3,1,1)
%     hold on
%     grid on
%     plot(RawData_table.IMU_TEMP,RawData_table.IMU_ACC_x,'DisplayName','X');
%     legend
%     xlabel('温度 ℃')
%     ylabel('加速度')
% subplot(3,1,2)
%     hold on
%     grid on
%     plot(RawData_table.IMU_TEMP,RawData_table.IMU_ACC_y,'DisplayName','Y');
%     legend
%     xlabel('温度 ℃')
%     ylabel('加速度')
% subplot(3,1,3)
%     hold on
%     grid on
%     plot(RawData_table.IMU_TEMP,RawData_table.IMU_ACC_z,'DisplayName','Z');
%     legend
%     xlabel('温度 ℃')
%     ylabel('加速度')
% f_figure(16,'温度-角速度')
% subplot(3,1,1)
%     hold on
%     grid on
%     plot(RawData_table.IMU_TEMP,RawData_table.IMU_Gyro_x*57.3,'DisplayName','X');
%     legend
%     xlabel('温度 ℃')
%     ylabel('角速度')
% subplot(3,1,2)
%     hold on
%     grid on
%     plot(RawData_table.IMU_TEMP,RawData_table.IMU_Gyro_y*57.3,'DisplayName','Y');
%     legend
%     xlabel('温度 ℃')
%     ylabel('角速度')
% subplot(3,1,3)
%     hold on
%     grid on
%     plot(RawData_table.IMU_TEMP,RawData_table.IMU_Gyro_z*57.3,'DisplayName','Z');
%     legend
%     xlabel('温度 ℃')
%     ylabel('角速度')

%% 加速度分辨率 *1000/0.244/9.7964
f_figure(4000,'加速度')
subplot(3,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_x,'DisplayName','x');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(3,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_y,'DisplayName','y');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(3,1,3)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_z,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
f_figure(4001,'角速度')
subplot(3,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_x*57.3,'DisplayName','x');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
subplot(3,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_y*57.3,'DisplayName','y');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
subplot(3,1,3)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_z*57.3,'DisplayName','z');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
%% 发动机转速
%     f_figure(20,'MotorSpeed')
%     hold on
%     grid on
%     plot(Time,RawData_table.MotorSpeed,'DisplayName','MotorSpeed');
%     legend
%     xlabel('时间 s')
%     ylabel('转速 rpm')
%     %ylim([0 40])
%     %% SteeringAngle
%     f_figure(21,'SteeringAngle')
%     hold on
%     grid on
%     plot(Time,RawData_table.SteeringAngle,'DisplayName','SteeringAngle');
%     legend
%     xlabel('时间 s')
%     ylabel('')
%     %% SteeringAngleValid
%     f_figure(22,'SteeringAngleValid')
%     hold on
%     grid on
%     plot(Time,RawData_table.SteeringAngleValid,'DisplayName','SteeringAngleValid');
%     legend
%     xlabel('时间 s')
%     ylabel('')
%     ylim([0 2])
%     %% SteeringVelocity
%     f_figure(23,'SteeringVelocity')
%     hold on
%     grid on
%     plot(Time,RawData_table.SteeringVelocity,'DisplayName','SteeringVelocity');
%     legend
%     xlabel('时间 s')
%     ylabel('')
%     %% SteeringVelocityValid
%     f_figure(24,'SteeringVelocityValid')
%     hold on
%     grid on
%     plot(Time,RawData_table.SteeringVelocityValid,'DisplayName','SteeringVelocityValid');
%     legend
%     xlabel('时间 s')
%     ylabel('')
%     ylim([0 2])
%% Wheelspeed
% VSPD_Factor = 0.05625;
% VSPD_Factor = 0.01; % tulingplus
% %     VSPD_Factor = 0.03125;
% %     VSPD_Factor = 0.0316;%标定数值
% VSPD_Factor = 0.1; % E50 kph 标定值
VSPD_Factor = 3.6; % E50 kph 标定值

%     f_figure(2600,'Wheelspeed')
%     hold on
%     grid on
%     plot(Time,RawData_table.FLwheelspeed*VSPD_Factor,'DisplayName','FLwheelspeed');
%     plot(Time,RawData_table.FRwheelspeed*VSPD_Factor,'DisplayName','FRwheelspeed');
%     plot(Time,RawData_table.RLwheelspeed*VSPD_Factor,'DisplayName','Rlwheelspeed');
%     plot(Time,RawData_table.RRwheelspeed*VSPD_Factor,'DisplayName','RRwheelspeed');
%     plot(Time,RawData_table.Vel_Level_GNSS*3.6,'DisplayName','GNSS');

%     VSPD_Factor = 0.01; % tulingplus

%     f_figure(2600,'Wheelspeed')
%     hold on
%     grid on
%     plot(Time,f_s16_array(RawData_table.FLwheelspeed)*VSPD_Factor,'DisplayName','FLwheelspeed');
%     plot(Time,f_s16_array(RawData_table.FRwheelspeed)*VSPD_Factor,'DisplayName','FRwheelspeed');
%     plot(Time,f_s16_array(RawData_table.RLwheelspeed)*VSPD_Factor,'DisplayName','Rlwheelspeed');
%     plot(Time,f_s16_array(RawData_table.RRwheelspeed)*VSPD_Factor,'DisplayName','RRwheelspeed');
%     plot(Time,RawData_table.Vel_Level_GNSS*3.6,'DisplayName','GNSS');
%
%     legend
%     xlabel('时间 s')
%     ylabel('速度 kph')
f_figure(2600,'Wheelspeed')
hold on
grid on
plot(Time,RawData_table.FLwheelspeed*VSPD_Factor,'DisplayName','FLwheelspeed');
plot(Time,RawData_table.FRwheelspeed*VSPD_Factor,'DisplayName','FRwheelspeed');
plot(Time,RawData_table.RLwheelspeed*VSPD_Factor,'DisplayName','Rlwheelspeed');
plot(Time,RawData_table.RRwheelspeed*VSPD_Factor,'DisplayName','RRwheelspeed');
plot(Time,RawData_table.Vel_Level_GNSS*3.6,'DisplayName','GNSS');

legend
xlabel('时间 s')
ylabel('速度 kph')

%% PedalPosnBrake
%     f_figure(30,'PedalPosnBrake')
%     hold on
%     grid on`
%     plot(Time,RawData_table.PedalPosnBrake,'DisplayName','PedalPosnBrake');
%     legend
%     xlabel('时间 s')
%     ylabel('')
%     ylim([0 100])
%% GNSS_Time_ms_g_u32
GNSS_Time_ms_g_u32_r = RawData_table.GNSS_Time_ms_g_u32 - RawData_table.GNSS_Time_ms_g_u32(1);
BestPos_time_r = RawData_table.Pos_time-RawData_table.Pos_time(1);

%% 只针对大屏幕 摆放图的位置
% gcf_pos = get(gcf,'position');
% figure(3);set(gcf,'position',[gcf_pos(1)-gcf_pos(3)*1.05, gcf_pos(2)-gcf_pos(4)*1.22, gcf_pos(3),gcf_pos(4)]);%set(gcf,'position',[10 50 560 420]);
% figure(2);set(gcf,'position',[gcf_pos(1), gcf_pos(2)-gcf_pos(4)*1.22, gcf_pos(3),gcf_pos(4)]);
% figure(10000);set(gcf,'position',[gcf_pos(1)+gcf_pos(3)*1.05, gcf_pos(2)-gcf_pos(4)*1.22, gcf_pos(3),gcf_pos(4)]);
% figure(71);set(gcf,'position',[gcf_pos(1)-gcf_pos(3)*1.05, gcf_pos(2), gcf_pos(3),gcf_pos(4)]);
% figure(10);set(gcf,'position',[gcf_pos(1)+gcf_pos(3)*1.05, gcf_pos(2), gcf_pos(3),gcf_pos(4)]);

% figure(11)

%% asm muruta plot


f_figure(40000,'加速度')
subplot(3,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_x_ASM330,'DisplayName','x ASM330');
plot(Time,RawData_table.IMU_ACC_x,'DisplayName','x Muruta');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(3,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_y_ASM330,'DisplayName','y ASM330');
plot(Time,RawData_table.IMU_ACC_y,'DisplayName','y Muruta');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
subplot(3,1,3)
hold on
grid on
plot(Time,RawData_table.IMU_ACC_z_ASM330,'DisplayName','z ASM330');
plot(Time,RawData_table.IMU_ACC_z,'DisplayName','z Muruta');
legend
xlabel('时间 s')
ylabel('加速度m/s^2')
f_figure(40001,'角速度')
subplot(3,1,1)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_x_ASM330*57.3,'DisplayName','x ASM330');
plot(Time,RawData_table.IMU_Gyro_x*57.3,'DisplayName','x Muruta');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
subplot(3,1,2)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_y_ASM330*57.3,'DisplayName','y ASM330');
plot(Time,RawData_table.IMU_Gyro_y*57.3,'DisplayName','y Muruta');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')
subplot(3,1,3)
hold on
grid on
plot(Time,RawData_table.IMU_Gyro_z_ASM330*57.3,'DisplayName','z ASM330');
plot(Time,RawData_table.IMU_Gyro_z*57.3,'DisplayName','z Muruta');
legend
xlabel('时间 s')
ylabel('角速度 deg/s')

%
% [Pos_Local_INS] = llh2local_V2([GNSS_Lati_Fusion,GNSS_Longi_Fusion,ones(size(GNSS_Longi_Fusion))],[31.28,121.2,0]);
% % [Pos_Local_KVH_Post] = llh2local_V2([Pos_lat.signals.values(:,2),Pos_long.signals.values(:,2),ones(size(Pos_lat.signals.values(:,1)))],[31.28,121.2,0]);
% [Pos_Local_GNSS] = llh2local_V2([GNSS_Lati,GNSS_Longi,ones(size(GNSS_Longi))],[31.28,121.2,0]);
% % [Pos_Local_INS_Smooth] = llh2local_V2([Pos_lat.signals.values(:,6),Pos_long.signals.values(:,6),ones(size(Pos_lat.signals.values(:,1)))],[31.28,121.2,0]);
%
% f_figure(1032,'Local Trajactory')
%     hold on
%     grid on
%     plot3(Pos_Local_INS(:,1),Pos_Local_INS(:,2),Time,'.','DisplayName','INS');
%     plot3(Pos_Local_GNSS(:,1),Pos_Local_GNSS(:,2),Time,'.','DisplayName','GNSS')
% %     plot3(Pos_Local_INS_Smooth(:,1),Pos_Local_INS_Smooth(:,2),Pos_long.time + First_Start_Plot,'.','DisplayName','INS Smooth')
%     legend
%     xlabel('x m')
%     ylabel('y m')
%
% %     CRC校验
% % function ulCRC = CRC32Value(input)
% %     global CRC32_POLYNOMIAL;
% %     ulCRC = uint32(input);
% %     for i = 1:8
% %         if bitand(ulCRC, 1) == 1
% %             ulCRC = bitxor(bitshift(ulCRC, -1), CRC32_POLYNOMIAL); % 负数右移，正数左移
% %         else
% %             ulCRC = bitshift(ulCRC, -1);
% %         end
% %     end
% % end
% %
% %
% % function ulCRC = CalculateBlockCRC32(ulCount, ucBuffer)
% %     ulTemp1 = uint32(0);
% %     ulTemp2 = uint32(0);
% %     ulCRC = uint32(0);
% %     for i = 1:ulCount
% %         ulTemp1 = bitand(bitshift(ulCRC, -8), uint32(hex2dec("00FFFFFF")));
% %         ulTemp2 = CRC32Value(bitand(bitxor(ulCRC, uint32(abs(ucBuffer(i)))), uint32(hex2dec("FF"))));
% %         ulCRC = bitxor(ulTemp1, ulTemp2);
% %     end
% % end
