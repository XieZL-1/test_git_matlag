%% 从20210509开始，输出的数据长度都是75
% 说明，相比于V7版本的解析文件，主要更新了 bestpostime bestveltime 和
% headingtime的解析，软件版本0.22.XX以后的要用V8去解析
clear;
% % clc;
close all;
addpath('Functions')
set(groot,'defaultLineLineWidth',2)
%% 定义float类型数据的长度，为字节数/4

% global CRC32_POLYNOMIAL;
% CRC32_POLYNOMIAL= uint32(hex2dec("EDB88320"));
% fidin = fopen('data_20211113_182720.bin','rt');
% data = fread(fidin);

%% 需要修改的部分

FileName= 'G:\data\temp\data20220323_1\data_20220323_173725.bin';%
FileName= 'G:\data\temp\data20220323_1\20220323223340.txt';%
FileName= 'G:\data\temp\data20220324\20220324122315.txt';%
FileName= 'G:\data\temp\data20220324\20220324131416.txt';%

%% 是否需要CRC校验

CRC_test = 1;   %是否进行CRC校验 1是需要 0不校验
SaveEnable = 1;
idx_dash =  find(FileName=='\',1,'last');
SaveFileFolder = FileName(1:idx_dash);

Start_T = 0;
End_T = inf;


%% 以下无需修改

tic
FileName2 = 'tmp_data.bin';%导出的临时文件名
disp(FileName);
fid = fopen(FileName,'rb');
data_temp=fread(fid,'uint8');
fclose(fid);

% 上位机保存标志位
i =1;
if ( data_temp(i)==67 && data_temp(i+1)==79 && data_temp(i+2)==77 && data_temp(i+3)==80 && data_temp(i+4)==85 && data_temp(i+5)==84 && data_temp(i+6)==69 && data_temp(i+7)==82)
    data_temp(1:i+7)=[];
    FLAG_computer = 1;
else
    FLAG_computer = 0;
end

if FLAG_computer == 0
%% 按消息头去除错误数据    
    datalength = 75;%
    idx0 = strfind(reshape(data_temp,1,[]),'AA44AA45');
    error0 = sum(uint16(diff(idx0))~=uint16(datalength*4));
    idx0 = idx0(uint16(diff(idx0))>=uint16(datalength*4));
    idx1 = zeros(datalength*4,size(idx0,2));
    for i = 1:datalength*4
        idx1(i,:) = idx0+i-1;
    end
    data_temp = reshape(data_temp(idx1),1,[]);
    %%
    %把洗过的数据重新存为bin文件
    fid = fopen(FileName2,'wb');
    fwrite(fid,data_temp,'uint8');
    fclose(fid);
    
    fid = fopen(FileName2,'rb');
    
    %% 数据处理：经纬度加法、删除多余数据等
    data_temp=[];
    data_temp=fread(fid,[datalength,inf],'float32');
    data_temp=data_temp';
    %
    data2=data_temp;
    data_temp(:,datalength)=[];%删除末尾一个字节的校验数据
    data_temp(:,10)=data_temp(:,10)+data_temp(:,11);%纬度
    data_temp(:,12)=data_temp(:,12)+data_temp(:,13);%经度
    data_temp(:,33)=data_temp(:,33)+data_temp(:,34);%INS纬度
    data_temp(:,35)=data_temp(:,35)+data_temp(:,36);%INS经度
    data_temp(:,36)=[];data_temp(:,34)=[];
    data_temp(:,13)=[];data_temp(:,11)=[];
    data_temp(:,2)=[]; data_temp(:,1)=[];
    %[xzl20211228]接口协议修改
    fid0 = fopen(FileName2,'rb');
    data_temp0=fread(fid0,[datalength*2,inf],'uint16');
    data_temp0=data_temp0';
    data_temp0 = data_temp0(1:size(data_temp,1),:);
    data_temp0(:,(datalength-1)*2+1:(datalength-1)*2+2)=[];%删除末尾一个字节的校验数据
    data_temp0(:,(36-1)*2+1:(36-1)*2+2)=[];data_temp0(:,(32-1)*2+1:(32-1)*2+2)=[];
    data_temp0(:,(13-1)*2+1:(13-1)*2+2)=[];data_temp0(:,(11-1)*2+1:(11-1)*2+2)=[];
    data_temp0(:,(2-1)*2+1:(2-1)*2+2)=[]; data_temp0(:,(1-1)*2+1:(1-1)*2+2)=[];
%         data_temp1 = int16(data_temp0(:,(27-1)*2+1:(27-1)*2+2));
%     data_temp2 = int16(data_temp0(:,(40-1)*2+1:(40-1)*2+2));
%     data_temp3 = int16(data_temp0(:,(41-1)*2+1:(41-1)*2+2));
%     data_temp4 = uint16(data_temp0(:,(42-1)*2+1:(42-1)*2+2));
    data_temp1 = [typecast(uint16(data_temp0(:,(27-1)*2+1)),'int16'), typecast(uint16(data_temp0(:,(27-1)*2+2)),'int16')];
    data_temp2 = [typecast(uint16(data_temp0(:,(40-1)*2+1)),'int16'), typecast(uint16(data_temp0(:,(40-1)*2+2)),'int16')];
    data_temp3 = [typecast(uint16(data_temp0(:,(41-1)*2+1)),'int16'), typecast(uint16(data_temp0(:,(41-1)*2+2)),'int16')];
    data_temp4 = data_temp0(:,(42-1)*2+1:(42-1)*2+2);
%     data_temp1 = typecast(data_temp1,'int16');
    data_temp1 = [double(data_temp1(:,1)), double(data_temp1(:,2))*0.1];
%     data_temp2 = typecast(data_temp2,'uint16');
    data_temp2 = [double(data_temp2(:,1))*0.01, double(data_temp2(:,2))*0.01];
%     data_temp3 = typecast(data_temp3,'uint16');
    data_temp3 = [double(data_temp3(:,1))*0.01, double(data_temp3(:,2))*0.01];
%     data_temp4 = typecast(data_temp4,'uint32');
    data_temp4 = uint32(uint32(data_temp4(:,1)) + uint32(data_temp4(:,2))*2^16);
%     data_temp4 = double([round(mod(data_temp4,2^1)), round(mod(data_temp4,2^2)), round(mod(data_temp4,2^3)), round(mod(data_temp4,2^4)), round(mod(data_temp4,2^5)), round(mod(data_temp4,2^6)), ...
%                 round(mod(data_temp4,2^7)) + round(mod(data_temp4,2^8))*2, round(mod(data_temp4,2^9)) + round(mod(data_temp4,2^10))*2, round(mod(data_temp4,2^11)) + round(mod(data_temp4,2^12))*2, round(mod(data_temp4,2^13)) + round(mod(data_temp4,2^14))*2, ...
%                 round(mod(data_temp4,2^15)), round(mod(data_temp4,2^16))]);
    n4 =16;
    data_temp4_ = zeros(size(data_temp4,1),n4);
    for i = 1:size(data_temp4,1)
        data_temp4_(i,:) = double(bitget(data_temp4(i),1:n4));
    end
    data_temp4_(:,7) = data_temp4_(:,7) + data_temp4_(:,8)*2;
    data_temp4_(:,9) = data_temp4_(:,9) + data_temp4_(:,10)*2;
    data_temp4_(:,11) = data_temp4_(:,11) + data_temp4_(:,12)*2;
    data_temp4_(:,13) = data_temp4_(:,13) + data_temp4_(:,14)*2;
    data_temp4_(:,[8,10,12,14]) = []; 
    data_temp = [data_temp(:,1:26), data_temp1, data_temp(:,28:39), data_temp2, data_temp3, data_temp4_, data_temp(:,43:end)];
    
    sizee=size(data_temp);
    data_temp(sizee(1),:)=[];%最后一行数据可能有空值，整行删掉
    End_T_num = min(End_T*100+1,sizee(1)-1);
    RawData_table = array2table(data_temp(Start_T*100+1:End_T_num,:));
    %下面的变量名称个数应该为数据总数-7，即减去字头2，末尾1，经度2，纬度2列
    RawData_table.Properties.VariableNames ={'IMU_ACC_x', 'IMU_ACC_y', 'IMU_ACC_z', 'IMU_Gyro_x', 'IMU_Gyro_y', ...%每行五个，便于计数
        'IMU_Gyro_z', 'HeadingAngle_GNSS', 'Latitude_GNSS_Double', 'Longitude_GNSS_Double', 'GNSS_Height',...
        'Pos_RMS_G_x', 'Pos_RMS_G_y','GNSS_Height_RMS','Quality_GNSS','VehicleSpeed',  ...
        'Vel_Level_GNSS', 'Course_GNSS', 'Heading_RMS_GNSS', 'Soln_SVs_GNSS', 'Soln_SVs_Ante2_GNSS',...
        'GNSS_Vel_Vertical','GNSS_Vel_Latency', 'Heading_time', 'Pos_time', 'Vel_time',...
        'SteeringAngleValid0', 'SteeringAngleSpeed','SteeringAngle','IMU_TEMP','Latitude_Fusion_Double', 'Longitude_Fusion_Double',...
        'ins_Height', 'ins_VE', 'ins_VN', 'ins_VU', 'ins_Heading_fusion', ...
        'ins_Roll', 'ins_Pitch', 'main_10ms_Count','GNSS_Time_ms_g_u32','FLwheelspeed',...%到此处40个
        'FRwheelspeed','RLwheelspeed','RRwheelspeed','SteeringAngleValid','SteeringAngleSpeedValid', 'FLwheelspeedValid', 'FRwheelspeedValid','RLwheelspeedValid','RRwheelspeedValid', 'FLwheelspeedDir', 'FRwheelspeedDir','RLwheelspeedDir','RRwheelspeedDir', 'BrakeApplied',  'BrakeAppliedValid',  'Torque', 'Array_47','Array_48',...//a~f为还没赋值的量
        'Array_49','Array_50','Array_51','Array_52','GNSS_Delay_ms_u16','PPS_Count_u32','Timmer_add_Count_u16','Timmer_minus_Count_u16',...
        'Array_57','Array_58','Array_59','Array_60','Array_61','Array_62','Array_63','Array_64',...
        'Vehicle_Speed_Ava','Array_66','Array_67','Array_68','Array_69','Array_70','Array_71'};
    % figure(1);
    % plot(data(:,5));
    % figure;
    % plot(RawData_table.main_10ms_Count(2:end)-RawData_table.main_10ms_Count(1:end-1));hold on;
    % CRC校验
    if CRC_test == 1
        fid2 = fopen(FileName2,'rb');
        data_temp3 = fread(fid2,[datalength*4,inf],'uint8');
        data_temp3=data_temp3';%这里就变成了单字节的数据
        sizee=size(data_temp3);
        data_temp3(sizee(1),:)=[];%最后一行数据可能有空值，整行删掉
        crc_32 = uint8(data_temp3(:,297:300));%CRC存储
        data = uint8(data_temp3(:,1:296));%数据存储
        %     data = data';
        crc_323 = reshape(cellstr(dec2hex(crc_32,2)),length(data_temp3(:,1)),4);
        crc_3232 = strcat(crc_323(:,4),crc_323(:,3),crc_323(:,2),crc_323(:,1));
        bool =  CRC_mexupdate_1227(data,crc_3232);
        all = find(bool==0);
        all_error = bool(1:all(1)-1);error = length(all_error);
        RawData_table(all_error , :) = [];
        rightdata = length(RawData_table.IMU_ACC_x(:,1));
        alldata = length(data(:,1));
        fprintf('Number of all data = %d\n',alldata +error0);
        fprintf('Number of wrong data = %d\n',error +error0);
        fprintf('Number of right data = %d\n',rightdata);
        
    end
    
else
%% 按消息头去除错误数据    
    datalength = 78;%
    idx0 = strfind(reshape(data_temp,1,[]),'AA44AA45');
    error0 = sum(uint16(diff(idx0))~=uint16(datalength*4));
    idx0 = idx0(uint16(diff(idx0))>=uint16(datalength*4));
    idx1 = zeros(datalength*4,size(idx0,2));
    for i = 1:datalength*4
        idx1(i,:) = idx0+i-1;
    end
    data_temp = reshape(data_temp(idx1),1,[]);
    %%
    %把洗过的数据重新存为bin文件
    fid = fopen(FileName2,'wb');
    fwrite(fid,data_temp,'uint8');
    fclose(fid);
    
    fid = fopen(FileName2,'rb');
    %% 数据处理：经纬度加法、删除多余数据等
    data_temp=[];
    data_temp=fread(fid,[datalength,inf],'float32');
    data_temp=data_temp';   
    %
    data_temp(:,datalength-3)=[];%删除末尾一个字节的校验数据  %这个是校验位
    data_temp(:,10)=data_temp(:,10)+data_temp(:,11);%纬度
    data_temp(:,12)=data_temp(:,12)+data_temp(:,13);%经度
    data_temp(:,33)=data_temp(:,33)+data_temp(:,34);%INS纬度
    data_temp(:,35)=data_temp(:,35)+data_temp(:,36);%INS经度
    data_temp(:,36)=[];data_temp(:,34)=[];
    data_temp(:,13)=[];data_temp(:,11)=[];
    data_temp(:,2)=[]; data_temp(:,1)=[];
    %[xzl20211228]接口协议修改
    fid0 = fopen(FileName2,'rb');
    data_temp0=fread(fid0,[datalength*2,inf],'uint16');
    data_temp0=data_temp0';
    data_temp0 = data_temp0(1:size(data_temp,1),:);
    data_temp0(:,(datalength-1)*2+1:(datalength-1)*2+2)=[];%删除末尾一个字节的校验数据
    data_temp0(:,(36-1)*2+1:(36-1)*2+2)=[];data_temp0(:,(32-1)*2+1:(32-1)*2+2)=[];
    data_temp0(:,(13-1)*2+1:(13-1)*2+2)=[];data_temp0(:,(11-1)*2+1:(11-1)*2+2)=[];
    data_temp0(:,(2-1)*2+1:(2-1)*2+2)=[]; data_temp0(:,(1-1)*2+1:(1-1)*2+2)=[];
%         data_temp1 = int16(data_temp0(:,(27-1)*2+1:(27-1)*2+2));
%     data_temp2 = int16(data_temp0(:,(40-1)*2+1:(40-1)*2+2));
%     data_temp3 = int16(data_temp0(:,(41-1)*2+1:(41-1)*2+2));
%     data_temp4 = uint16(data_temp0(:,(42-1)*2+1:(42-1)*2+2));
    data_temp1 = [typecast(uint16(data_temp0(:,(27-1)*2+1)),'int16'), typecast(uint16(data_temp0(:,(27-1)*2+2)),'int16')];
    data_temp2 = [typecast(uint16(data_temp0(:,(40-1)*2+1)),'int16'), typecast(uint16(data_temp0(:,(40-1)*2+2)),'int16')];
    data_temp3 = [typecast(uint16(data_temp0(:,(41-1)*2+1)),'int16'), typecast(uint16(data_temp0(:,(41-1)*2+2)),'int16')];
    data_temp4 = data_temp0(:,(42-1)*2+1:(42-1)*2+2);
%     data_temp1 = typecast(data_temp1,'int16');
    data_temp1 = [double(data_temp1(:,1)), double(data_temp1(:,2))*0.1];
%     data_temp2 = typecast(data_temp2,'uint16');
    data_temp2 = [double(data_temp2(:,1))*0.01, double(data_temp2(:,2))*0.01];
%     data_temp3 = typecast(data_temp3,'uint16');
    data_temp3 = [double(data_temp3(:,1))*0.01, double(data_temp3(:,2))*0.01];
%     data_temp4 = typecast(data_temp4,'uint32');
    data_temp4 = uint32(uint32(data_temp4(:,1)) + uint32(data_temp4(:,2))*2^16);
%     data_temp4 = double([round(mod(data_temp4,2^1)), round(mod(data_temp4,2^2)), round(mod(data_temp4,2^3)), round(mod(data_temp4,2^4)), round(mod(data_temp4,2^5)), round(mod(data_temp4,2^6)), ...
%                 round(mod(data_temp4,2^7)) + round(mod(data_temp4,2^8))*2, round(mod(data_temp4,2^9)) + round(mod(data_temp4,2^10))*2, round(mod(data_temp4,2^11)) + round(mod(data_temp4,2^12))*2, round(mod(data_temp4,2^13)) + round(mod(data_temp4,2^14))*2, ...
%                 round(mod(data_temp4,2^15)), round(mod(data_temp4,2^16))]);
    n4 =16;
    data_temp4_ = zeros(size(data_temp4,1),n4);
    for i = 1:size(data_temp4,1)
        data_temp4_(i,:) = double(bitget(data_temp4(i),1:n4));
    end
    data_temp4_(:,7) = data_temp4_(:,7) + data_temp4_(:,8)*2;
    data_temp4_(:,9) = data_temp4_(:,9) + data_temp4_(:,10)*2;
    data_temp4_(:,11) = data_temp4_(:,11) + data_temp4_(:,12)*2;
    data_temp4_(:,13) = data_temp4_(:,13) + data_temp4_(:,14)*2;
    data_temp4_(:,[8,10,12,14]) = []; 
    data_temp = [data_temp(:,1:26), data_temp1, data_temp(:,28:39), data_temp2, data_temp3, data_temp4_, data_temp(:,43:end)];
    
    sizee=size(data_temp);
    data_temp(sizee(1),:)=[];%最后一行数据可能有空值，整行删掉
    End_T_num = min(End_T*100+1,sizee(1)-1);
    RawData_table = array2table(data_temp(Start_T*100+1:End_T_num,:));
    %下面的变量名称个数应该为数据总数-7，即减去字头2，末尾1，经度2，纬度2列
    RawData_table.Properties.VariableNames ={'IMU_ACC_x', 'IMU_ACC_y', 'IMU_ACC_z', 'IMU_Gyro_x', 'IMU_Gyro_y', ...%每行五个，便于计数
        'IMU_Gyro_z', 'HeadingAngle_GNSS', 'Latitude_GNSS_Double', 'Longitude_GNSS_Double', 'GNSS_Height',...
        'Pos_RMS_G_x', 'Pos_RMS_G_y','GNSS_Height_RMS','Quality_GNSS','VehicleSpeed',  ...
        'Vel_Level_GNSS', 'Course_GNSS', 'Heading_RMS_GNSS', 'Soln_SVs_GNSS', 'Soln_SVs_Ante2_GNSS',...
        'GNSS_Vel_Vertical','GNSS_Vel_Latency', 'Heading_time', 'Pos_time', 'Vel_time',...
        'SteeringAngleValid0', 'SteeringAngleSpeed','SteeringAngle','IMU_TEMP','Latitude_Fusion_Double', 'Longitude_Fusion_Double',...
        'ins_Height', 'ins_VE', 'ins_VN', 'ins_VU', 'ins_Heading_fusion', ...
        'ins_Roll', 'ins_Pitch', 'main_10ms_Count','GNSS_Time_ms_g_u32','FLwheelspeed',...%到此处40个
        'FRwheelspeed','RLwheelspeed','RRwheelspeed','SteeringAngleValid','SteeringAngleSpeedValid', 'FLwheelspeedValid', 'FRwheelspeedValid','RLwheelspeedValid','RRwheelspeedValid', 'FLwheelspeedDir', 'FRwheelspeedDir','RLwheelspeedDir','RRwheelspeedDir', 'BrakeApplied',  'BrakeAppliedValid',  'Torque', 'Array_47','Array_48',...//a~f为还没赋值的量
        'Array_49','Array_50','Array_51','Array_52','GNSS_Delay_ms_u16','PPS_Count_u32','Timmer_add_Count_u16','Timmer_minus_Count_u16',...
        'Array_57','Array_58','Array_59','Array_60','Array_61','Array_62','Array_63','Array_64',...
        'Vehicle_Speed_Ava','Array_66','Array_67','Array_68','Array_69','Array_70','Array_71','Local_Date','Local_Time','Local_Msec'};
    
    % CRC校验 %目前是以一组数据的形式给进去 如果后期想提速可以以矩阵的形式给进去运算
    if CRC_test == 1
        fid2 = fopen(FileName2,'rb');
        data_temp3 = fread(fid2,[datalength*4,inf],'uint8');
        data_temp3=data_temp3';%这里就变成了单字节的数据
        sizee=size(data_temp3);
        data_temp3(sizee(1),:)=[];%最后一行数据可能有空值，整行删掉
        crc_32 = uint8(data_temp3(:,297:300));%CRC存储
        data = uint8(data_temp3(:,1:296));%数据存储
        %     data = data';
        crc_323 = reshape(cellstr(dec2hex(crc_32,2)),length(data_temp3(:,1)),4);
        crc_3232 = strcat(crc_323(:,4),crc_323(:,3),crc_323(:,2),crc_323(:,1));
        bool =  CRC_mexupdate_1227(data,crc_3232);
        all = find(bool==0);
        all_error = bool(1:all(1)-1);error = length(all_error);
        RawData_table(all_error , :) = [];
        rightdata = length(RawData_table.IMU_ACC_x(:,1));
        alldata = length(data(:,1));
        fprintf('Number of all data = %d\n',alldata +error0);
        fprintf('Number of wrong data = %d\n',error +error0);
        fprintf('Number of right data = %d\n',rightdata);
    end
    
    toc
    
end

main_10ms_Count = RawData_table.main_10ms_Count;
Time = [0:1:length(main_10ms_Count)-1]'*0.01;


%% IMU 数据处理b
RawData_table.IMU_ACC_x_ASM330 = RawData_table.Array_57;
RawData_table.IMU_ACC_y_ASM330 = RawData_table.Array_58;
RawData_table.IMU_ACC_z_ASM330 = RawData_table.Array_59;
RawData_table.IMU_Gyro_x_ASM330 = RawData_table.Array_60;
RawData_table.IMU_Gyro_y_ASM330 = RawData_table.Array_61;
RawData_table.IMU_Gyro_z_ASM330 = RawData_table.Array_62;
RawData_table.SCHA634_UNO_TEMP = RawData_table.Array_63;
RawData_table.SCHA634_DUE_TEMP = RawData_table.Array_64;


%%
RawData_table.Pos_time = RawData_table.Pos_time.*65536 + RawData_table.Vel_time;
VelTimeError_u32 = fix(RawData_table.Heading_time/65536);
HeadingTimeError_u32 = RawData_table.Heading_time - VelTimeError_u32*65536;
RawData_table.Vel_time = RawData_table.Pos_time - VelTimeError_u32;
RawData_table.Heading_time = RawData_table.Pos_time - HeadingTimeError_u32;
% for i = 1:length(HeadingTimeError_u32)
%     if HeadingTimeError_u32(i) == 1
%         RawData_table.Heading_time(i) = 0;
%     end
% end
RawData_table.Pos_time = RawData_table.Pos_time/1000;
RawData_table.Vel_time = RawData_table.Vel_time/1000;
RawData_table.Heading_time = RawData_table.Heading_time/1000;


hold on
plot(Time,RawData_table.IMU_Gyro_z,'DisplayName','GJ_z');
%     plot(Rawtable_1026_3.Time+6,-Rawtable_1026_3.RATE_Z,'DisplayName','lingsi_z');


% 处理杆臂数据
fileLever = fopen([ FileName(1:idx_dash),'Lever.txt']);
C = textscan(fileLever,'%s %f %f %f','Delimiter',{'\n','\r'});
fclose(fileLever);
lever = single([C{2};C{3};C{4}]);

%% 存储数据

if SaveEnable
    if exist(SaveFileFolder(1:end-1),'dir')==0
        mkdir(SaveFileFolder(1:end-1));
    end
end

if SaveEnable
    data_num = FileName(length(FileName)-17:length(FileName)-4);
    save_name = strcat(SaveFileFolder,'STM32_',data_num);
end
if SaveEnable
    save(save_name,'RawData_table','Time','lever');
    disp('将数据存放在如下位置：')
    disp(save_name)
else
    disp('这组解析后的数据没有保存')
end

%% 处理经纬度数据
Pos_ratio_longi = 5.519e6*deg2rad(1);
Pos_ratio_lati = 6.3515e6*deg2rad(1);
pos_local_x0 = 1.212088647000000e+02;%Pos_long_9250_4.signals.values(1,1);
pos_local_y0 = 31.289691800000000;%Pos_lat_9250_4.signals.values(1,1);

GNSS_Lati = RawData_table.Latitude_GNSS_Double;
GNSS_Longi = RawData_table.Longitude_GNSS_Double;
GNSS_Lati_Fusion = RawData_table.Latitude_Fusion_Double;
GNSS_Longi_Fusion = RawData_table.Longitude_Fusion_Double;

GNSS_local_x = (GNSS_Longi- pos_local_x0)*Pos_ratio_longi;
GNSS_local_y = (GNSS_Lati - pos_local_y0)*Pos_ratio_lati;


GNSS_local_x_Fusion = (GNSS_Longi_Fusion- pos_local_x0)*Pos_ratio_longi;
GNSS_local_y_Fusion = (GNSS_Lati_Fusion - pos_local_y0)*Pos_ratio_lati;


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
temp = A<-100;
A(temp) = 1;
temp = find(A>100);
A(temp) = 1;
A(end) = 1;
temp = find(A>15);
disp(temp)

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
% f_figure(13,'PPS&时钟调节标志')
% subplot(2,1,1)
% hold on
% grid on
% plot(Time,RawData_table.PPS_Count_u32,'DisplayName','PPS_Count_u32');
% xlabel('时间 s')
% ylabel('PPS时间 s')
% legend
% subplot(2,1,2)
% hold on
% grid on
% plot(Time,RawData_table.Timmer_add_Count_u16,'DisplayName','Timmer_add_Count_u16');
% plot(Time,RawData_table.Timmer_minus_Count_u16,'DisplayName','Timmer_minus_Count_u16');
% legend
% xlabel('时间 s')
% ylabel('调整次数')
%%
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
plot(Time,RawData_table.Array_63,'DisplayName','IMU _UNO acc y_gyro TEMP');
plot(Time,RawData_table.Array_64,'DisplayName','IMU _DUE gyro TEMP');
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
VSPD_Factor = 0.05625;
VSPD_Factor = 0.01; % tulingplus
%     VSPD_Factor = 0.03125;
%     VSPD_Factor = 0.0316;%标定数值
VSPD_Factor = 0.1; % E50 kph 标定值
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

%     CRC校验
% function ulCRC = CRC32Value(input)
%     global CRC32_POLYNOMIAL;
%     ulCRC = uint32(input);
%     for i = 1:8
%         if bitand(ulCRC, 1) == 1
%             ulCRC = bitxor(bitshift(ulCRC, -1), CRC32_POLYNOMIAL); % 负数右移，正数左移
%         else
%             ulCRC = bitshift(ulCRC, -1);
%         end
%     end
% end
%
%
% function ulCRC = CalculateBlockCRC32(ulCount, ucBuffer)
%     ulTemp1 = uint32(0);
%     ulTemp2 = uint32(0);
%     ulCRC = uint32(0);
%     for i = 1:ulCount
%         ulTemp1 = bitand(bitshift(ulCRC, -8), uint32(hex2dec("00FFFFFF")));
%         ulTemp2 = CRC32Value(bitand(bitxor(ulCRC, uint32(abs(ucBuffer(i)))), uint32(hex2dec("FF"))));
%         ulCRC = bitxor(ulTemp1, ulTemp2);
%     end
% end
