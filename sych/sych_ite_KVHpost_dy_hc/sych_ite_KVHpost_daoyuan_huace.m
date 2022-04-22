%% ����ʵ������
clear
close all
addpath('llh2local\att_q_function20210707'); %


% kvh
KVH_sych_enable = 1;
load('G:\data\KVH1750\Rover\20220111\Data_BMAW18200029H_16_20220111_010504_new.mat')    % kvhʵʱ���ݣ�������ȡIMU
load('G:\data\KVH1750\Rover\20220111\KVH_20220111_010504_Post.mat')    % kvh��������

% daoyuan
daoyuan_sych_enable = 1;
load('G:\data\UART_Data\20220303\Daoyuan\r20220303001_1.mat')

% huace
huace_sych_enable = 1;
load('G:\data\UART_Data\20220303\Huace\20220303093833_Huace.mat')


% gongji
filename = 'G:\data\UART_Data\20220111\422_scha\STM32_JLY52.mat';
load(filename)

idx_dash =  find(filename=='\',1,'last');
SaveFileFolder = filename(1:idx_dash);


%% ��Ҫ����
g = 9.8;
% bias_gyro_z_gongji = 2e-3 ;    %rad ��ƫ���������Ҫ�ֶ�����
% bias_gyro_z_gongji = 1.5e-3 ;    %rad ��ƫ���������Ҫ�ֶ�����
bias_gyro_z_gongji = -0.7e-3 ;    %rad ��ƫ���������Ҫ�ֶ�����

T_error_a = -0.01:1e-3:0.05;    %ʱ�����������Χ

% [xzl20220117] ���ܴ��ڿ�������ζϵ��ϵ磬�����ݴ洢û�ϣ���������������,
cut_start = 2.1e4;
cut_end = size(RawData_table,1);
RawData_table = RawData_table(cut_start:cut_end, :);     %��ȡ������Ҫ����
Time = Time(cut_start:cut_end);

Time = Time - Time(1);
Time_temp = round(Time*1000);
Time = Time_temp *0.001;     %����ʱ�侫�����

figure
subplot(4,1,1)
plot(RawData_table.main_10ms_Count,'DisplayName','main_10ms_Count')
legend
subplot(4,1,2)
plot(diff(RawData_table.main_10ms_Count),'DisplayName','diff main_10ms_Count')
legend
subplot(4,1,3)
plot(diff(RawData_table.Pos_time),'DisplayName','diff Pos_time')
legend
subplot(4,1,4)
plot(diff(RawData_table.Pos_time*1000 + RawData_table.GNSS_Time_ms_g_u32),'DisplayName','diff Pos_time + GNSS_Time')
legend
if (sum(diff(RawData_table.main_10ms_Count)<0)>0 || max(diff(RawData_table.Pos_time))>1 || max(diff(RawData_table.Pos_time*1000 + RawData_table.GNSS_Time_ms_g_u32))>20)
    error('��������ζϵ��ϵ磬���������');
end

GNSS_postime_gongji = RawData_table.Pos_time*1000 + RawData_table.GNSS_Time_ms_g_u32;     %�ĳ�100Hzʱ�����

%% ####################################################################
%% kvhpost
if KVH_sych_enable ==1
    T_error_a_kvhpost = T_error_a;
    T_error_min_kvhpost = 0;
    err_g_min_kvhpost = inf;
    for i = 1:length(T_error_a_kvhpost)
        T_error_kvhpost = T_error_a_kvhpost(i);
        if mod(i/length(T_error_a_kvhpost),0.2)<= 1/length(T_error_a_kvhpost)
            disp(T_error_kvhpost);
        end
        
        %% kvh RAWIMUʱ��ͬ��
        
        temp = find(RawData_table.Pos_time>0,1);
        KVH_RAWIMU_time = KVH1750.RAWIMU.time_RAWIMU;
        
        temp_1 = find( KVH_RAWIMU_time >= GNSS_postime_gongji(temp(1))/1000,1);
        temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_RAWIMU_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_RAWIMU_time(temp_1))),1,'last' ) + temp(1)-1;
        
        T_RAWIMU = KVH_RAWIMU_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_kvhpost;
        
        Time_temp = round(T_RAWIMU*1000);
        T_RAWIMU = Time_temp *0.001;     %����ʱ�侫�����
        %% kvhpost ʱ��ͬ��
        leap_second = 18;   %����
        
        temp = find(RawData_table.Pos_time>0,1);
        KVH_post_time = KVH_Post.LocalTime;
        
        temp_1 = find( KVH_post_time >= GNSS_postime_gongji(temp(1))/1000,1);
        temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_post_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_post_time(temp_1))),1,'last' ) + temp(1)-1;
        
        
        T_post = KVH_post_time +leap_second - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_kvhpost;
        
        Time_temp = round(T_post*1000);
        T_post = Time_temp *0.001;     %����ʱ�侫�����
        %% ��simulink����ʱ��ͬ�����洢����
        
        Sim_T = Time(end);
        sim('time_gongji_KVHpost.slx',Sim_T)
        
        k1 = find(KVH_gongji.signals.values(:,6) ~=0,1);
        k2 = find(KVH_gongji.signals.values(:,6) ~=0,1,'last');
        
        err_g = mean(abs(RawData_table.IMU_Gyro_z(k1:k2) - bias_gyro_z_gongji - KVH_gongji.signals.values(k1:k2,6)));
        if err_g< err_g_min_kvhpost
            err_g_min_kvhpost = err_g;
            T_error_min_kvhpost = T_error_a_kvhpost(i);
            
            RawData_table.KVH_ACC_x = KVH_gongji.signals.values(:,1);
            RawData_table.KVH_ACC_y = KVH_gongji.signals.values(:,2);
            RawData_table.KVH_ACC_z = KVH_gongji.signals.values(:,3);
            RawData_table.KVH_Gyro_x = KVH_gongji.signals.values(:,4);
            RawData_table.KVH_Gyro_y = KVH_gongji.signals.values(:,5);
            RawData_table.KVH_Gyro_z = KVH_gongji.signals.values(:,6);
            RawData_table.KVH_Pitch = KVH_gongji.signals.values(:,7);
            RawData_table.KVH_Roll = KVH_gongji.signals.values(:,8);
            RawData_table.KVH_Yaw = KVH_gongji.signals.values(:,9);
            RawData_table.KVH_Vel_E = KVH_gongji.signals.values(:,10);
            RawData_table.KVH_Vel_N = KVH_gongji.signals.values(:,11);
            RawData_table.KVH_Vel_U = KVH_gongji.signals.values(:,12);
            RawData_table.KVH_Pos_N = KVH_gongji.signals.values(:,13);
            RawData_table.KVH_Pos_E = KVH_gongji.signals.values(:,14);
            RawData_table.KVH_Pos_U = KVH_gongji.signals.values(:,15);
            RawData_table.KVH_Pitch_sigma = KVH_gongji.signals.values(:,16);
            RawData_table.KVH_Roll_sigma = KVH_gongji.signals.values(:,17);
            RawData_table.KVH_Yaw_sigma = KVH_gongji.signals.values(:,18);
            RawData_table.KVH_Vel_E_sigma = KVH_gongji.signals.values(:,19);
            RawData_table.KVH_Vel_N_sigma = KVH_gongji.signals.values(:,20);
            RawData_table.KVH_Vel_U_sigma = KVH_gongji.signals.values(:,21);
            RawData_table.KVH_Pos_E_sigma = KVH_gongji.signals.values(:,22);
            RawData_table.KVH_Pos_N_sigma = KVH_gongji.signals.values(:,23);
            RawData_table.KVH_Pos_U_sigma = KVH_gongji.signals.values(:,24);
            RawData_table.KVH_Time = KVH_gongji.signals.values(:,25);
            
        end
        
    end
    disp('kvhpost final')
    disp(err_g_min_kvhpost);
    disp(T_error_min_kvhpost);
    %% kvh RAWIMU finalʱ��ͬ��
    temp = find(RawData_table.Pos_time>0,1);
    KVH_RAWIMU_time = KVH1750.RAWIMU.time_RAWIMU;
    
    temp_1 = find( KVH_RAWIMU_time >= GNSS_postime_gongji(temp(1))/1000,1);
    temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_RAWIMU_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_RAWIMU_time(temp_1))),1,'last' ) + temp(1)-1;
    
    T_RAWIMU = KVH_RAWIMU_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_min_kvhpost;
    
    Time_temp = round(T_RAWIMU*1000);
    T_RAWIMU = Time_temp *0.001;     %����ʱ�侫�����
    %% kvhpost finalʱ��ͬ��
    leap_second = 18;   %����
    
    temp = find(RawData_table.Pos_time>0,1);
    KVH_post_time = KVH_Post.LocalTime;
    
    temp_1 = find( KVH_post_time >= GNSS_postime_gongji(temp(1))/1000,1);
    temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_post_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -KVH_post_time(temp_1))),1,'last' ) + temp(1)-1;
    
    T_post = KVH_post_time +leap_second - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_min_kvhpost;
    
    Time_temp = round(T_post*1000);
    T_post = Time_temp *0.001;     %����ʱ�侫�����
end

%% ###############################################################
%% daoyuan
if daoyuan_sych_enable ==1
    T_error_a_daoyuan = T_error_a;
    T_error_min_daoyuan = 0;
    err_g_min_daoyuan = inf;
    for i = 1:length(T_error_a_daoyuan)
        T_error = T_error_a_daoyuan(i);
        if mod(i/length(T_error_a_daoyuan),0.2)<= 1/length(T_error_a_daoyuan)
            disp(T_error);
        end
        
        %% daoyuan INSʱ��ͬ��
        temp = find(RawData_table.Pos_time>0,1);
        
        daoyuan_INS_time = data_daoyuan.INSData.ts;
        
        temp_1 = find( daoyuan_INS_time >= GNSS_postime_gongji(temp(1))/1000, 1);
        temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_INS_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_INS_time(temp_1))),1,'last'  ) + temp(1)-1;
        
        T_INS_daoyuan = daoyuan_INS_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error;
        
        Time_temp = round(T_INS_daoyuan*1000);
        T_INS_daoyuan = Time_temp *0.001;     %����ʱ�侫�����
        
        %% daoyuan gnss ʱ��ͬ��
        
        temp = find(RawData_table.Pos_time>0,1);
        daoyuan_GNSS_time = data_daoyuan.GPSData.ts;
        
        temp_1 = find( daoyuan_GNSS_time >= GNSS_postime_gongji(temp(1))/1000,1);
        temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_GNSS_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_GNSS_time(temp_1))),1,'last'  ) + temp(1)-1;
        
        T_GNSS_daoyuan = daoyuan_GNSS_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error;
        
        Time_temp = round(T_GNSS_daoyuan*1000);
        T_GNSS_daoyuan = Time_temp *0.001;     %����ʱ�侫�����
        
        %% daoyuan ���н���ʱ��ͬ��
        temp = find(RawData_table.Pos_time>0,1);
        DY_INS_time = RawData_table_DY.GNSS_Time/1000;
        
        temp_1 = find( DY_INS_time >= GNSS_postime_gongji(temp(1))/1000, 1);
        temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -DY_INS_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -DY_INS_time(temp_1))) ,1,'last' ) + temp(1)-1;
        
        T_INS_DY = DY_INS_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error;
        
        Time_temp = round(T_INS_DY*1000);
        T_INS_DY = Time_temp *0.001;     %����ʱ�侫�����
        
        %% ��simulink����ʱ��ͬ�����洢����
        
        Sim_T = Time(end);
        sim('time_gongji_daoyuan.slx',Sim_T)
        
        k1 = find(daoyuan_gongji.signals.values(:,6) ~=0,1);
        k2 = find(daoyuan_gongji.signals.values(:,6) ~=0,1,'last');
        err_g = mean(abs(RawData_table.IMU_Gyro_z(k1:k2) - bias_gyro_z_gongji - daoyuan_gongji.signals.values(k1:k2,6)));
        if err_g< err_g_min_daoyuan
            err_g_min_daoyuan = err_g;
            T_error_min_daoyuan = T_error_a_daoyuan(i);
            
            RawData_table.daoyuan_ACC_x = daoyuan_gongji.signals.values(:,1);
            RawData_table.daoyuan_ACC_y = daoyuan_gongji.signals.values(:,2);
            RawData_table.daoyuan_ACC_z = daoyuan_gongji.signals.values(:,3);
            RawData_table.daoyuan_Gyro_x = daoyuan_gongji.signals.values(:,4);
            RawData_table.daoyuan_Gyro_y = daoyuan_gongji.signals.values(:,5);
            RawData_table.daoyuan_Gyro_z = daoyuan_gongji.signals.values(:,6);
            RawData_table.daoyuan_Pitch = daoyuan_gongji.signals.values(:,7);
            RawData_table.daoyuan_Roll = daoyuan_gongji.signals.values(:,8);
            RawData_table.daoyuan_Yaw = daoyuan_gongji.signals.values(:,9);
            RawData_table.daoyuan_Vel_E = daoyuan_gongji.signals.values(:,10);
            RawData_table.daoyuan_Vel_N = daoyuan_gongji.signals.values(:,11);
            RawData_table.daoyuan_Vel_U = daoyuan_gongji.signals.values(:,12);
            RawData_table.daoyuan_Pos_N = daoyuan_gongji.signals.values(:,13);
            RawData_table.daoyuan_Pos_E = daoyuan_gongji.signals.values(:,14);
            RawData_table.daoyuan_Pos_U = daoyuan_gongji.signals.values(:,15);
            
            RawData_table.daoyuan_GNSS_NumSV = daoyuan_gongji.signals.values(:,16);
            RawData_table.daoyuan_GNSS_heading = daoyuan_gongji.signals.values(:,17);
            RawData_table.daoyuan_GNSS_lat = daoyuan_gongji.signals.values(:,18);
            RawData_table.daoyuan_Time = daoyuan_gongji.signals.values(:,19);
            % daoyuan ��־λ, ��Ҫʹ�ô���־λ��ȡ��daoyuan����
            RawData_table.daoyuan_V_R = daoyuan_gongji.signals.values(:,20);
            RawData_table.daoyuan_P_R = daoyuan_gongji.signals.values(:,21);
            RawData_table.daoyuan_R2_R = daoyuan_gongji.signals.values(:,22);
            RawData_table.daoyuan_H2_R = daoyuan_gongji.signals.values(:,23);
            % ���н������ݣ���Ҫ���н����ĵ�Զ����
            RawData_table.daoyuan_Pitch_std = daoyuan_gongji.signals.values(:,27);
            RawData_table.daoyuan_Roll_std = daoyuan_gongji.signals.values(:,28);
            RawData_table.daoyuan_Yaw_std = daoyuan_gongji.signals.values(:,29);
            RawData_table.daoyuan_Vel_E_std = daoyuan_gongji.signals.values(:,30);
            RawData_table.daoyuan_Vel_N_std = daoyuan_gongji.signals.values(:,31);
            RawData_table.daoyuan_Vel_U_std = daoyuan_gongji.signals.values(:,32);
            RawData_table.daoyuan_Pos_N_std = daoyuan_gongji.signals.values(:,33);
            RawData_table.daoyuan_Pos_E_std = daoyuan_gongji.signals.values(:,34);
            RawData_table.daoyuan_Pos_U_std = daoyuan_gongji.signals.values(:,35);
            
        end
    end
    disp('daoyuan final��');
    disp(err_g_min_daoyuan);
    disp(T_error_min_daoyuan);
    
    %% daoyuan INS final ʱ��ͬ��
    temp = find(RawData_table.Pos_time>0,1);
    
    daoyuan_INS_time = data_daoyuan.INSData.ts;
    
    temp_1 = find( daoyuan_INS_time >= GNSS_postime_gongji(temp(1))/1000, 1);
    temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_INS_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_INS_time(temp_1))),1,'last'  ) + temp(1)-1;
    
    T_INS_daoyuan = daoyuan_INS_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_min_daoyuan;
    
    Time_temp = round(T_INS_daoyuan*1000);
    T_INS_daoyuan = Time_temp *0.001;     %����ʱ�侫�����
    
    %% daoyuan gnss  final ʱ��ͬ��
    
    temp = find(RawData_table.Pos_time>0,1);
    
    temp_1 = find( daoyuan_GNSS_time >= GNSS_postime_gongji(temp(1))/1000,1);
    temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_GNSS_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -daoyuan_GNSS_time(temp_1))),1,'last'  ) + temp(1)-1;
    
    T_GNSS_daoyuan = daoyuan_GNSS_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_min_daoyuan;
    
    Time_temp = round(T_GNSS_daoyuan*1000);
    T_GNSS_daoyuan = Time_temp *0.001;     %����ʱ�侫�����
    
    %% daoyuan  final ���н���ʱ��ͬ��
    temp = find(RawData_table.Pos_time>0,1);
    DY_INS_time = RawData_table_DY.GNSS_Time/1000;
    
    temp_1 = find( DY_INS_time >= GNSS_postime_gongji(temp(1))/1000, 1);
    temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -DY_INS_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -DY_INS_time(temp_1))) ,1,'last' ) + temp(1)-1;
    
    T_INS_DY = DY_INS_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_min_daoyuan;
    
    Time_temp = round(T_INS_DY*1000);
    T_INS_DY = Time_temp *0.001;     %����ʱ�侫�����
end

%% ###############################################################
%% huace
if huace_sych_enable ==1
    T_error_a_huace = T_error_a;
    % T_error_a = 0.031;
    T_error_min_huace = 0;
    err_g_min_huace = inf;
    for i = 1:length(T_error_a_huace)
        T_error = T_error_a_huace(i);
        if mod(i/length(T_error_a_huace),0.2)<= 1/length(T_error_a_huace)
            disp(T_error);
        end
        
        %% huace INSʱ��ͬ��
        
        temp = find(RawData_table.Pos_time>0,1);
        temp0 = find(data_huace.time_GPCHC0>0,1);
        huace_GPCHC_time = data_huace.time_GPCHC0;
        
        temp_1 = find( huace_GPCHC_time >= GNSS_postime_gongji(temp(1))/1000, 1);
        temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -huace_GPCHC_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -huace_GPCHC_time(temp_1))),1,'last' ) + temp(1)-1;
        
        T_GPCHC = huace_GPCHC_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error;
        
        Time_temp = round(T_GPCHC*1000);
        T_GPCHC = Time_temp *0.001;     %����ʱ�侫�����
        %% ��simulink����ʱ��ͬ�����洢����
        
        Sim_T = Time(end);
        sim('time_gongji_huace.slx',Sim_T)
        
        k1 = find(huace_gongji.signals.values(:,6) ~=0,1);
        k2 = find(huace_gongji.signals.values(:,6) ~=0,1,'last');
        err_g = mean(abs(RawData_table.IMU_Gyro_z(k1:k2) - bias_gyro_z_gongji - huace_gongji.signals.values(k1:k2,6)));
        if err_g< err_g_min_huace
            err_g_min_huace = err_g;
            T_error_min_huace = T_error_a_huace(i);
            
            RawData_table.huace_ACC_x = huace_gongji.signals.values(:,1);
            RawData_table.huace_ACC_y = huace_gongji.signals.values(:,2);
            RawData_table.huace_ACC_z = huace_gongji.signals.values(:,3);
            RawData_table.huace_Gyro_x = huace_gongji.signals.values(:,4);
            RawData_table.huace_Gyro_y = huace_gongji.signals.values(:,5);
            RawData_table.huace_Gyro_z = huace_gongji.signals.values(:,6);
            RawData_table.huace_Pitch = huace_gongji.signals.values(:,7);
            RawData_table.huace_Roll = huace_gongji.signals.values(:,8);
            RawData_table.huace_Yaw = huace_gongji.signals.values(:,9);
            RawData_table.huace_Vel_E = huace_gongji.signals.values(:,10);
            RawData_table.huace_Vel_N = huace_gongji.signals.values(:,11);
            RawData_table.huace_Vel_U = huace_gongji.signals.values(:,12);
            RawData_table.huace_Pos_N = huace_gongji.signals.values(:,13);
            RawData_table.huace_Pos_E = huace_gongji.signals.values(:,14);
            RawData_table.huace_Pos_U = huace_gongji.signals.values(:,15);
            
            RawData_table.huace_GNSS_NumSV = huace_gongji.signals.values(:,16);
            RawData_table.huace_Status_GNSS = huace_gongji.signals.values(:,17);
            RawData_table.huace_Status_sys = huace_gongji.signals.values(:,18);
            RawData_table.huace_Time = huace_gongji.signals.values(:,19);
            RawData_table.huace_vehicle_speed = huace_gongji.signals.values(:,20);
            
        end
    end
    disp('huace final��');
    disp(err_g_min_huace);
    disp(T_error_min_huace);
    
    %% huace INS final ʱ��ͬ��
    
    temp = find(RawData_table.Pos_time>0,1);
    temp0 = find(data_huace.time_GPCHC0>0,1);
    huace_GPCHC_time = data_huace.time_GPCHC0;
    
    temp_1 = find( huace_GPCHC_time >= GNSS_postime_gongji(temp(1))/1000, 1);
    temp_2 = find( abs(GNSS_postime_gongji(temp(1):end)/1000 -huace_GPCHC_time(temp_1)) == min(abs(GNSS_postime_gongji(temp(1):end)/1000 -huace_GPCHC_time(temp_1))),1,'last' ) + temp(1)-1;
    
    T_GPCHC = huace_GPCHC_time - GNSS_postime_gongji(temp_2)/1000 + Time(temp_2) - T_error_min_huace;
    
    Time_temp = round(T_GPCHC*1000);
    T_GPCHC = Time_temp *0.001;     %����ʱ�侫�����
end

%% ###################################################################################
%% ��������
temp = num2str(cut_start);
temp_i = find(temp~='0', 1, 'last');
save_name = [filename(1:end-4), '_', temp(1:temp_i), 'e', num2str(length(temp) - 1)];

if KVH_sych_enable ==1
    save_name = [save_name, '_KVHpost'];
end
if daoyuan_sych_enable ==1
    save_name = [save_name, '_daoyuan'];
end
if huace_sych_enable ==1
    save_name = [save_name, '_huace'];
end

save([save_name, '.mat'], 'RawData_table', 'Time')

%% ��ͼ
figure
hold on
plot(Time,RawData_table.IMU_Gyro_z - bias_gyro_z_gongji,'DisplayName','gongji')
if KVH_sych_enable ==1
    plot(Time,RawData_table.KVH_Gyro_z,'DisplayName','KVH')
end
if daoyuan_sych_enable ==1
    plot(Time,RawData_table.daoyuan_Gyro_z,'DisplayName','daoyuan')
end
if huace_sych_enable ==1
    plot(Time,RawData_table.huace_Gyro_z,'DisplayName','huace')
end
legend
xlabel('ʱ�� s')
ylabel('���ٶ� rad/s')
grid on



%����GNSSγ�ȶԱ�
figure;
hold on
plot(Time,RawData_table.Latitude_GNSS_Double,'DisplayName','gongji GNSS')
if daoyuan_sych_enable ==1
    plot(Time,RawData_table.daoyuan_GNSS_lat,'DisplayName','daoyuan GNSS')
end
if huace_sych_enable ==1
    plot(Time,RawData_table.huace_Pos_N,'DisplayName','huace')
end
legend
xlabel('ʱ�� s')
ylabel('γ��')
grid on

%�������Ǹ����Ա�
figure;
hold on
plot(Time,RawData_table.Soln_SVs_GNSS,'DisplayName','gongji')
plot(Time,RawData_table.Soln_SVs_Ante2_GNSS,'DisplayName','gongji2')
if daoyuan_sych_enable ==1
    plot(Time,RawData_table.daoyuan_GNSS_NumSV,'DisplayName','daoyuan')
end
if huace_sych_enable ==1
    plot(Time,RawData_table.huace_GNSS_NumSV,'DisplayName','huace')
end
legend
xlabel('ʱ�� s')
ylabel('���Ǹ���')
grid on

%�ֲ�λ��
figure
hold on
grid on
[Pos_Local_GNSS] = llh2local_V2([RawData_table.Latitude_GNSS_Double, RawData_table.Longitude_GNSS_Double, ones(size(RawData_table.Longitude_GNSS_Double))],[31.28,121.2,0]);
plot3(Pos_Local_GNSS(:,1),Pos_Local_GNSS(:,2),Time,'.','DisplayName','GNSS');
[Pos_Local_INS] = llh2local_V2([RawData_table.Latitude_Fusion_Double, RawData_table.Longitude_Fusion_Double, ones(size(RawData_table.Latitude_Fusion_Double))],[31.28,121.2,0]);
plot3(Pos_Local_INS(:,1),Pos_Local_INS(:,2),Time,'.','DisplayName','INS');
if KVH_sych_enable ==1
    [Pos_Local_KVH] = llh2local_V2([RawData_table.KVH_Pos_N/pi*180, RawData_table.KVH_Pos_E/pi*180, ones(size(RawData_table.KVH_Pos_N))],[31.28,121.2,0]);
    plot3(Pos_Local_KVH(:,1), Pos_Local_KVH(:,2), Time,'.','DisplayName','KVH')
end
if daoyuan_sych_enable ==1
    [Pos_Local_daoyuan] = llh2local_V2([RawData_table.daoyuan_Pos_N, RawData_table.daoyuan_Pos_E, ones(size(RawData_table.daoyuan_Pos_N))],[31.28,121.2,0]);
    plot3(Pos_Local_daoyuan(:,1), Pos_Local_daoyuan(:,2), Time,'.','DisplayName','daoyuan')
end
if huace_sych_enable ==1
    [Pos_Local_huace] = llh2local_V2([RawData_table.huace_Pos_N, RawData_table.huace_Pos_E, ones(size(RawData_table.huace_Pos_N))],[31.28,121.2,0]);
    plot3(Pos_Local_huace(:,1), Pos_Local_huace(:,2), Time,'.','DisplayName','huace')
end
legend
xlabel('x m')
ylabel('y m')

