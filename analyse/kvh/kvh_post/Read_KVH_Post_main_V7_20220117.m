clear
close all
addpath(genpath('Functions'))
addpath(genpath('PlotFiles'))

% FileFolder = 'D:\ExperimentData\KVH1750\Rover\20220111\Project_files\';
FileFolder = 'G:\data\KVH1750\Rover\20220303\data1\';
% SaveFileFolder = 'G:\data\KVH1750\Rover\20220224\data1\';
SaveFileFolder = FileFolder;
FileName = 'KVH_1';

if exist(SaveFileFolder(1:end-1),'dir')==0
   mkdir(SaveFileFolder(1:end-1));
end
SaveFileName = strcat(SaveFileFolder,FileName,'_Post.mat');

FileName_PVA = strcat(FileFolder,FileName,'_PVA.txt');
% FileName_PVA_GNSS = strcat(FileFolder,FileName,'_PVA_GNSS.txt');
FileName_Bias = strcat(FileFolder,FileName,'_Bias.txt');
FileName_IMU = strcat(FileFolder,FileName,'_IMU.txt');
FileName_Std = strcat(FileFolder,FileName,'_Std.txt');

tic
    File_lines =  Fcn_read_txt_lines(FileName_PVA); % 读取文件的行数
    startRow = 22;  %需要调节
%     endRow = File_lines - 7; 
    endRow = File_lines - 10; 

    Data_PVA = importfile_pva_tab(FileName_PVA, startRow, endRow);
    Data_Bias = importfile_bias_tab(FileName_Bias, startRow, endRow);
    Data_IMU = importfile_IMU_tab(FileName_IMU, startRow, endRow);
    Data_Std = importfile_std_tab(FileName_Std, startRow+4, endRow+4);  %std部分多几行

    KVH_Post = [Data_PVA(:,:), Data_Bias(:,2:end), Data_IMU(:,2:end), Data_Std(:,2:end)];
    SaveEnable = 1;
    if SaveEnable
        if exist(SaveFileFolder(1:end-1),'dir') == 0
            makdir(SaveFileFolder);
        end
        save(SaveFileName,'KVH_Post')
        disp(SaveFileName)
    end
    disp(strcat('All Data has been save in mat file: ',' ',SaveFileName))
    clear startRow endRow Data_PVA Data_Bias Data_IMU  Data_Std File_lines FileName_PVA FileName_Bias FileName_IMU FileName_Std save_FileName FileName
toc

%% 画图
KVH_Post_Plot_V1
%     plot( KVH_Post_1.Longitude, KVH_Post_1.Latitude, '.','DisplayName','位置')
%%

f_figure(31,'Heading Course')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.Heading),'DisplayName','Heading')
%     plot( KVH_Post.LocalTime, (KVH_Post.GPSCOG),'DisplayName','Course')
    plot( KVH_Post.LocalTime, deg0_360t0__180_180(KVH_Post.GPSCOG),'DisplayName','Course 0 180')
    legend
    grid on
 
% f_figure(321,'Heading')
%     hold on
%     plot( KVH_Post.LocalTime, f_360_minus(KVH_Post.Heading - deg0_360t0__180_180(KVH_Post.GPSCOG)),'DisplayName','航向角安装角误差')
%     legend
%     grid on

    %%
%     [Pos_Local_KVH_Post] = llh2local_V2([ KVH_Post.Latitude, KVH_Post.Longitude,KVH_Post.HEll],[31.28,121.2,0]);
%     Pos_offset = [0.1000,-0.3000];
%     figure(20210707)
%     hold on
%     plot(Pos_Local_KVH_Post(:,1)-Pos_offset(1),Pos_Local_KVH_Post(:,2)-Pos_offset(2),'.','DisplayName','KVH_Post')
%     xlabel('Local x m')
%     ylabel('Local y m')


