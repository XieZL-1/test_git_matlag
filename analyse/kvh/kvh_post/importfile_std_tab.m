function Data0612MorningV3Std1 = importfile1_std_tab(filename, startRow, endRow)
%IMPORTFILE1 将文本文件中的数值数据作为矩阵导入。
%   DATA0612MORNINGV3STD1 = IMPORTFILE1(FILENAME) 读取文本文件 FILENAME
%   中默认选定范围的数据。
%
%   DATA0612MORNINGV3STD1 = IMPORTFILE1(FILENAME, STARTROW, ENDROW) 读取文本文件
%   FILENAME 的 STARTROW 行到 ENDROW 行中的数据。
%
% Example:
%   Data0612MorningV3Std1 = importfile1('Data_0612_Morning_V3_Std.txt', 24, 5924);
%
%    另请参阅 TEXTSCAN。

% 由 MATLAB 自动生成于 2020/06/13 16:20:15

%% 初始化变量。
if nargin<=2
    startRow = 24;
    endRow = 5924;
end

%% 每个文本行的格式:
%   列1: 双精度值 (%f)
%	列2: 双精度值 (%f)
%   列3: 双精度值 (%f)
%	列4: 双精度值 (%f)
%   列5: 双精度值 (%f)
%	列6: 双精度值 (%f)
%   列7: 双精度值 (%f)
%	列8: 双精度值 (%f)
%   列9: 双精度值 (%f)
%	列10: 双精度值 (%f)
% 有关详细信息，请参阅 TEXTSCAN 文档。
formatSpec = '%9f%13f%13f%13f%10f%10f%10f%15f%15f%f%[^\n\r]';

%% 打开文本文件。
fileID = fopen(filename,'r');

%% 根据格式读取数据列。
% 该调用基于生成此代码所用的文件的结构。如果其他文件出现错误，请尝试通过导入工具重新生成代码。
textscan(fileID, '%[^\n\r]', startRow(1)-1, 'WhiteSpace', '', 'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'EmptyValue', NaN, 'ReturnOnError', false, 'EndOfLine', '\r\n');
for block=2:length(startRow)
    frewind(fileID);
    textscan(fileID, '%[^\n\r]', startRow(block)-1, 'WhiteSpace', '', 'ReturnOnError', false);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'EmptyValue', NaN, 'ReturnOnError', false, 'EndOfLine', '\r\n');
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% 关闭文本文件。
fclose(fileID);

%% 对无法导入的数据进行的后处理。
% 在导入过程中未应用无法导入的数据的规则，因此不包括后处理代码。要生成适用于无法导入的数据的代码，请在文件中选择无法导入的元胞，然后重新生成脚本。

%% 创建输出变量
Data0612MorningV3Std1 = table(dataArray{1:end-1}, 'VariableNames', {'LocalTime','SDEast','SDNorth','SDHeight','SD_VE','SD_VN','SD_VH','RollSD','PitchSD','HdngSD'});

