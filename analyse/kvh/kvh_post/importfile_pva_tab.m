function Data0612MorningV3PVA = importfile_pva_tab(filename, startRow, endRow)
%IMPORTFILE ���ı��ļ��е���ֵ������Ϊ�����롣
%   DATA0612MORNINGV3PVA = IMPORTFILE(FILENAME) ��ȡ�ı��ļ� FILENAME ��Ĭ��ѡ����Χ�����ݡ�
%
%   DATA0612MORNINGV3PVA = IMPORTFILE(FILENAME, STARTROW, ENDROW) ��ȡ�ı��ļ�
%   FILENAME �� STARTROW �е� ENDROW ���е����ݡ�
%
% Example:
%   Data0612MorningV3PVA = importfile('Data_0612_Morning_V3_PVA.txt', 20, 254920);
%
%    ������� TEXTSCAN��

% �� MATLAB �Զ������� 2020/06/13 15:58:22

%% ��ʼ��������
if nargin<=2
    startRow = 20;
    endRow = 254920;
end

%% ÿ���ı��еĸ�ʽ:
%   ��1: ˫����ֵ (%f)
%	��2: ˫����ֵ (%f)
%   ��3: ˫����ֵ (%f)
%	��4: ˫����ֵ (%f)
%   ��5: ˫����ֵ (%f)
%	��6: ˫����ֵ (%f)
%   ��7: ˫����ֵ (%f)
%	��8: ˫����ֵ (%f)
%   ��9: ˫����ֵ (%f)
%	��10: ˫����ֵ (%f)
%   ��11: ˫����ֵ (%f)
% �й���ϸ��Ϣ������� TEXTSCAN �ĵ���
formatSpec = '%9f%15f%15f%13f%10f%10f%10f%15f%15f%15f%f%[^\n\r]';

%% ���ı��ļ���
fileID = fopen(filename,'r');

%% ���ݸ�ʽ��ȡ�����С�
% �õ��û������ɴ˴������õ��ļ��Ľṹ����������ļ����ִ����볢��ͨ�����빤���������ɴ��롣
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

%% �ر��ı��ļ���
fclose(fileID);

%% ���޷���������ݽ��еĺ���
% �ڵ��������δӦ���޷���������ݵĹ�����˲�����������롣Ҫ�����������޷���������ݵĴ��룬�����ļ���ѡ���޷������Ԫ����Ȼ���������ɽű���

%% �����������
Data0612MorningV3PVA = table(dataArray{1:end-1}, 'VariableNames', {'LocalTime','Latitude','Longitude','HEll','VEast','VNorth','VUp','Roll','Pitch','Heading','GPSCOG'});

