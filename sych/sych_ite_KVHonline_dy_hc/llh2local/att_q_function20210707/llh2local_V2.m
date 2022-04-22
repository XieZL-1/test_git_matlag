function [xyz_local] = llh2local_V2(llh,xyz0_deg)
%��xzl20210707����γ����ת��Ϊ���Ϊԭ�㣬������Ϊ����ľֲ�����
%   llh = [lati; longi; height];  %��λ �� ��
%   xyz_local = [x; y; z];  %��λ ��
%   xyz0_deg Ϊ��ʼ������꣬���û��������Ĭ��Ϊllh�ĵ�һ��

if nargin < 2
    [xyz_local,~,dcm0] = llh2m(llh);
    xyz_local = (dcm0*(xyz_local'))';
    xyz0 = xyz_local(1,:);
else
    [xyz0,~,dcm1] = llh2m(xyz0_deg);
    xyz0 = (dcm1*(xyz0'))';
    [xyz_local,~,~] = llh2m(llh);
    xyz_local = (dcm1*(xyz_local'))';
end
xyz_local = xyz_local - xyz0;

end