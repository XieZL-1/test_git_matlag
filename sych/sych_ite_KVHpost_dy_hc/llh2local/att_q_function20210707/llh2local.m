function [xyz_local] = llh2local(llh)
%# ��xzl20210707����γ����ת��Ϊ���Ϊԭ�㣬������Ϊ����ľֲ�����

%   llh = [lati; longi; height];  %��λ �� ��
%   xyz_local = [x; y; z];  %��λ ��


[xyz_local,q_xyz2nav,dcm0] = llh2m(llh);

xyz_local = (dcm0*(xyz_local'))';
xyz_local = xyz_local - xyz_local(1,:);
% xyz_local = xyz_local - xyz_local(end,:);

end