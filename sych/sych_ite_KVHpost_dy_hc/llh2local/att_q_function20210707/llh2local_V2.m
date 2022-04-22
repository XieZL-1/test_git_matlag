function [xyz_local] = llh2local_V2(llh,xyz0_deg)
%【xzl20210707】把纬经高转化为起点为原点，东北天为方向的局部坐标
%   llh = [lati; longi; height];  %单位 度 米
%   xyz_local = [x; y; z];  %单位 米
%   xyz0_deg 为初始点的坐标，如果没有输入则默认为llh的第一行

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