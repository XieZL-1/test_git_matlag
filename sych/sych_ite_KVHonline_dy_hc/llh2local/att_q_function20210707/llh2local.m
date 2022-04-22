function [xyz_local] = llh2local(llh)
%# 【xzl20210707】把纬经高转化为起点为原点，东北天为方向的局部坐标

%   llh = [lati; longi; height];  %单位 度 米
%   xyz_local = [x; y; z];  %单位 米


[xyz_local,q_xyz2nav,dcm0] = llh2m(llh);

xyz_local = (dcm0*(xyz_local'))';
xyz_local = xyz_local - xyz_local(1,:);
% xyz_local = xyz_local - xyz_local(end,:);

end