function [xyz, q_xyz2nav, dcm_e_n1] = llh2xyz(llh)
%# 【xzl20201206】把纬经高转化为地心直角坐标系, 并获得将该点xyz坐标到该点导航坐标系下坐标的四元数及方向余弦矩阵
%   llh = [lati; longi; height];  %单位 度

%%
Re = 6378137;   %地球长轴半径
f = 1/298.257;  
e = sqrt(2*f-f^2); %偏心率

L = llh(1)/180*pi;
lamda = llh(2)/180*pi;
h = llh(3);

RN = Re/sqrt(1-e^2*sin(L)^2);

xyz = zeros(3,1);
xyz(1) = (RN+h) *cos(L) *cos(lamda);
xyz(2) = (RN+h) *cos(L) *sin(lamda);
xyz(3) = (RN*(1-e^2)+h) *sin(L);

dcm_g_e = [cos(-lamda)  sin(-lamda)    0;
            -sin(-lamda)    cos(-lamda)    0;
            0    0   1] *...
            [cos(-(pi/2-L))  0   -sin(-(pi/2-L));
            0    1   0;
            sin(-(pi/2-L))   0   cos(-(pi/2-L))] *...
            [cos(-pi/2)     sin(-pi/2)  0;
            -sin(-pi/2)     cos(-pi/2)  0;
            0   0   1];     %严共敏书p44
        
%理论上dcm_e_n和dcm_e_n1一样
dcm_e_n = dcm_g_e^(-1);
dcm_e_n1 = [cos(-pi/2)     -sin(-pi/2)     0;
            sin(-pi/2)      cos(-pi/2)  0;
            0   0   1] *...
            [cos(-(pi/2-L))  0   sin(-(pi/2-L));
            0    1   0;
            -sin(-(pi/2-L))   0   cos(-(pi/2-L))] *...
            [cos(-lamda)  -sin(-lamda)    0;
            sin(-lamda)    cos(-lamda)    0;
            0    0   1];
        
q_xyz2nav = m2qua(dcm_e_n1);
