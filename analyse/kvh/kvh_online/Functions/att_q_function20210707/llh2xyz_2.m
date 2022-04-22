function [xyz_sphere, q_nav2xyz, dcm_n_e1] = llh2xyz_2(llh)
%# 【xzl20210319】把纬经高转化为地心直角坐标系,
%并获得将该点xyz坐标到该点导航坐标系下坐标的四元数及方向余弦矩阵，即直角坐标到导航坐标，也即导航坐标系到直角坐标系的旋转
%# 【xzl20210319】比起【xzl20201206】，理清楚了旋转关系，规范了符号

%   llh = [lati; longi; height];  %单位 度

%%
Re = 6378137;   %地球长轴半径
f = 1/298.257;  
e = sqrt(2*f-f^2); %偏心率

L = llh(1)/180*pi;
lamda = llh(2)/180*pi;
h = llh(3);

RN = Re/sqrt(1-e^2*sin(L)^2);

xyz_sphere = zeros(3,1);
xyz_sphere(1) = (RN+h) *cos(L) *cos(lamda);
xyz_sphere(2) = (RN+h) *cos(L) *sin(lamda);
xyz_sphere(3) = (RN*(1-e^2)+h) *sin(L);
 
dcm_e_g = [cos(-lamda)  sin(-lamda)    0;
            -sin(-lamda)    cos(-lamda)    0;
            0    0   1] *...
            [cos(-(pi/2-L))  0   -sin(-(pi/2-L));
            0    1   0;
            sin(-(pi/2-L))   0   cos(-(pi/2-L))] *...
            [cos(-pi/2)     sin(-pi/2)  0;
            -sin(-pi/2)     cos(-pi/2)  0;
            0   0   1];     %严共敏书p44
        
%理论上dcm_n_e和dcm_n_e1一样
dcm_n_e = dcm_e_g^(-1);
dcm_n_e1 = [cos(-pi/2)     -sin(-pi/2)     0;
            sin(-pi/2)      cos(-pi/2)  0;
            0   0   1] *...
            [cos(-(pi/2-L))  0   sin(-(pi/2-L));
            0    1   0;
            -sin(-(pi/2-L))   0   cos(-(pi/2-L))] *...
            [cos(-lamda)  -sin(-lamda)    0;
            sin(-lamda)    cos(-lamda)    0;
            0    0   1]; %dcm_e_g三矩阵相乘的转置
        
dcm_e_n_x = [cos(lamda+pi/2)     -sin(lamda+pi/2)     0;
            sin(lamda+pi/2)      cos(lamda+pi/2)  0;
            0   0   1] *...
            [1  0   0;
            0   cos(pi/2-L)     -sin(pi/2-L);
            0   sin(pi/2-L)     cos(pi/2-L)]; %【xzl20210319】我自己写的,e系到n系的旋转
        
% 只用了dcm_n_e1，其它矩阵只是用于检验，可以直接屏蔽，减少计算
dcm_n_e1 = mnormlz(dcm_n_e1);
q_nav2xyz = qnormlz(m2qua(dcm_n_e1));
