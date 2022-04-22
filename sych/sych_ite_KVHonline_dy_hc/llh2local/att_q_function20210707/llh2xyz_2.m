function [xyz_sphere, q_nav2xyz, dcm_n_e1] = llh2xyz_2(llh)
%# ��xzl20210319����γ����ת��Ϊ����ֱ������ϵ,
%����ý��õ�xyz���굽�õ㵼������ϵ���������Ԫ�����������Ҿ��󣬼�ֱ�����굽�������꣬Ҳ����������ϵ��ֱ������ϵ����ת
%# ��xzl20210319������xzl20201206�������������ת��ϵ���淶�˷���

%   llh = [lati; longi; height];  %��λ ��

%%
Re = 6378137;   %������뾶
f = 1/298.257;  
e = sqrt(2*f-f^2); %ƫ����

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
            0   0   1];     %�Ϲ�����p44
        
%������dcm_n_e��dcm_n_e1һ��
dcm_n_e = dcm_e_g^(-1);
dcm_n_e1 = [cos(-pi/2)     -sin(-pi/2)     0;
            sin(-pi/2)      cos(-pi/2)  0;
            0   0   1] *...
            [cos(-(pi/2-L))  0   sin(-(pi/2-L));
            0    1   0;
            -sin(-(pi/2-L))   0   cos(-(pi/2-L))] *...
            [cos(-lamda)  -sin(-lamda)    0;
            sin(-lamda)    cos(-lamda)    0;
            0    0   1]; %dcm_e_g��������˵�ת��
        
dcm_e_n_x = [cos(lamda+pi/2)     -sin(lamda+pi/2)     0;
            sin(lamda+pi/2)      cos(lamda+pi/2)  0;
            0   0   1] *...
            [1  0   0;
            0   cos(pi/2-L)     -sin(pi/2-L);
            0   sin(pi/2-L)     cos(pi/2-L)]; %��xzl20210319�����Լ�д��,eϵ��nϵ����ת
        
% ֻ����dcm_n_e1����������ֻ�����ڼ��飬����ֱ�����Σ����ټ���
dcm_n_e1 = mnormlz(dcm_n_e1);
q_nav2xyz = qnormlz(m2qua(dcm_n_e1));
