function [xyz_sphere,q_xyz2nav,dcm0] = llh2m(llh)
%   llh = [lati; longi; height];  %��λ �� ��
dcm0 = eye(3);
xyz_sphere = zeros(size(llh,1),3);
q_xyz2nav = zeros(size(llh,1),4);
[m,~] = size(llh);
for i = 1:m
    [xyz_sphere(i,:), q_xyz2nav(i,:), dcm_e_n1] = llh2xyz_2(llh(i,:)); %��xzl20210319������ķ��ű�ʾ�����⣬����������û���⣬��llh2xyz_2
    if i==1
        dcm0 =  dcm_e_n1;
    end
end

end

