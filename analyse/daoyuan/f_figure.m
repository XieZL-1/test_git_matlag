function [] = f_figure(fig_id,name,show_en)

if nargin==1
    name = '';
    show_en = 'on';
elseif nargin==2
    show_en = 'on';
end

if show_en==0
    show_en='off';
elseif show_en==1
    show_en = 'on';
end

figure(fig_id);
% clf;
set(gcf,'visible',show_en);
set(gcf,'name',name);
assignin('base',strcat('fig_num_',num2str(fig_id)),fig_id);
