clear all
clc
close all

filename = 'v_pose_nat_f_'
for i=1:36
vars = load(strcat(filename,num2str(i),'.mat'));
v = vars.pose_print;
exp = v(:,1:1001);
p = exp(1:6,:); p(1:3,:) = p(1:3,:)/1000;
v = exp(7:12,:); v(1:3,:) = v(1:3,:)/1000;
a = exp(13:18,:); a(1:3,:) = a(1:3,:)/1000;
save(strcat('44_exp_',num2str(i)),'p','v','a');
end