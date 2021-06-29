clear all
clc
close all

line1 = [1 0 1 2 3];
t = 0:0.001:20-0.001;
l = length(t);
line = [0 4 0 0];
line2 = [0 0 4 0];
m = [];
z = zeros(l/2,1);
torque = 300;
cmd_torque = linspace(0,torque,l/4)';
cmd_torque = [cmd_torque;linspace(torque,0,l/4)'];
cmd_tot = [z cmd_torque z z];
cmd_tot = [cmd_tot;z z cmd_torque  z];
for i=1:l/2
    m = [m;line];
end
for i=1:l/2
    m = [m;line2];
end
m = [t' m cmd_tot];


fileID = fopen('/home/grablab/Desktop/prova_torque_mix.txt','w');

fprintf(fileID,'%d %d %d %d %d\n', line1);
for i=1:l
fprintf(fileID,'%4.4f %d %d %d %d %4.4f %4.4f %4.4f %4.4f\n', m(i,:));
end


fileID = fclose(fileID);