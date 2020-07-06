clear all
clc
close all

load('data_f.mat');
load('data_p.mat');
load('data_l.mat');

n = length(data_f);
f = zeros(n,4);
p = zeros(n,6);
l = zeros(n,4);

for i=1:n
    f(i,1:length(data_f(i).val)) = data_f(i).val';
    l(i,1:length(data_l(i).val)) = data_l(i).val';
    p(i,:) = data_p(i).val';
end

xlswrite('f.xlsx',f);
xlswrite('p.xlsx',p);
xlswrite('l.xlsx',l);