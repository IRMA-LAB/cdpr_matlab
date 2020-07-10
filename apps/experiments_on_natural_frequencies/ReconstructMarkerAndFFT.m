clear all
close all
clc

options = optimoptions('lsqnonlin','MaxIterations',1000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'StepTolerance',1e-7,'UseParallel',true);

addpath('../../libs/orientation_geometry')

dest_dir = 'ParsedDataPose';
org_file_name = 'v_nat_f_';
dest_file_name = 'v_pose_nat_f_';
dest_file_name_pose = 'v_pose_nat_f_';

data_pose_file_name = 'v_nat_f_26052020/ParsedDataPose/v_pose_nat_f_';
data_f_file_name = 'v_nat_f_26052020/ParsedData/data_f.mat';
data_p_file_name = 'v_nat_f_26052020/ParsedData/data_p.mat';
data_l_file_name = 'v_nat_f_26052020/ParsedData/data_l.mat';
data_marker_file_name = 'v_nat_f_26052020/ParsedDataPose/marker_pos_rel_to_ref.mat';
data_real_f_name = 'v_nat_f_26052020/ParsedData/data_real_f.mat';
data_real_z_name = 'v_nat_f_26052020/ParsedData/data_real_z.mat';

load (data_f_file_name);
load (data_p_file_name);
load (data_l_file_name);
load(data_marker_file_name);
f_exp = zeros(60,4);
f_max = zeros(60,4);
f_min = zeros(60,4);
f_std = zeros(60,4);
% fileID = fopen('exptable_new.txt', 'w');
for i=48:60
    load([data_pose_file_name num2str(i) '.mat']);
    p = pose_print(1:3,101:1100);
    ang = pose_print(4:6,101:1100);
    pks = [];
    locs = [];
    f = figure();
    a = gca;
    hold on
    grid on
    lgd = legend();
    lgd.Interpreter = 'latex';
    for j=1:1000
        R = RotXYZ(ang(:,j));
        trial(i).marker(1).position(:,j) = p(:,j);
        for k=1:4
            trial(i).marker(k+1).position(:,j) = p(:,j)+R*marker_rec(k).p;
        end
    end
    for j=1:5
        switch j
            case 1
                col_line = 'b';
                name = 'marker n. 1';
            case 2
                col_line = 'r';
                name = 'marker n. 2';
            case 3
                col_line = 'm';
                name = 'marker n. 3';
            case 4
                col_line = 'k';
                name = 'marker n. 4';
            case 5
                col_line = 'g';
                name = 'marker n. 5';
        end          
       for k=1:3
           switch k
            case 1
                col_line_d = [col_line '-'];
                name_d = [name ' coord. x'];
            case 2
                col_line_d = [col_line '--'];
                name_d = [name ' coord. y'];
            case 3
                col_line_d = [col_line ':'];
                name_d = [name ' coord. z'];
           end 
            [pp,ll] = PlotFreq(trial(i).marker(j).position(k,:)',col_line_d,name_d);
            pks = [pks pp];
            locs = [locs ll'];
       end
    end
    for k=1:length(data_f(i).val)
        lims = [data_f(i).val(k)-7*data_f(i).val(k)/100 data_f(i).val(k)+7*data_f(i).val(k)/100];
        f_locs = find(locs < lims(2) & locs > lims(1));
        [A,idx] = sort(pks(f_locs),'descend');
        f_star = locs(f_locs);
        f_star = f_star(idx);
        if length(A)>15
            A(16:end) = [];
            f_star(16:end) = [];
        end
        f_mean = 0;
        std = 0;
        tot = sum(A);
        for idx=1:length(A)
           f_mean = f_mean+A(idx)*f_star(idx)/tot; 
        end
        for idx=1:length(A)
           std = std+A(idx)*(f_star(idx)-f_mean)^2/tot; 
        end
        std = sqrt(std);
        f_exp(i,k) = f_mean;
        f_std(i,k) = std;
    end
    
    limit = a.YTick(end);
    for k=1:length(data_f(i).val)
      %  plot([data_f(i).val(k);data_f(i).val(k)],[0;limit],'k-o','DisplayName',['$f_' num2str(k) '= ' num2str(data_f(i).val(k),'%3.2f') '~\mathrm{Hz}$'],'LineWidth',1);
    end
    hold off
    i+2
   % savefig(f,['Experiment ' num2str(i)]);
    close(f)
    
% fprintf(fileID,'$%i$ & ',i);
% fprintf(fileID,'$%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f$ & ',data_p(i).val);
% fprintf(fileID,'$%3.2f,%3.2f$ & ',data_l(i).val);
% fprintf(fileID,'\\makecell{$%3.2f,%3.2f,$ \\\\ $%3.2f,%3.2f$} & ',data_f(i).val);
% fprintf(fileID,'\\makecell{$%3.2f,%3.2f,$ \\\\ $%3.2f,%3.2f$} & ',f_exp(i,1:4));
% fprintf(fileID,'\\makecell{$%3.2f,%3.2f,$ \\\\ $%3.2f,%3.2f$} & ',f_std(i,1:4));
% fprintf(fileID,'\\makecell{$%3.2f,%3.2f,$ \\\\ $%3.2f,%3.2f$} \\\\\n',(f_exp(i,1:4)'-data_f(i).val)./data_f(i).val.*100);
% fprintf(fileID,'\\hline \n');

    
end
% fclose(fileID);
% xlswrite('f_exp_mean.xlsx',f_exp);
% xlswrite('f_exp_std.xlsx',f_std);
clear x
clear f_mod
clear f_exp_s
clear f_sorted
clear f_sorted_exp
hold on
grid on
ax = gca;
ax.XTick = 1:60;
ax.YTick = 0:0.25:4;
ax.XLim = [min(ax.XTick) max(ax.XTick)];
ax.YLim = [min(ax.YTick) max(ax.YTick)];
set(gcf, 'Units', 'centimeters', 'InnerPosition', [0, 0, 18, 6]);

for i=1:36
   x(i-0) = i;
   f_mod(:,i-0) = data_f(i).val';
   f_exp_s(:,i-0) = f_exp(i,1:2)';
end
[f_sorted,indexes] = sort(f_mod(1,:));
f_sorted = [f_sorted;f_mod(2,indexes)];
f_sorted_exp = f_exp_s(:,indexes);

plot(x,f_sorted(1,:),'k-x',x,f_sorted_exp(1,:),'r--o','MarkerSize',5)
plot(x,f_sorted(2,:),'k-x',x,f_sorted_exp(2,:),'r--o','MarkerSize',5)

clear x
clear f_mod
clear f_exp_s
clear f_sorted
clear f_sorted_exp
for i=37:48
   x(i-36) = i;
   f_mod(:,i-36) = data_f(i).val';
   f_exp_s(:,i-36) = f_exp(i,1:3)';
end
[f_sorted,indexes] = sort(f_mod(1,:));
f_sorted = [f_sorted;f_mod(2:3,indexes)];
f_sorted_exp = f_exp_s(:,indexes);

plot(x,f_sorted(1,:),'k-x',x,f_sorted_exp(1,:),'r--o','MarkerSize',5)
plot(x,f_sorted(2,:),'k-x',x,f_sorted_exp(2,:),'r--o','MarkerSize',5)
plot(x,f_sorted(3,:),'k-x',x,f_sorted_exp(3,:),'r--o','MarkerSize',5)
%plot(x,f_sorted(4,:),'k-x',x,f_sorted_exp(4,:),'r--o')

clear x
clear f_mod
clear f_exp_s
clear f_sorted
clear f_sorted_exp
for i=49:60
   x(i-48) = i;
   f_mod(:,i-48) = data_f(i).val';
   f_exp_s(:,i-48) = f_exp(i,1:4)';
end
[f_sorted,indexes] = sort(f_mod(1,:));
f_sorted = [f_sorted;f_mod(2:4,indexes)];
f_sorted_exp = f_exp_s(:,indexes);

plot(x,f_sorted(1,:),'k-x',x,f_sorted_exp(1,:),'r--o','MarkerSize',5)
plot(x,f_sorted(2,:),'k-x',x,f_sorted_exp(2,:),'r--o','MarkerSize',5)
plot(x,f_sorted(3,:),'k-x',x,f_sorted_exp(3,:),'r--o','MarkerSize',5)
plot(x,f_sorted(4,:),'k-x',x,f_sorted_exp(4,:),'r--o','MarkerSize',5)

hold off
        
figure;
hold on
for i=1:36
   x = i;
   f_mod = data_f(i).val';
   f_exp_s = f_exp(i,1:2);
   w1 = 1; 
   bar(x,f_mod,w1,'FaceColor',[100/255 100/255 100/255],'EdgeColor','k')
    w1 = 0.5; 
   bar(x,f_exp_s,w1,'FaceColor',[166/255 210/255 70/255],'EdgeColor','k')
end
hold off
