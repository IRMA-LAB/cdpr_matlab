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
data_marker_file_name = 'v_nat_f_26052020/ParsedDataPose/marker_pos_rel_to_ref.mat';
data_real_f_name = 'v_nat_f_26052020/ParsedData/data_real_f.mat';
data_real_z_name = 'v_nat_f_26052020/ParsedData/data_real_z.mat';

load (data_f_file_name);
load(data_marker_file_name);
f_exp = zeros(60,4);
f_max = zeros(60,4);
f_min = zeros(60,4);

for i=1:60
    load([data_pose_file_name num2str(i) '.mat']);
    p = pose_print(1:3,101:1100);
    ang = pose_print(4:6,101:1100);
    for j=1:1000
        R = RotXYZ(ang(:,j));
        trial(i).marker(1).position(:,j) = p(:,j);
        for k=1:4
            trial(i).marker(k+1).position(:,j) = p(:,j)+R*marker_rec(k).p;
        end
    end
    if (i<37)
        n_f = 2;
   elseif(i<49)
        n_f = 3;
   else
       n_f = 4;
   end
   v0 = [];
   lb = [];
   ub = [];
   for j=1:n_f
       v0 = [v0;20;0.02;data_f(i).val(j);0];
       lb = [lb;-Inf;0;data_f(i).val(j)-data_f(i).val(j)/10;-2*pi];
       ub = [ub;Inf;0.04;data_f(i).val(j)+data_f(i).val(j)/10;2*pi];
   end
   

   idx = 0;
for j=1:5
    for k=1:3
        idx = idx+1;
        mean_p0 = mean(trial(i).marker(j).position(k,:));
        p0_opt  = trial(i).marker(j).position(k,:)-mean_p0;
        [sol(:,idx),fval(idx)] = lsqnonlin(@(x)IdentifyF_Vlast(x,p0_opt,n_f),v0,lb,ub,options);
    end
end

for j=1:n_f
    f_exp(i,j) = abs(sol((j-1)*4+1,:))*sol((j-1)*4+3,:)'/sum(abs(sol((j-1)*4+1,:)));
    f_max(i,j) = max(sol((j-1)*4+3,:));
    f_min(i,j) = min(sol((j-1)*4+3,:));
end
clear sol
clear fval
   
end

xlswrite('f_exp.xlsx',f_exp);
xlswrite('f_max.xlsx',f_max);
xlswrite('f_min.xlsx',f_min);

