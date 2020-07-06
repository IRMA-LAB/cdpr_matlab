clear all
close all
clc

options = optimoptions('lsqnonlin','MaxIterations',1000,'MaxFunctionEvaluations',10000,'OptimalityTolerance',1e-8,'FunctionTolerance',1e-8,'StepTolerance',1e-7,'UseParallel',true);
%ms = MultiStart('UseParallel',true,'Display','iter');

data_pose_file_name = 'v_nat_f_26052020/ParsedDataPose/v_pose_nat_f_';
data_f_file_name = 'v_nat_f_26052020/ParsedData/data_f.mat';
data_p0_file_name = 'v_nat_f_26052020/ParsedData/data_p.mat';
data_real_f_name = 'v_nat_f_26052020/ParsedData/data_real_f.mat';
data_real_z_name = 'v_nat_f_26052020/ParsedData/data_real_z.mat';
load (data_f_file_name);
load (data_p0_file_name);

l = 60;
for i=1:l
   load ([data_pose_file_name num2str(i) '.mat'])
   data_po(i).val = pose_print(1:6,200:800);
   clear pose_print
   
end

for i=1:l
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
       v0 = [v0;1;0.02;data_f(i).val(j);0];
       lb = [lb;-Inf;0;data_f(i).val(j)-data_f(i).val(j)/10;-2*pi];
       ub = [ub;Inf;0.04;data_f(i).val(j)+data_f(i).val(j)/10;2*pi];
   end
%     problem = createOptimProblem('lsqnonlin',...
%     'objective',@(x)IndetifyF_V3(x,data_analysis,n_f,n_points,n_coordinates),'lb',lb,'ub',ub,...
%     'x0',v0,'options',options);   
%     [x,ff] = run(ms,problem,15);
mean_p0 = mean(data_po(i).val,2);
p0_opt  = data_po(i).val-mean_p0;

for j=1:6
    [sol(:,j),fval(j)] = lsqnonlin(@(x)IdentifyF_Vlast(x,p0_opt(j,:),n_f),v0,lb,ub,options);
end
for j=1:n_f
   data_f_real(i).val(j,:) = sol(j*4-1,:); 
   data_z_real(i).val(j,:) = sol(j*4-2,:);
end
   clear sol
   clear fval
    
   % ComputePlot_V3(x,data_analysis,n_f,n_points,n_coordinates);
    clear points
    clear data_analysis
   
end

save(data_real_f_name, 'data_f_real');    
save(data_real_z_name, 'data_z_real');  