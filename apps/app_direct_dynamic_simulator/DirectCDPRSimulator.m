clear all
close all
clc
addpath('../files_configuration')
addpath('../files_workspace')
addpath('../fun_cdpr_model')
addpath('../fun_export_utilities')
addpath('../fun_numeric')
addpath('../fun_orientation_geometry')
addpath('../fun_under_actuated')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod.json","DynamicPlanning"); 

sym_result_name = 'C:\Users\EdoPortable\OneDrive - Alma Mater Studiorum Università di Bologna\Work\CableRobotCodes\Results\InverseRTR';
traj(1) = load(strcat(sym_result_name,'1.mat'));
traj(2) = load(strcat(sym_result_name,'2.mat'));
traj(3) = load(strcat(sym_result_name,'3.mat'));

[t,cables,p0] = JoinTrajectories(traj,cdpr_parameters.n_cables,utilities.t_interval);

% i=0;
% for tt = t
%     i=i+1;
%     for j=1:cdpr_parameters.n_cables
%         l(j,i) = cables(j).length(i)+(2.*rand(1,1)-1).*0.0002;
%         if i==1
%             dl(j,i) = FiniteDifferentiation(l(j,i),l(j,i),0,t(i),utilities);
%             ddl(j,i) = FiniteDifferentiation(dl(j,i),dl(j,i),0,t(i),utilities);
%         else
%             dl(j,i) = FiniteDifferentiation(l(j,i),l(j,i-1),dl(j,i-1),t(i),utilities);
%             ddl(j,i) = FiniteDifferentiation(dl(j,i),dl(j,i-1),ddl(j,i-1),t(i),utilities);
%         end
%     end
% end

for j=1:cdpr_parameters.n_cables
   spline_id.l(j) = spline(t,cables(j).complete_length);
   spline_id.l_d(j) = spline(t,cables(j).complete_speed);
   spline_id.l_d2(j) = spline(t,cables(j).complete_acceleration);
   
end

% for j=1:cdpr_parameters.n_cables
%    spline_real.l(j) = spline(t,l(j,:));
%    spline_real.l_d(j) = spline(t,dl(j,:));
%    spline_real.l_d2(j) = spline(t,ddl(j,:));
%    
% end
 
%  tic
%  sol_id = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamics33(cdpr_parameters,...
%         cdpr_variables,utilities,spline_id,time,state),...
%         0:utilities.t_interval:t(end),p0);  
%  t1 = toc
 
 tic
 sol_real = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamics33(cdpr_parameters,...
        cdpr_variables,utilities,spline_id,time,state),...
        0:utilities.t_interval:t(end),p0);  
 t2 = toc
