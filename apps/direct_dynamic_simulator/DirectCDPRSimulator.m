clear all
close all
clc
addpath('../../config')
addpath('../../data/workspace_files')
addpath('../../libs/cdpr_model')
addpath('../../libs/export_utilities')
addpath('../../libs/numeric')
addpath('../../libs/orientation_geometry')
addpath('../../libs/under_actuated')
folder = '../../data';

[cdpr_parameters, cdpr_variables,ws_data, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod","DynamicPlanning"); 

traj_name = '..\..\data\planning_results\InverseRTR';
traj(1) = load(strcat(traj_name,'1.mat'));
traj(2) = load(strcat(traj_name,'2.mat'));
traj(3) = load(strcat(traj_name,'3.mat'));

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
 
%  sol_real = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamics(cdpr_parameters,...
%         cdpr_variables,utilities,spline_id,time,state),...
%         0:utilities.t_interval:t(end),p0);  
 sol_real_simp = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamicsSpeedInputUnder(cdpr_parameters,...
        cdpr_variables,spline_id,time,state),...
        0:utilities.t_interval:t(end),p0);     
