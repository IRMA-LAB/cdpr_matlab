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

index = randi(ws_data.counter);
scale_fact_pos = 2; %max amplitude degree
scale_fact_vel = 0; %max amplitude degree/s
t_end = 10; %sim time
linearized_data = ExtractInfoFromWS(cdpr_variables,cdpr_parameters,ws_data,...
    utilities,record,index,scale_fact_pos,scale_fact_vel);
i = 0;

for t = 0:utilities.t_interval:t_end
    i=i+1;
    time(i,1) = t;
    [p(:,i),dp(:,i),ddp(:,i)] = CalcLinearizedConfiguration(linearized_data,t);
end

 sol_real_simp = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamicsNoInput(cdpr_parameters,...
        cdpr_variables,utilities,linearized_data.l,time,state),...
        0:utilities.t_interval:t_end,[linearized_data.p0;linearized_data.v0]);    
 real_sol = 180/pi.*sol_real_simp.y(4:6,:);
 dreal_sol = 180/pi.*sol_real_simp.y(10:12,:);
 ddreal_sol = 180/pi.*sol_real_simp.f(4:6,:);

figure
subplot(3,1,1)
plot(time,p(1,:),time,real_sol(1,:))
subplot(3,1,2)
plot(time,p(2,:),time,real_sol(2,:))
subplot(3,1,3)
plot(time,p(3,:),time,real_sol(3,:))

 diff_p = (real_sol-p);
 diff_dp = (dreal_sol-dp);
 diff_ddp = (ddreal_sol-ddp);