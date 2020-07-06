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
  LoadConfigAndInit("Grab_prototype_33","DynamicPlanning"); 

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
    [un_p(:,i),un_dp(:,i),un_ddp(:,i)] = CalcLinearizedConfiguration(linearized_data,t);
    p(:,i) = linearized_data.J_ort*un_p(:,i)+[linearized_data.pos;linearized_data.ang_par];
    dp(:,i) = linearized_data.J_ort*un_dp(:,i);
    ddp(:,i) = linearized_data.J_ort*un_ddp(:,i);
end
 sol = HuenDiscreteSolver(@(time,state) IntegrableDirectDynamicsNoInput(cdpr_parameters,...
        cdpr_variables,utilities,linearized_data.l,time,state),...
        0:utilities.t_interval:t_end,[linearized_data.p0;linearized_data.v0]);
 for k =1:i
    cdpr_variables = UpdateIKZeroOrd(sol.y(1:3,k),sol.y(4:6,k),cdpr_parameters,cdpr_variables);
    cdpr_variables = UpdateIKFirstOrd(sol.y(7:9,k),sol.y(10:12,k),cdpr_parameters,cdpr_variables);
    cdpr_variables = UpdateIKSecondOrd(sol.f(1:3,k),sol.f(4:6,k),cdpr_parameters,cdpr_variables);
    cdpr_variables.platform = cdpr_variables.platform.UpdateMassMatrixStateSpace(cdpr_parameters);
    cdpr_variables = CalcTotalLoadsStateSpace(cdpr_variables,cdpr_parameters);
    cdpr_variables = CalcCablesDynamicTensionStateSpace(cdpr_variables);
    real_tau(:,k) = cdpr_variables.tension_vector;

 end
 for k =1:i
    cdpr_variables = UpdateIKZeroOrd(p(1:3,k),p(4:6,k),cdpr_parameters,cdpr_variables);
    cdpr_variables = UpdateIKFirstOrd(dp(1:3,k),dp(4:6,k),cdpr_parameters,cdpr_variables);
    cdpr_variables = UpdateIKSecondOrd(ddp(1:3,k),ddp(4:6,k),cdpr_parameters,cdpr_variables);
    cdpr_variables.platform = cdpr_variables.platform.UpdateMassMatrixStateSpace(cdpr_parameters);
    cdpr_variables = CalcTotalLoadsStateSpace(cdpr_variables,cdpr_parameters);
    cdpr_variables = CalcCablesDynamicTensionStateSpace(cdpr_variables);
    lin_tau(:,k) = cdpr_variables.tension_vector;

 end
       
 real_sol = sol.y(4:6,:);
 dreal_sol = sol.y(10:12,:);
 ddreal_sol = sol.f(4:6,:);

 figure
 subplot(3,1,1)
 plot(time,p(4,:),time,real_sol(1,:))
 subplot(3,1,2)
plot(time,p(5,:),time,real_sol(2,:))
 subplot(3,1,3)
 plot(time,p(6,:),time,real_sol(3,:))
 
figure 
subplot(3,1,1)
plot(time,lin_tau(1,:),time,real_tau(1,:))
subplot(3,1,2)
plot(time,lin_tau(2,:),time,real_tau(2,:))
subplot(3,1,3)
plot(time,lin_tau(3,:),time,real_tau(3,:))

figure
subplot(6,1,1)
plot(time,p(4,:),time,real_sol(1,:))
subplot(6,1,2)
plot(time,p(5,:),time,real_sol(2,:))
subplot(6,1,3)
plot(time,p(6,:),time,real_sol(3,:))
subplot(6,1,4)
plot(time,lin_tau(1,:),time,real_tau(1,:))
subplot(6,1,5)
plot(time,lin_tau(2,:),time,real_tau(2,:))
subplot(6,1,6)
plot(time,lin_tau(3,:),time,real_tau(3,:))

 diff_p = (real_sol-p);
 diff_dp = (dreal_sol-dp);
 diff_ddp = (ddreal_sol-ddp);