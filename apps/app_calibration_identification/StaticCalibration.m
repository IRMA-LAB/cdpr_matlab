% Simple Static Calibration Script UnderActuated CDPR

clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_pre_calib.json","StaticCalibration");

initial_guess = [];
for i=1:cdpr_parameters.n_cables
    initial_guess = [initial_guess;1]; %l_0
    initial_guess = [initial_guess;cdpr_parameters.cable(i).motor_cable_tau]; %tau
    initial_guess = [initial_guess;cdpr_parameters.cable(i).swivel_pulley_r]; %r
    initial_guess = [initial_guess;zeros(2,1)]; %beta_x,beta_y
    initial_guess = [initial_guess;cdpr_parameters.cable(i).pos_D_glob]; %point_D
    initial_guess = [initial_guess;cdpr_parameters.cable(i).pos_A_loc]; %point_A
end
cdpr_p = cdpr_parameters;
for i=1:cdpr_parameters.n_cables
    cdpr_p.cable(i).motor_cable_tau = initial_guess(2+11*(i-1)).*1.15;
    cdpr_p.cable(i).swivel_pulley_r = initial_guess(3+11*(i-1)).*1.10;
    R = RotX(pi/30)*RotZ(pi/50);
    cdpr_p.cable(i).vers_i = R*cdpr_p.cable(i).vers_i;
    cdpr_p.cable(i).vers_j = R*cdpr_p.cable(i).vers_j;
    cdpr_p.cable(i).vers_k = R*cdpr_p.cable(i).vers_k;
    cdpr_p.cable(i).pos_D_glob = initial_guess(6+11*(i-1):8+11*(i-1)).*1.15;
    cdpr_p.cable(i).pos_A_loc = initial_guess(9+11*(i-1):11+11*(i-1)).*1.15;
end


simulationData = struct();
simulationData = GetPosesForStaticCalibration...
  (simulationData,cdpr_p,record,utilities);

initial_guess(1) = simulationData.l(1,1).*1.15;
initial_guess(12) = simulationData.l(2,1).*1.15;
initial_guess(23) = simulationData.l(3,1).*1.15;

x = [];
for i=1:cdpr_parameters.n_cables
    x = [x;simulationData.l(i,1)]; %l_0
    x = [x;cdpr_p.cable(i).motor_cable_tau]; %tau
    x = [x;cdpr_p.cable(i).swivel_pulley_r]; %r
    x = [x;pi/30;pi/50]; %beta_x,beta_y
    x = [x;cdpr_p.cable(i).pos_D_glob]; %point_D
    x = [x;cdpr_p.cable(i).pos_A_loc]; %point_A
end
  
[x_true,~,res_true,exitflag_true,~,~,identif_matrix_true] = lsqnonlin(@(x)StaticCalibrationFunction(cdpr_parameters,simulationData.p,simulationData.dcount,x),initial_guess,[],[],utilities.lsqnonlin_options);
%devs = StdDeviationAbs(identif_matrix_true,x_true);

simulationData2 = simulationData;
simulationData2.p(1:3,:) = simulationData.p(1:3,:)+ 0.01.*(2.*rand(3,length(simulationData.p(1:3,:)))-1);
simulationData2.p(4:6,:) = simulationData.p(4:6,:)+ 0.01.*(2.*rand(3,length(simulationData.p(4:6,:)))-1);
lb = x-abs(x)./10;
ub = x+abs(x)./10;
ub(7) = 0.01;
ub(21) = 0.01;
lb(7) = -0.01;
lb(21) = -0.01;
[x_2,~,res_2,exitflag_2,~,~,identif_matrix_2] = lsqnonlin(@(x)StaticCalibrationFunction(cdpr_parameters,simulationData2.p,simulationData2.dcount,x),initial_guess,lb,ub,utilities.lsqnonlin_options);
%devs2 = StdDeviationAbs(identif_matrix_2,x_2);
diff = (x-x_2)./x.*100;
% [outputDataRTR] = RestToRestCoefficients33(cdpr_parameters,cdpr_variables,...
%      simulationData,geometricFunction,utilities);
% [outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
%      simulationData,geometricFunction,utilities);
% DataLoggerStruct(outputDataRTR,'InverseRTR',true,cdpr_parameters,cdpr_variables,record,utilities);
% DataLoggerStruct(outputDataSTD,'InverseSTD',true,cdpr_parameters,cdpr_variables,record,utilities);
%  