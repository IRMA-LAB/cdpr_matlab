% Simple Static Calibration Script UnderActuated CDPR

clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib2.json","DynamicCalibration");

initial_guess = [];
for i=1:cdpr_parameters.n_cables
    initial_guess = [initial_guess;cdpr_parameters.cable(i).motor_cable_tau]; %tau
    initial_guess = [initial_guess;cdpr_parameters.cable(i).swivel_pulley_r]; %r
    initial_guess = [initial_guess;zeros(2,1)]; %beta_x,beta_y
    initial_guess = [initial_guess;cdpr_parameters.cable(i).pos_D_glob]; %point_D
    initial_guess = [initial_guess;cdpr_parameters.cable(i).pos_A_loc]; %point_A
end

simulationData = struct();
simulationData = GetPosesForDynamicCalibration...
  (simulationData,'..\Calibration_data\calibration_data_20190614_results\');

  
%[x_true,~,res_true,exitflag_true,~,~,identif_matrix_true] = lsqnonlin(@(x)DynamicCalibrationFunction(cdpr_parameters,simulationData,x),initial_guess,[],[],utilities.lsqnonlin_options);
%[x_true2,~,res_true2,exitflag_true2,~,~,identif_matrix_true2] = lsqnonlin(@(x)StaticComGravCalibrationFunction(cdpr_parameters,simulationData,x),[cdpr_parameters.platform.pos_G_loc;0;0],[],[],utilities.lsqnonlin_options);
inertia_paramters = ComputeInertia(cdpr_parameters,simulationData);

x_true = x_true3;
for i=1:cdpr_parameters.n_cables
    cdpr_parameters.cable(i).motor_cable_tau = x_true(1+10*(i-1));
    cdpr_parameters.cable(i).swivel_pulley_r = x_true(2+10*(i-1));
    %R = RotX(x_true(3+11*(i-1)))*RotY(x_true(4+11*(i-1)))*RotZ(x_true(5+11*(i-1)));
    if (i==1||i==3)
        R = RotY(x_true(3+10*(i-1)))*RotZ(x_true(4+10*(i-1)));
    else
        R = RotX(x_true(3+10*(i-1)))*RotZ(x_true(4+10*(i-1)));
    end
    cdpr_parameters.cable(i).vers_i = R*cdpr_parameters.cable(i).vers_i;
    cdpr_parameters.cable(i).vers_j = R*cdpr_parameters.cable(i).vers_j;
    cdpr_parameters.cable(i).vers_k = R*cdpr_parameters.cable(i).vers_k;
    cdpr_parameters.cable(i).pos_D_glob = x_true(5+10*(i-1):7+10*(i-1));
    cdpr_parameters.cable(i).pos_A_loc = x_true(8+10*(i-1):10+10*(i-1));
end
cdpr_parameters.platform.pos_G_loc = x_true(end-4:end-2);
cdpr_parameters.platform.gravity_acceleration = RotX(x_true(end-1))*RotY(x_true(end))*cdpr_parameters.platform.gravity_acceleration;

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