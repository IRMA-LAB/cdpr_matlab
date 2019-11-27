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

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_33","HomingTest33");

% % Generation of the "roughly" estimated home pose
tension_limits = [50 100];
steps4cable = 5;
start = SimulateGoToStartProcedureErroneous...
    (cdpr_parameters, cdpr_variables,ws_parameters,record,utilities,tension_limits);
home = SimulateGoToStartProcedureIdeal...
    (cdpr_parameters, cdpr_variables,start.pose,record,utilities,tension_limits);
[delta_l,delta_sw] = SimulateDataAcquisition(cdpr_parameters, cdpr_variables, steps4cable,home,record,utilities,tension_limits);
init_guess = GenerateInitialGuess(cdpr_parameters,cdpr_variables,delta_l,start,record,utilities);

% Contruct external patameter and initial guess
init_guess = [start.l;start.sw;init_guess];
[sol,resnorm,residual,exitflag,output] = lsqnonlin(@(v)HomingOptimizationFunction...
  (cdpr_parameters,record,delta_l,delta_sw,v),init_guess ,[],[],utilities.lsqnonlin_options_grad);
l0 = sol(1:cdpr_parameters.n_cables,1);
s0 = sol(cdpr_parameters.n_cables+1:2*cdpr_parameters.n_cables,1);
[pose0,resnormp,residualp,exitflagp,outputp] = lsqnonlin(@(v)FunDkGsSwL...
  (cdpr_parameters,l0,s0,v,record),start.pose ,[],[],utilities.lsqnonlin_options_grad);




