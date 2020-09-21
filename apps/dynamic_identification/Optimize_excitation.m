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
addpath('../../libs/over_actuated')
folder = '../../data';

%workspace of grab underactuated prototype
[cdpr_parameters, cdpr_variables, ws_parameters ,cdpr_outputs,record,utilities] = ...
LoadConfigAndInit("Grab_prototype_33","Grab_prototype_33");

[l0,zeta0] = FindCenter(cdpr_parameters,cdpr_variables,ws_parameters);

ms = MultiStart('FunctionTolerance',1e-6,'XTolerance',1e-6,...
    'UseParallel',true);
param0 = [l0;0;0;0];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [zeros(cdpr_parameters.n_cables,1);-pi/3;-pi/3;-pi/3];
% ub = [10.*ones(cdpr_parameters.n_cables,1);dposef0;ddposef0];
ub = [10.*ones(cdpr_parameters.n_cables,1);pi/3;pi/3;pi/3];
problem = createOptimProblem('fmincon','x0',param0,...
    'Aeq',Aeq,'Aineq',A,'beq',beq,'bineq',b,'lb',lb,'ub',ub,'nonlcon',...
    @(param)OptExtNonLinConstrNovel(cdpr_parameters,cdpr_variables,ws_parameters,param,utilities),...
    'objective',@(param)OptimExcitFunctionNovel(cdpr_parameters,cdpr_variables,ws_parameters,param,utilities),...
    'options',utilities.brutal_fmincon_options_nopar);
paramet = run(ms,problem,10);
% paramet = fmincon(@(param)OptimExcitFunctionNovel(cdpr_parameters,cdpr_variables,ws_parameters,param,utilities),...
%     param_0,A,b,Aeq,beq,lb,ub,@(param)OptExtNonLinConstrNovel(cdpr_parameters,cdpr_variables,ws_parameters,param,utilities),utilities.fmincon_options)

l = paramet(1:cdpr_parameters.n_cables,1);
in_cond = paramet(cdpr_parameters.n_cables+1:end,1);
pose0 = FindClosestPose(cdpr_parameters,cdpr_variables,ws_parameters,l);

pose = fsolve(@(v) FunDkGsL(cdpr_parameters,l,v),pose0,utilities.fsolve_options_grad);
perm = cdpr_parameters.underactuated_platform.permutation_matrix*pose;
free_p = perm(cdpr_parameters.n_cables+1:end)+in_cond;
pose_c = fsolve(@(v) FunDkLFree(cdpr_parameters,l,free_p,v),perm(1:cdpr_parameters.n_cables),utilities.fsolve_options_grad);
delta_a = pose_c-perm(1:cdpr_parameters.n_cables);
opt_delta = cdpr_parameters.underactuated_platform.permutation_matrix'*[delta_a;in_cond];
