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
LoadConfigAndInit("Grab_prototype_44","Grab_prototype_44");
load exp_conf
%[l0,zeta0] = FindCenter(cdpr_parameters,cdpr_variables,ws_parameters);
l0 = [];
p0 = [];
exp_num = 12;
for i=1:exp_num
  l0 = [l0; l_stat(:,i)];
  p0 = [p0; p_stat(:,i)];
end
ms = MultiStart('FunctionTolerance',1e-6,'XTolerance',1e-6,...
    'UseParallel',true);
%l0 = [0.948874485005648;1.740773458135074;2.102887067361541;1.462086065158142];
cdpr_parameters.underactuated_platform.permutation_matrix =...
    [0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    1 0 0 0 0 0;
    0 1 0 0 0 0];
%param0 = [l0;0.00;0.00];
param0 = [l0;0.01.*ones((2*length(l0))/4,1)];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [zeros(length(l0),1);-0.1.*ones((2*length(l0))/4,1)];
% ub = [10.*ones(cdpr_parameters.n_cables,1);dposef0;ddposef0];
ub = [10.*ones(length(l0),1);0.1.*ones((2*length(l0))/4,1)];
problem = createOptimProblem('fmincon','x0',param0,...
    'Aeq',Aeq,'Aineq',A,'beq',beq,'bineq',b,'lb',lb,'ub',ub,'nonlcon',...
    @(param)OptExtNonLinConstrNovelLinMulti(cdpr_parameters,cdpr_variables,p_stat,param,utilities),...
    'objective',@(param)OptimExcitFunctionNovelLinMulti(cdpr_parameters,cdpr_variables,p_stat,param,utilities),...
    'options',utilities.brutal_fmincon_options_nopar);
%paramet = run(ms,problem,20);
%param0 = [1.557356989971975;1.310240806367134;1.401182007216357;0.325495685309497;0.544810687870450;0.086281458156127];
paramet = fmincon(@(param)OptimExcitFunctionNovelLinMulti(cdpr_parameters,cdpr_variables,p_stat(:,1:exp_num),param,utilities),...
    param0,A,b,Aeq,beq,lb,ub,@(param)OptExtNonLinConstrNovelLinMulti(cdpr_parameters,cdpr_variables,p_stat(:,1:exp_num),param,utilities),utilities.fmincon_options)

% paramet = fmincon(@(param)OptimExcitFunctionNovelLinMulti(cdpr_parameters,cdpr_variables,p_stat(:,1:exp_num),param,utilities),...
%     param0,A,b,Aeq,beq,lb,ub,[],utilities.fmincon_options)


l = paramet(1:cdpr_parameters.n_cables,1);
in_cond = paramet(cdpr_parameters.n_cables+1:end,1);
pose0 = FindClosestPose(cdpr_parameters,cdpr_variables,ws_parameters,l);

pose = fsolve(@(v) FunDkGsL(cdpr_parameters,l,v),pose0,utilities.fsolve_options_grad);
perm = cdpr_parameters.underactuated_platform.permutation_matrix*pose;
free_p = perm(cdpr_parameters.n_cables+1:end)+in_cond;
pose_c = fsolve(@(v) FunDkLFree(cdpr_parameters,l,free_p,v),perm(1:cdpr_parameters.n_cables),utilities.fsolve_options_grad);
delta_a = pose_c-perm(1:cdpr_parameters.n_cables);
opt_delta = cdpr_parameters.underactuated_platform.permutation_matrix'*[delta_a;in_cond];
