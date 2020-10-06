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

load 4_cab_exp_1
exp(1:3,:) = exp(1:3,:)/1000;
exp(7:9,:) = exp(7:9,:)/1000;
exp(13:15,:) = exp(13:15,:)/1000;
exp2 = exp;
load 4_cab_exp
exp(1:3,:) = exp(1:3,:)/1000;
exp(7:9,:) = exp(7:9,:)/1000;
exp(13:15,:) = exp(13:15,:)/1000;
exp = [exp exp2];
pose_stat = mean(exp(1:6,:),2);
W = [];
l = W;
for i=1:length(exp)
    cdpr_variables = UpdateIKZeroOrd(exp(1:3,i),exp(4:6,i),cdpr_parameters,cdpr_variables);
    cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobians...
        (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian,cdpr_variables.D_mat);
    cdpr_variables = UpdateIKFirstOrd(exp(7:9,i),exp(10:12,i),cdpr_parameters,cdpr_variables);
    cdpr_variables = UpdateIKSecondOrd(exp(13:15,i),exp(16:18,i),cdpr_parameters,cdpr_variables);
    l = [l cdpr_variables.cable_vector];
    W_EE = ComputeIdentificationMatrix(cdpr_parameters,cdpr_variables);
    W = [W;cdpr_variables.underactuated_platform.geometric_orthogonal'*W_EE];
end

[U,S,V] = svd(W,'econ');
X = V(:,10)./V(1,10);
W_hat = W-S(10,10).*U(:,10)*V(:,10)';
sigma = S(10,10)/sqrt(length(W)-10);
C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
sigma_perc = 100*sqrt(diag(C))./X(2:end);

val = S(1,1)/S(9,9)+1/S(9,9);

cdpr_variables = UpdateIKZeroOrd(pose_stat(1:3),pose_stat(4:end),cdpr_parameters,cdpr_variables);
cdpr_variables = CalcExternalLoads(cdpr_variables,cdpr_parameters);
cdpr_variables = CalcCablesStaticTensionNoCheck(cdpr_variables);
% cdpr_parameters.underactuated_platform.permutation_matrix = [...
%    cdpr_parameters.underactuated_platform.permutation_matrix(4:6,:);...
%    cdpr_parameters.underactuated_platform.permutation_matrix(1:3,:)];
cdpr_parameters.underactuated_platform.permutation_matrix =...
    [0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    1 0 0 0 0 0;
    0 1 0 0 0 0];

W = [];
for i=1:10:length(exp)
     [act(:,i),val] = fsolve(@(v) FunDkLFree(cdpr_parameters,...
        cdpr_variables.cable_vector,exp(1:2,i),v),pose_stat(3:6),...
        utilities.fsolve_options_grad);
%      [pos(:,i),val] = fsolve(@(v) FunDkLFree(cdpr_parameters,...
%         cdpr_variables.cable_vector,exp(4:6,i),v),pose_stat(1:3),...
%         utilities.fsolve_options_grad);
    new_pose(:,i) = cdpr_parameters.underactuated_platform.permutation_matrix'*[act(:,i);exp(1:2,i)];
    cdpr_variables = UpdateIKZeroOrd(new_pose(1:3,i),new_pose(4:end,i),cdpr_parameters,cdpr_variables);
    cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobians...
        (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian,cdpr_variables.D_mat);
    
    new_pose_d(:,i) = cdpr_variables.underactuated_platform.analitic_orthogonal*exp(7:8,i);
    cdpr_variables = UpdateIKFirstOrd(new_pose_d(1:3,i),new_pose_d(4:end,i),cdpr_parameters,cdpr_variables);
    cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobiansD...
        (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian_d,cdpr_variables.D_mat,cdpr_variables.D_mat_d);
    new_pose_dd(:,i) = cdpr_variables.underactuated_platform.analitic_orthogonal*exp(13:14,i)+...
        cdpr_variables.underactuated_platform.analitic_orthogonal_d*exp(7:8,i);
    cdpr_variables = UpdateIKSecondOrd(new_pose_dd(1:3,i),new_pose_dd(4:end,i),cdpr_parameters,cdpr_variables);
    W_EE = ComputeIdentificationMatrix(cdpr_parameters,cdpr_variables);
    W = [W;cdpr_variables.underactuated_platform.geometric_orthogonal'*W_EE];
end

[U,S,V] = svd(W,'econ');
X = V(:,10)./V(1,10);
W_hat = W-S(10,10).*U(:,10)*V(:,10)';
sigma = S(10,10)/sqrt(length(W)-10);
C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
sigma_perc = 100*sqrt(diag(C))./X(2:end);

val = S(1,1)/S(9,9)+1/S(9,9);