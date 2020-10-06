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
% W = [];
% for i=1:15
%    filename = strcat('data_opt\44_exp_',num2str(i));
%    load(filename);
%    pose_stat = mean(p,2);
%    %W = [];
%    for j=1:1:length(p)
%    cdpr_variables = UpdateIKZeroOrd(p(1:3,j),p(4:end,j),cdpr_parameters,cdpr_variables);
%      cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobians...
%         (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian,cdpr_variables.D_mat);
%      cdpr_variables = UpdateIKFirstOrd(v(1:3,j),v(4:end,j),cdpr_parameters,cdpr_variables);
%      cdpr_variables = UpdateIKSecondOrd(a(1:3,j),a(4:end,j),cdpr_parameters,cdpr_variables);
%      W_EE = ComputeIdentificationMatrix(cdpr_parameters,cdpr_variables);
%     W = [W;cdpr_variables.underactuated_platform.geometric_orthogonal'*W_EE];
%    end
% %    [U,S,V] = svd(W,'econ');
% % X = V(:,10)./V(1,10);
% % W_hat = W-S(10,10).*U(:,10)*V(:,10)';
% % sigma = S(10,10)/sqrt(length(W)-10);
% % C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
% % sigma_perc = 100*sqrt(diag(C))./X(2:end);
% % valv =  S(1,1)/S(9,9)+1/S(9,9)
% % val(i,1) = valv;
% % X_val(:,i) = X;
% % sigma_val(:,i) = sigma_perc;
% end
% [v_sort,idx] = sort(val);
% sigma_sort = sigma_val(:,idx);
% X_sort = X_val(:,idx);

% [U,S,V] = svd(W,'econ');
% X = V(:,10)./V(1,10);
% W_hat = W-S(10,10).*U(:,10)*V(:,10)';
% sigma = S(10,10)/sqrt(length(W)-10);
% C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
% sigma_perc = 100*sqrt(diag(C))./X(2:end);
% 
% val = S(1,1)/S(9,9)+1/S(9,9);
% 
% [U2,S2,V2] = svd(W(:,1:7),'econ');
% X2 = V2(:,7)./V2(1,7);
% W_hat2 = W(:,1:7)-S2(7,7).*U2(:,7)*V2(:,7)';
% sigma2 = S2(7,7)/sqrt(length(W)-7);
% C2 = (sigma2^2).*(1+norm(X2(2:end)))*inv(W_hat2(:,2:7)'*W_hat2(:,2:7));
% sigma_perc2 = 100*sqrt(diag(C2))./X2(2:end);
% val2 = S2(1,1)/S2(7,7)+1/S2(7,7);

% cdpr_parameters.underactuated_platform.permutation_matrix =...
%     [0 0 1 0 0 0;
%     0 0 0 1 0 0;
%     0 0 0 0 1 0;
%     0 0 0 0 0 1;
%     1 0 0 0 0 0;
%     0 1 0 0 0 0];
W = [];
p_stat = [];
l_stat = [];
for i=1:15
  filename = strcat('data_opt\44_exp_',num2str(i));
  load(filename);
  pose_stat = mean(p,2);
  ig = cdpr_parameters.underactuated_platform.permutation_matrix*pose_stat;
  ig =ig(1:cdpr_parameters.n_cables);
  cdpr_variables = UpdateIKZeroOrd(pose_stat(1:3),pose_stat(4:end),cdpr_parameters,cdpr_variables);
  l = cdpr_variables.cable_vector;
  p_stat = [p_stat pose_stat];
  l_stat = [l_stat l];
%   for j=1:100:length(p)
%     free = cdpr_parameters.underactuated_platform.permutation_matrix*p(:,i);
%     free = free(cdpr_parameters.n_cables+1:end);
%     freev = cdpr_parameters.underactuated_platform.permutation_matrix*v(:,i);
%     freev = freev(cdpr_parameters.n_cables+1:end);
%     freea = cdpr_parameters.underactuated_platform.permutation_matrix*a(:,i);
%     freea = freea(cdpr_parameters.n_cables+1:end);
%     [act,val] = fsolve(@(v) FunDkLFree(cdpr_parameters,l,free,v),...
%       ig,utilities.fsolve_options_grad);
%     p_new =  cdpr_parameters.underactuated_platform.permutation_matrix'*[act;free];
%     cdpr_variables = UpdateIKZeroOrd(p_new(1:3),p_new(4:end),cdpr_parameters,cdpr_variables);
%     cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobians...
%       (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian,cdpr_variables.D_mat);
%     v_new = cdpr_variables.underactuated_platform.analitic_orthogonal*freev;
%     cdpr_variables = UpdateIKFirstOrd(v_new(1:3),v_new(4:end),cdpr_parameters,cdpr_variables);
%     cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobiansD...
%       (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian_d,cdpr_variables.D_mat,cdpr_variables.D_mat_d);
%     a_new = cdpr_variables.underactuated_platform.analitic_orthogonal*freea+...
%       cdpr_variables.underactuated_platform.analitic_orthogonal_d*freev;
%     cdpr_variables = UpdateIKSecondOrd(a_new(1:3),a_new(4:6),cdpr_parameters,cdpr_variables);
%     W_EE = ComputeIdentificationMatrix(cdpr_parameters,cdpr_variables);
%     W = [W;cdpr_variables.underactuated_platform.geometric_orthogonal'*W_EE];
%   end
end

[U,S,V] = svd(W,'econ');
X = V(:,10)./V(1,10);
W_hat = W-S(10,10).*U(:,10)*V(:,10)';
sigma = S(10,10)/sqrt(length(W)-10);
C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
sigma_perc = 100*sqrt(diag(C))./X(2:end);

val = S(1,1)/S(9,9)+1/S(9,9);