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
cdpr_parameters.underactuated_platform.permutation_matrix =...
    [0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    1 0 0 0 0 0;
    0 1 0 0 0 0];
X = [1;0.001994862439784;-0.002457515786185;0.198435218141939;0.056913927267093;0.062697598947814;0.031368876196682;2.044802000072844e-04;-2.038606065108885e-04;1.638902098250699e-04];
X_opt = [1;0.001573187331411;-0.002457399385454;0.199936618811935;0.056283644846570;0.062719966247648;0.032238859969719;4.797807340533470e-05;-2.490752565591076e-04;3.436065678983215e-04];
k=0;
for i=1:36
   filename = strcat('data\44_exp_',num2str(i));
   load(filename);
   pose_stat = mean(p,2);
   W = [];
   for j=1:1:length(p)
   cdpr_variables = UpdateIKZeroOrd(p(1:3,j),p(4:end,j),cdpr_parameters,cdpr_variables);
     cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateJacobians...
        (cdpr_parameters.underactuated_platform,cdpr_variables.analitic_jacobian,cdpr_variables.D_mat);
     cdpr_variables = UpdateIKFirstOrd(v(1:3,j),v(4:end,j),cdpr_parameters,cdpr_variables);
     cdpr_variables = UpdateIKSecondOrd(a(1:3,j),a(4:end,j),cdpr_parameters,cdpr_variables);
     W_EE = ComputeIdentificationMatrix(cdpr_parameters,cdpr_variables);
    W = [W;cdpr_variables.underactuated_platform.geometric_orthogonal'*W_EE];
   end
   
   e(:,i) = W*X_opt;
   norme(1,i) = norm(e(:,i))/norm(W(:,1));
   switch i
     case {31,20,12,1,6,10,27,28,24,15,7,16,29,5,11,35,2,30,34,4,25,3}
       k= k+1
   meane(1,k) = sum(e(:,i).^2)/length(e(:,i));
     otherwise
       
   end

end
% [v_sort,idx] = sort(val);
% sigma_sort = sigma_val(:,idx);
% X_sort = X_val(:,idx);

[U,S,V] = svd(W,'econ');
X = V(:,10)./V(1,10);
W_hat = W-S(10,10).*U(:,10)*V(:,10)';
sigma = S(10,10)/sqrt(length(W)-10);
C = (sigma^2).*(1+norm(X(2:end)))*inv(W_hat(:,2:10)'*W_hat(:,2:10));
sigma_perc = 100*sqrt(diag(C))./X(2:end);

val = S(1,1)/S(9,9)+1/S(9,9);

[U2,S2,V2] = svd(W(:,1:7),'econ');
X2 = V2(:,7)./V2(1,7);
W_hat2 = W(:,1:7)-S2(7,7).*U2(:,7)*V2(:,7)';
sigma2 = S2(7,7)/sqrt(length(W)-7);
C2 = (sigma2^2).*(1+norm(X2(2:end)))*inv(W_hat2(:,2:7)'*W_hat2(:,2:7));
sigma_perc2 = 100*sqrt(diag(C2))./X2(2:end);
val2 = S2(1,1)/S2(7,7)+1/S2(7,7);
