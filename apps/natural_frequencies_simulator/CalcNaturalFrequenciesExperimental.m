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
folder = '../../data//Robot_Nat_Freq/v_nat_f_26052020/ParsedDataPose';
data_pose_file_name = 'v_pose_nat_f_';

[cdpr_parameters, cdpr_variables,~, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_44","NatFreqComp");
cur_dir = cd(folder);
aa=dir('*.mat');
l = length(aa);
%l=48;
l0 = 1.5.*ones(cdpr_parameters.n_cables,1);
for i=49:l
   load ([data_pose_file_name num2str(i) '.mat'])
   data_p(i).val = pose_print(1:6,200:800);
   data_p(i).val(1:3,:) = data_p(i).val(1:3,:)/1000;
   data_v(i).val = pose_print(7:12,200:800);
   data_a(i).val = pose_print(13:18,200:800);
   
   clear pose_print
end
cd(cur_dir);

for i=49:l
    [data_l(i).val,fval] = lsqnonlin(@(x)CableLengthFromOscillation(x,cdpr_parameters,cdpr_variables,data_p(i)),l0,[],[],utilities.lsqnonlin_options);
    [data_pStat(i).val,fval] = fsolve(@(x)FunDkGsL(cdpr_parameters,data_l(i).val,x),data_p(i).val(:,1),utilities.fsolve_options_grad);
    if (norm(fval)<1e-6)
        data_nat_f(i).val= CalcNatFreqPose(cdpr_parameters,cdpr_variables,data_pStat(i).val);
    else
        disp('what?');
    end
end