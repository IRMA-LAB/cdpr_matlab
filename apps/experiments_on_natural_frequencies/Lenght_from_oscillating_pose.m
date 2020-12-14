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
folder = 'EE_pose_vicon';
dest_file_name = 'vicon_data_f_';
data_pose_file_name = 'vicon_pose_';

[cdpr_parameters, cdpr_variables,~, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_44","NatFreqComp");
cur_dir = cd(folder);
aa=dir('*.mat');
l = length(aa);
%l=48;
l0 = 1.5.*ones(cdpr_parameters.n_cables,1);
offset = 0;
for i=73:108
    try
   load ([data_pose_file_name num2str(i) '.mat'])
   data_p(i).val = pose(1:6,:);
   clear pose
    end
end
cd(cur_dir);

for i=73:108
    try
    [data_l(i).val,fval] = lsqnonlin(@(x)CableLengthFromOscillation(x,cdpr_parameters,cdpr_variables,data_p(i)),l0,[],[],utilities.lsqnonlin_options);
    [data_pStat(i).val,fval] = fsolve(@(x)FunDkGsL(cdpr_parameters,data_l(i).val,x),data_p(i).val(:,1),utilities.fsolve_options_grad);
    if (norm(fval)<1e-6)
        data_nat_f(i).val= CalcNatFreqPose(cdpr_parameters,cdpr_variables,data_pStat(i).val);
        res.l = data_l(i).val;
        res.pose = data_pStat(i).val;
        res.f = data_nat_f(i).val;
        save([folder '/' dest_file_name num2str(i) '.mat'], 'res');
    else
        disp('what?');
    end
    end
end