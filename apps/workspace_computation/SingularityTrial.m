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


%workspace of grab underactuated prototype
[cdpr_parameters, cdpr_variables, ws_data ,cdpr_outputs,record,utilities] = ...
LoadConfigAndInit("Grab_prototype_44_nominal","Grab_prototype_44_nominal_WS");
cdpr_parameters.underactuated_platform.actuated_mask = [1,2,3,6];
cdpr_parameters.underactuated_platform.unactuated_mask = [4,5];
incr = pi/180;
for i=1:ws_data.counter
pose = [ws_data.position(:,i);ws_data.ang_par(:,i)];
isposdef = 1;
try
[x,~,flag] = fmincon(@(s)0,...
        pose(4:6),[],[],[],[],[],[],...
        @(s)FindSingularityFromStableEquilibrium(cdpr_parameters,cdpr_variables,utilities,pose,s),utilities.brutal_fmincon_options);
end
if (flag>0&&flag<4)
    pose(4:6) = x;
    cdpr_variables = UpdateIKZeroOrd(pose(1:3),pose(4:6),cdpr_parameters,cdpr_variables);
    cdpr_variables = CalcExternalLoads(cdpr_variables,cdpr_parameters);
    cdpr_variables.underactuated_platform = cdpr_variables.underactuated_platform.UpdateGeometricJacobians(cdpr_parameters.underactuated_platform,cdpr_variables.geometric_jacobian);
    [cdpr_variables,~] = UnderactuatedStaticConstraintNoCheck(cdpr_variables);
    record.SetFrame(cdpr_variables,cdpr_parameters);
end
pause(0.01)
end