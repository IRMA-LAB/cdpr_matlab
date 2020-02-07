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

for i=1:ws_data.counter
cdpr_variables = UpdateIKZeroOrd(ws_data.position(:,i),ws_data.ang_par(:,i),cdpr_parameters,cdpr_variables);
record.SetFrame(cdpr_variables,cdpr_parameters);
pause(0.01)
end