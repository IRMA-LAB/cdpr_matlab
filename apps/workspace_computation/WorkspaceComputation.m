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
[cdpr_parameters, cdpr_variables, ~ ,cdpr_outputs,record,utilities] = ...
LoadConfigAndInit("Grab_prototype_33","Grab_prototype_33");

ws_info = LoadWsInfo("Grab_prototype_33_WS_info");

cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
    utilities,cdpr_outputs,folder,record,ws_info);
record = record.ResetFigureLimits(cdpr_outputs.limits,ws_info.display_grid_divider);
DisplayAndSaveWorkspace(cdpr_parameters,cdpr_variables,cdpr_outputs,ws_info,folder,record);