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
[cdpr_parameters, cdpr_variables, ~ ,cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("config_calib","calib_mod_WS");
tension_limits = [10;1000];
z_lim_inf = -1.5;
cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
    utilities,1,tension_limits,folder,record,z_lim_inf);

% [cdpr_parameters, cdpr_variables, ~ ,cdpr_outputs,record,utilities] = ...
%   LoadConfigAndInit("Greenline_config","Greenline_WS");
% 
%     tension_limits = [0.1;10];
%     position = [0;0;-0.3];
%     cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
%         utilities,2,tension_limits,folder,record,position);
