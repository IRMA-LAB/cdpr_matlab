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

% [cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
%   LoadConfigAndInit("Greenline_config.json","DynamicPlanning");

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod.json","calib_mod_WS");

tension_limits = [10;1000];
z_lim_inf = -1;
cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
    utilities,1,tension_limits,folder,record,z_lim_inf);
