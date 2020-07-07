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
LoadConfigAndInit("Grab_prototype_44","Grab_prototype_44");
tension_limits = [2;200];
ws_type = WorkspaceType.MAX_MIN_TENSION;
z_lim_inf = -1.5;
mesh_elements = 15;
cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
    utilities,cdpr_outputs,1,tension_limits,folder,record,mesh_elements,z_lim_inf,ws_type);
% 
%     tension_limits = [0.1;10];
%     position = [0;0;-0.2];
%     cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,...
%         utilities,cdpr_outputs,3,tension_limits,folder,record,3,position);
