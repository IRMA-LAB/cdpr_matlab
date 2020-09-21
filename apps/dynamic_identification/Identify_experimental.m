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

%workspace of grab underactuated prototype
[cdpr_parameters, cdpr_variables, ~ ,cdpr_outputs,record,utilities] = ...
LoadConfigAndInit("Grab_prototype_44","Grab_prototype_44");