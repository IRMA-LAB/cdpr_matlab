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

[cdpr_p, cdpr_v,ws_data, out,record,ut] = ...
  LoadConfigAndInit("Planar_22_nopul_example","Planar"); 

pose = [0;0.5;1;0;0;0];
tau_lim = [10;1000];
cdpr_p.underactuated_platform=cdpr_p.underactuated_platform.SetMask([0;1;1;0;0;0]);
SolveUnderActGeoStatWSPlanar(cdpr_p,cdpr_v,ut,tau_lim,out,pose)