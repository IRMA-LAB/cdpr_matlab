% InverseDynamicPlanner33
clear all
close all
clc
addpath('../../config')
%addpath('../../data/files_workspace')
addpath('../../libs/fun_cdpr_model')
addpath('../../libs/fun_export_utilities')
addpath('../../libs/fun_numeric')
addpath('../../libs/fun_orientation_geometry')
addpath('../../libs/fun_under_actuated')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_calib_mod.json","DynamicPlanning");
folder = 'C:\Users\EdoPortable\OneDrive - Alma Mater Studiorum Università di Bologna\Work\CableRobotCodes\Results\';
% fare la cartella output per la roba enorme con dentro un .keep

simulationData = struct();
geometricFunction = @LineFunction;
simulationData = NormalizedPoly7Coefficients(1,simulationData);
simulationData = GetDestinations(simulationData,cdpr_parameters,record,utilities);
  
[outputDataRTR] = RestToRestCoefficients33(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities,record);
[outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
 
DataLoggerStruct(outputDataRTR,folder,'InverseRTR',true,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataSTD,folder,'InverseSTD',true,cdpr_parameters,cdpr_variables,record,utilities);
 