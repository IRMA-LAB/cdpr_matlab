% InverseDynamicPlanner33
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

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_33","DynamicPlanning");

% fare la cartella output per la roba enorme con dentro un .keep

simulationData = struct();
geometricFunction = @LineFunction;
simulationData = NormalizedPoly7Coefficients(1,simulationData);
simulationData = GetDestinations(simulationData,ws_parameters,cdpr_parameters,record,utilities);
  
[outputDataRTR] = RestToRestCoefficients(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities,record);
[outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
 
DataLoggerStruct(outputDataRTR,folder,'InverseRTR',true,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataSTD,folder,'InverseSTD',true,cdpr_parameters,cdpr_variables,record,utilities);
 