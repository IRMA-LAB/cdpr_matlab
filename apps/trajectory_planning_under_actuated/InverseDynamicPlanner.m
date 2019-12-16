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
simulationData.lims = [1/3;1/3];
simulationData = GetDestinations(simulationData,ws_parameters,cdpr_parameters,record,utilities,geometricFunction);
 
tic
[outputDataRTR] = RestToRestCoefficients(cdpr_parameters,cdpr_variables,...
      simulationData,geometricFunction,utilities,record);
tRTR = toc;
tic
[outputDataIS] = ShapedInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
tIS = toc;
tic;
[outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
tSTD = toc; 
 
DataLoggerStruct(outputDataRTR,folder,'InverseRTR',false,cdpr_parameters,cdpr_variables,record,utilities); 
DataLoggerStruct(outputDataIS,folder,'InverseIS',false,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataSTD,folder,'InverseSTD',false,cdpr_parameters,cdpr_variables,record,utilities);
 