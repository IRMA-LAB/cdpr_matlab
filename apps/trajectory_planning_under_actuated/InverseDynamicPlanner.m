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

% [cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
%   LoadConfigAndInit("Grab_prototype_33","DynamicPlanning");
[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_33","DynamicPlanning_Grab_prototype_33");

% fare la cartella output per la roba enorme con dentro un .keep

simulationData = struct();
geometricFunction = @LineFunction;
simulationData = GetDestinations(simulationData,ws_parameters,cdpr_parameters,...
    cdpr_variables,record,utilities,geometricFunction);
 
% tic
% [outputDataRTR] = RestToRestCoefficients(cdpr_parameters,cdpr_variables,...
%       simulationData,geometricFunction,utilities,record);
% tRTR = toc;
%  tic
% [outputDataRTRMT] = RestToRestCoefficientsMinimumTime(cdpr_parameters,cdpr_variables,...
%       simulationData,geometricFunction,utilities,record);
% tRTRMT = toc;
tic
[outputDataIS] = ShapedInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
tIS = toc;
tic;
[outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
     simulationData,geometricFunction,utilities);
tSTD = toc; 
 
% DataLoggerStruct(outputDataRTR,folder,'InverseRTR',false,cdpr_parameters,cdpr_variables,record,utilities);
% DataLoggerStruct(outputDataRTRMT,folder,'InverseRTRMT',true,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataIS,folder,'InverseIS',true,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataSTD,folder,'InverseSTD',true,cdpr_parameters,cdpr_variables,record,utilities);
 