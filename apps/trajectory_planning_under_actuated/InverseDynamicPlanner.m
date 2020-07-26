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
addpath('../../libs/over_actuated')
folder = '../../data';

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Grab_prototype_44","RTR_Grab_prototype_44");

% fare la cartella output per la roba enorme con dentro un .keep
plan_info = LoadPlanInfo("Grab_prototype_44_planning_info");
plan_info = GetFeasibleDestinations(plan_info,ws_parameters,cdpr_parameters,...
    cdpr_variables,record,utilities);
% simulationData = struct();
% geometricFunction = @LineFunction;
% simulationData = GetDestinations(simulationData,ws_parameters,cdpr_parameters,...
%     cdpr_variables,record,utilities,geometricFunction);
 
% tic
% [outputDataRTR] = RestToRestCoefficients(cdpr_parameters,cdpr_variables,...
%       simulationData,geometricFunction,utilities,record);
% tRTR = toc;
%  tic
% [outputDataRTRMT] = RestToRestCoefficientsMinimumTime(cdpr_parameters,cdpr_variables,...
%       simulationData,geometricFunction,utilities,record);
% tRTRMT = toc;
% tic
% [outputDataIS] = ShapedInverseSimulator(cdpr_parameters,cdpr_variables,...
%      plan_info,utilities);
% tIS = toc;
tic;
[outputDataSTD] = StandardInverseSimulator(cdpr_parameters,cdpr_variables,...
     plan_info,utilities);
tSTD = toc; 
 
% DataLoggerStruct(outputDataRTR,folder,'InverseRTR',false,cdpr_parameters,cdpr_variables,record,utilities);
% DataLoggerStruct(outputDataRTRMT,folder,'InverseRTRMT',true,cdpr_parameters,cdpr_variables,record,utilities);
% DataLoggerStruct(outputDataIS,folder,'TEST_IS',false,cdpr_parameters,cdpr_variables,record,utilities);
DataLoggerStruct(outputDataSTD,folder,'TEST_STD',false,cdpr_parameters,cdpr_variables,record,utilities);
 