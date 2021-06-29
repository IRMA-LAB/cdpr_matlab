clear
close all
clc

filepath = fileparts(mfilename('fullpath'));
addpath(fullfile(filepath, '/../../config'))
addpath(fullfile(filepath, '/../../data/workspace_files'))
addpath(fullfile(filepath, '/../../libs/cdpr_model'))
addpath(fullfile(filepath, '/../../libs/export_utilities'))
addpath(fullfile(filepath, '/../../libs/numeric'))
addpath(fullfile(filepath, '/../../libs/orientation_geometry'))
addpath(fullfile(filepath, '/../../libs/under_actuated'))
addpath(fullfile(filepath, '/../../libs/over_actuated'))
addpath(fullfile(filepath, '/../../libs/prototype_log_parser'))
addpath(fullfile(filepath, '/../../libs/prototype_log_parser/msgs'))
folder = fullfile(filepath, '/../../data');

tmp_file = '/tmp/cable_robot_app_tmp.txt';
fid = fopen(tmp_file, 'r');
abs_path = fgetl(fid);
fclose(fid);
[config_dir, config_name, config_ext] = fileparts(abs_path);

[cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
    LoadConfigAndInit(fullfile(config_dir, config_name), "HomingTest44");
% [cdpr_parameters, cdpr_variables, ws_parameters, cdpr_outputs,record,utilities] = ...
%   LoadConfigAndInit("Grab_prototype_44","HomingTest44");
homing_info = LoadHomingInfo("Homing_info");

start = SimulateGoToStartProcedureErroneous...
    (cdpr_parameters, cdpr_variables,ws_parameters,record,utilities,homing_info.tension_limits);
home = SimulateGoToStartProcedureIdeal...
    (cdpr_parameters, cdpr_variables,start.pose,record,utilities,homing_info.tension_limits);

% Here automatic initial guess for the homing algorithm is generated,
% making use of banal extimation of the workspace center (geometrical
% property  of the robot) and acquired data. No need for user interaction.

%[imported_data_coarse, ~] = parseCableRobotLogFile('data.log');
[imported_data_coarse, ~] = parseCableRobotLogFile('/tmp/cable-robot-logs/data.log');
[delta_l,delta_sw] = Reparse(imported_data_coarse.actuator_status.values,...
    cdpr_parameters);
init_guess = GenerateInitialGuess(cdpr_parameters,cdpr_variables,delta_l,home,record,utilities);

init_guess = [start.l;start.sw;init_guess];
[sol,resnorm,residual,exitflag,output] = lsqnonlin(@(v)HomingOptimizationFunction...
    (cdpr_parameters,record,delta_l,delta_sw,v),init_guess ,[],[],utilities.lsqnonlin_options_grad);
l0 = sol(1:cdpr_parameters.n_cables,1);
s0 = sol(cdpr_parameters.n_cables+1:2*cdpr_parameters.n_cables,1);
[pose0,resnormp,residualp,exitflagp,outputp] = lsqnonlin(@(v)FunDkGsSwL...
    (cdpr_parameters,l0,s0,v,record),start.pose ,[],[],utilities.lsqnonlin_options_grad);

j_struct.init_pose = pose0';

json.startup
ofilepath = fullfile(filepath, '../../data/homing_results.json');
json.write(j_struct, ofilepath)
fprintf('Results dumped in "%s"\n', ofilepath)
