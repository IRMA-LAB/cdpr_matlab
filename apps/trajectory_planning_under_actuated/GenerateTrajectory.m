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
load ..\..\data\planning_files\TEST_IS2
motors_id = [];
data = t';
for i=1:cdpr_parameters.n_cables
motors_id = [motors_id cdpr_parameters.cable(i).id];
data = [data cables(i).complete_length'];
end
motors_id_str = num2str(motors_id);
name = "..\..\data\planning_files\IS";
suffix = name + "_" + motors_id_str(~isspace(motors_id_str));
filepath =  suffix + ".txt";
N = cdpr_parameters.n_cables;
fid = fopen(filepath, 'w');
% 0 cable length
% 1 motor counts
% 2 motor speed 
% 3 motor torque
traj_type = 0;

relative = 0;
fprintf(fid, strcat('%d %d', repmat(' %d', 1, N), '\n'), [traj_type, relative, motors_id]);
fprintf(fid, strcat('%f', repmat(' %f', 1, N), '\n'), data.');
fclose(fid);
fprintf("File saved in %s\n", filepath)