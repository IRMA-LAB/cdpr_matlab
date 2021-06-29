function [parameters,variables, ws_data,outputs,record,utilities] = ...
    LoadConfigAndInit(config_filepath,simulation_title)
%LOADCONFIGANDINIT performs basic initializations of the cdpr simulation.
%   LOADCONFIGANDINIT needs the name of a .json configuration file in
%   string form and a string containing the name of the simulation to be
%   performed. It performs initialization of each class needed for a cdpr
%   simulation and outputs them.
%
%   CONFIG_FILEPATH is a string containing the path of a .json config file.
%   SIMULATION_TITLE is a string containing the name of the simulation which
%   will be used in the graphical output of the process.
%
%   PARAMETERS is an object containing static parameters of the cdpr.
%   VARIABLES is an object containing variables of the cdpr.
%   OUTPUTS is an object containing the logged variables of the simulation.
%   RECORD is an object containing necessary variables and function for
%     graphical display.
%   UTILITIES are objects containing various facilities for numerical
%   resolution, logging, timing, etc...

parameters = CdprParameter(strcat(config_filepath, '.json'));
variables = CdprVar(parameters.n_cables,parameters.pose_dim);

outputFieldsPlatform = fieldnames(variables.platform);
n_elem = parameters.n_cables;
outputFieldsCable = fieldnames(variables.cable(1));

for i=1:numel(outputFieldsPlatform)
    outputs.platform.(outputFieldsPlatform{i}) = [];
end

for j = 1:n_elem
    for i = 1:numel(outputFieldsCable)
        outputs.cable(j).(outputFieldsCable{i}) = [];
    end
end
outputs.index = [];
temp_split = split(config_filepath, '/');
config_filename = temp_split{end};
record = RecordType(parameters,simulation_title, config_filename);
try
    wp_name = strcat(config_filename, '_WS.mat');
    load(wp_name);
    record = record.ResetFigureLimits(ws_data.limits, 8);
catch
    ws_data = [];
end
    

utilities = UtilitiesType;

end