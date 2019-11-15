clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("Greenline_config.json","DynamicPlanning");

% i = 0;
% for j=1:6
% cdpr_parameters.cable(j).pos_A_loc = cdpr_parameters.cable(j).pos_A_loc*0.01;
% end
% for ang = 0:pi/180:2*pi
%     i = i+1;
%     cdpr_variables = UpdateIKZeroOrd([0;0;-0.3],[ang;pi/3;0],cdpr_parameters,cdpr_variables);
%     cdpr_variables = CalcExternalLoadsStateSpace(cdpr_variables,cdpr_parameters,eye(3));
%     cdpr_variables = CalcCablesTensionStat(cdpr_variables);
%     T(:,i) = cdpr_variables.tension_vector;
%     %record.SetFrame(cdpr_variables,cdpr_parameters);
% end
% figure();
% hold on
% for i=1:6
%     plot(T(i,:))
% end
% hold off

% record.SetFrame(cdpr_variables,cdpr_parameters);
v = [];
%for z = -0.3
    for z = -0.1:-0.0025:-0.35
cdpr_outputs = CalcWorkspace(cdpr_parameters,cdpr_variables,cdpr_outputs,utilities,2,[0;0;z],record);
v = [v; z mean(cdpr_outputs.manip)];
end
plot(v(:,1), v(:,2))