clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_copt.json","homing");

reference_position = cdpr_parameters.workspace_center-[0;0;1.5];
 geometric_static_mask = [1;1;1;0;0;0];
reference_angle = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_parameters,record,reference_position,v,geometric_static_mask),...
    zeros(cdpr_parameters.pose_dim-cdpr_parameters.n_cables,1),utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(reference_position,reference_angle,cdpr_parameters,cdpr_variables);
for i=1:cdpr_parameters.n_cables
  l0(i,1) = cdpr_variables.cable(i).length;
  s0(i,1) = cdpr_variables.cable(i).swivel_ang;
end
p0 = cdpr_variables.platform.pose;
start_pose = [reference_position; reference_angle];
tension_module = [40;40;40];
home_pose = fsolve(@(v) CalcPoseFromTensionModule(...
    cdpr_parameters,record,tension_module,v),...
    start_pose,utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(home_pose(1:cdpr_parameters.n_cables),...
  home_pose(cdpr_parameters.n_cables+1:cdpr_parameters.pose_dim),cdpr_parameters,cdpr_variables);
for i=1:cdpr_parameters.n_cables
  home_l(i,1) = cdpr_variables.cable(i).length;
  home_swivel_ang(i,1) = cdpr_variables.cable(i).swivel_ang;
end
        
% Here automatic initial guess for the homing algorithm is generated,
% making use of banal extimation of the workspace center (geometrical
% property  of the robot) and
% acquired data. No need for user interaction.

imported_data = importdata('homingDataFile.txt');
sizes=size(imported_data);
vLength = [imported_data(1,1:sizes(2)/2)'];
vSwivelAngle = [imported_data(1,sizes(2)/2+1:sizes(2))'];
 for j=2:sizes(1)
     vLength = [vLength;imported_data(j,1:sizes(2)/2)'];
     vSwivelAngle = [vSwivelAngle;imported_data(j,sizes(2)/2+1:sizes(2))'];
 end
 [meas_stage,~] = size(vLength); 
 meas_stage = meas_stage/cdpr_parameters.n_cables;
 cables_acquisitions = meas_stage/cdpr_parameters.n_cables;
 
 % Automatic Initial Guess Generation for Homing
for i=1:meas_stage
  cdpr_v_ideal = cdpr_variables;
  l_current = home_l+vLength(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables);
  l(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables) = l_current;
  swivel_ang_current = home_swivel_ang+vSwivelAngle(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables);
  swivel_ang(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables) = swivel_ang_current;
  if(mod(i-1,cables_acquisitions) == 0)
    start_sol = home_pose;
  end
  [sol,fval] = fsolve(@(v)CalcDirectKinematicsLength(cdpr_parameters,...
    record,l_current,eye(3),v),start_sol,utilities.fsolve_options);
  start_sol = sol; 
  vPose(cdpr_parameters.pose_dim*(i-1)+1:i*cdpr_parameters.pose_dim,1) = sol;
  cdpr_v_ideal = UpdateIKZeroOrd(sol(1:3,1),...
    sol(4:cdpr_parameters.pose_dim,1),cdpr_parameters,cdpr_v_ideal);
  cdpr_v_ideal = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),...
    cdpr_parameters,cdpr_v_ideal);
  cdpr_v_ideal.ext_load = CalcExternalLoads(cdpr_parameters.platform,...
    cdpr_v_ideal.platform.rot_mat,cdpr_v_ideal.platform.H_mat,cdpr_v_ideal.platform.pos_PG_glob,eye(3));
  cdpr_v_ideal.tension_vector = CalcCablesTension(cdpr_v_ideal);
  record.SetFrame(cdpr_v_ideal,cdpr_parameters);
  
end

% Contruct external patameter and initial guess
initial_guess1 = [home_swivel_ang;home_l;vPose];
initial_guessG = [0;0;initial_guess1];
initial_guess2 = [zeros(3,1);ones(3,1);vPose];

tau = [0.961590794366752;0.961590794366752;0.962250315307843];
for i=1:length(vLength)/cdpr_parameters.n_cables
  vLength((i-1)*cdpr_parameters.n_cables+1:i*cdpr_parameters.n_cables,1) = vLength((i-1)*cdpr_parameters.n_cables+1:i*cdpr_parameters.n_cables).*tau(1:cdpr_parameters.n_cables,1);  
end 
vLength2 = vLength(4:end);
vSwivelAngle2 = vSwivelAngle(4:end);
vPose2 = vPose(7:end);
initial_guess2 = [home_swivel_ang;home_l;vPose2];
    tic 
        [sol2,resnorm2,residual2,exitflag2,output2] = lsqnonlin(@(v)HomingOptimizationFunction6...
        (cdpr_parameters,record,vLength,vSwivelAngle,v),initial_guess1,[],[],utilities.lsqnonlin_options_grad);
    t2 = toc;
    
    tic 
        [sol3,resnorm3,residual3,exitflag3,output3] = lsqnonlin(@(v)HomingOptimizationFunction6...
        (cdpr_parameters,record,vLength2,vSwivelAngle2,v),initial_guess2,[],[],utilities.lsqnonlin_options_grad);
    t3 = toc;
l_start = [1.355272545474591;1.354450977328913;1.399180279334799];
fileID = fopen('/home/labpc/Desktop/homingResultFile.txt','w');
fprintf(fileID,'%12.10f \n %12.10f \n %12.10f \n %12.10f \n %12.10f \n %12.10f \n %12.10f \n %12.10f \n %12.10f \n', sol2(4:6),sol2(1:3),l_start);
fclose(fileID);   