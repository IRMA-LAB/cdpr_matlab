clear all
clc
addpath('Common')

[cdpr_parameters, cdpr_variables, cdpr_outputs,record,utilities] = ...
  LoadConfigAndInit("my_config_3.json","homing33");

% % Generation of the "roughly" estimated home pose
tension_module = 40;
tension_module_max = 60;
n_delta = 10;
DT = (tension_module_max-tension_module)/n_delta;
%random_tension = [45;35;40];
random_tension = [40;40;40];
ii=0;
count = 0;
for x = 0:0.1:2.6
    for y = -1.3:0.1:1.7
        for z=-2.6:0.1:-1
           for a=0:pi/6:2*pi
             for b=0:pi/6:2*pi
               for cc=0:pi/6:2*pi
            ii = ii + 1;
            solution_est = [x;y;z;a;b;cc];
           % solution_est = [x;y;z;0;0;-0.5];
            [solution_calc(:,ii),fval(:,ii),exitflag(ii),output] = fsolve(@(v)CalcPoseFromTensionModuleScalar(...
              cdpr_parameters,record,random_tension,v),solution_est,utilities.fsolve_options_grad);
            if (ii == 1)
              count = count +1;
              diffSol = solution_calc(:,ii);
              pose = solution_calc(:,ii);
              cdpr_variables = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_parameters,cdpr_variables);
              for i=1:cdpr_parameters.n_cables
                diffl0(i,1) = cdpr_variables.cable(i).length;
                diffs0(i,1) = cdpr_variables.cable(i).swivel_ang;
              end
              record.SetFrame(cdpr_variables,cdpr_parameters);
              isNotStable = AssertStability33(cdpr_parameters,solution_calc(:,ii));
              frame(ii) = getframe(record.figure_handle); 
            else
              if (exitflag(ii)>0 )
                [r,c] = size(diffSol);
                 flag = 0;
                for iii=1:c
                  if (norm(solution_calc(1:3,ii)-diffSol(1:3,iii))>0.1)
                  flag = flag+1;
                  end
                end
                if (flag == c)
                  count = count +1;
                  diffSol = [diffSol solution_calc(:,ii)];
                  pose = solution_calc(:,ii);
                  cdpr_variables = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_parameters,cdpr_variables);
                  for i=1:cdpr_parameters.n_cables
                    l0(i,1) = cdpr_variables.cable(i).length;
                    s0(i,1) = cdpr_variables.cable(i).swivel_ang;
                  end
                  diffl0 = [diffl0 l0];
                  diffs0 = [diffs0 s0];
                  record.SetFrame(cdpr_variables,cdpr_parameters);
                  isNotStable = [isNotStable AssertStability33(cdpr_parameters,solution_calc(:,ii))];
                  frame(count) = getframe(record.figure_handle);
                end
              end
            end
               end
             end
          end
        end
    end
end
 reference_position = cdpr_parameters.workspace_center-[0.312;0.256;1.5];
 geometric_static_mask = [1;1;1;0;0;0];
reference_angle = fsolve(@(v) CalcGenericGeometricStatic(...
    cdpr_parameters,record,reference_position,v,geometric_static_mask),...
    zeros(cdpr_parameters.pose_dim-cdpr_parameters.n_cables,1),utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(reference_position,reference_angle,cdpr_parameters,cdpr_variables);
cdpr_variables = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),...
    cdpr_parameters,cdpr_variables);
  cdpr_variables.ext_load = CalcExternalLoads(cdpr_parameters.platform,...
    cdpr_variables.platform.rot_mat,cdpr_variables.platform.H_mat,cdpr_variables.platform.pos_PG_glob,eye(3));
  cdpr_variables.tension_vector = CalcCablesTension(cdpr_variables);

p0 = cdpr_variables.platform.pose;
start_pose = [reference_position; reference_angle];

home_pose = fsolve(@(v) CalcPoseFromTensionModule(...
    cdpr_parameters,record,tension_module,v),...
    start_pose,utilities.fsolve_options);
cdpr_variables = UpdateIKZeroOrd(home_pose(1:cdpr_parameters.n_cables),...
  home_pose(cdpr_parameters.n_cables+1:cdpr_parameters.pose_dim),cdpr_parameters,cdpr_variables);
for i=1:cdpr_parameters.n_cables
  home_l(i,1) = cdpr_variables.cable(i).length;
  home_swivel_ang(i,1) = cdpr_variables.cable(i).swivel_ang;
end


% Simulation of the robot movement and data acquisition
cdpr_variables = UpdateIKZeroOrd(home_pose(1:3,1),home_pose(4:end,1),cdpr_parameters,cdpr_variables);
cdpr_variables.tension_vector = tension_module.*ones(cdpr_parameters.n_cables,1);
meas_stage = 0;
for index = 1:cdpr_parameters.n_cables
  cdpr_v_ideal = cdpr_variables;
  cables_acquisitions = 0;
  for cc = 1:2
    if (cc == 1)
      signF = 1;
    else
      signF = -1;
    end
    for j=1:n_delta
      meas_stage = meas_stage + 1;
      cables_acquisitions = cables_acquisitions + 1;
      solution_est = cdpr_v_ideal.platform.pose;
      for k=1:cdpr_parameters.n_cables
        if (k == index)
          if (j == 1 && cc == 1)
            cdpr_variables.tension_vector(k,1) = tension_module+DT;
          elseif  (j == 1 && cc == 2)
            cdpr_variables.tension_vector(k,1) = tension_module_max-DT;
          else
            cdpr_variables.tension_vector(k,1) = cdpr_variables.tension_vector(k,1) + signF*DT;
          end
        else
          cdpr_variables.tension_vector(k,1) = tension_module;
        end
      end
      solution_calc = fsolve(@(v)CalcPoseFromTensionModuleScalar(...
        cdpr_parameters,record,cdpr_variables.tension_vector,v),solution_est,utilities.fsolve_options_grad);
      cdpr_v_ideal = UpdateIKZeroOrd(solution_calc(1:3,1),...
        solution_calc(4:cdpr_parameters.pose_dim,1),cdpr_parameters,cdpr_v_ideal);
      cdpr_v_ideal = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),...
        cdpr_parameters,cdpr_v_ideal);
      cdpr_v_ideal.ext_load = CalcExternalLoads(cdpr_parameters.platform,...
        cdpr_v_ideal.platform.rot_mat,cdpr_v_ideal.platform.H_mat,cdpr_v_ideal.platform.pos_PG_glob,eye(3));
      cdpr_v_ideal.tension_vector = cdpr_variables.tension_vector;
      record.SetFrame(cdpr_v_ideal,cdpr_parameters);
      i = 0;
      for j = cdpr_parameters.n_cables*(meas_stage-1)+1:meas_stage*cdpr_parameters.n_cables
        i = i+1;
        % Log of the simulated measurements
        vLength(j,1) = cdpr_v_ideal.cable(i).length - cdpr_variables.cable(i).length;
        vSwivelAngle(j,1) = cdpr_v_ideal.cable(i).swivel_ang - cdpr_variables.cable(i).swivel_ang;
        vTension(j,1) = cdpr_v_ideal.tension_vector(i,1);
      end
    end
  end
end

vLength = [zeros(cdpr_parameters.n_cables,1);vLength];
vSwivelAngle = [zeros(cdpr_parameters.n_cables,1);vSwivelAngle];

[r,c] = size(diffSol);
for ii = 1:c
  clear vPose
% Automatic Initial Guess Generation for Homing
for i=1:meas_stage
  cdpr_v_ideal = cdpr_variables;
  l_current = diffl0(:,ii)+vLength(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables);
  l(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables) = l_current;
  swivel_ang_current = diffs0(:,ii)+vSwivelAngle(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables);
  swivel_ang(cdpr_parameters.n_cables*(i-1)+1:i*cdpr_parameters.n_cables) = swivel_ang_current;
  if(mod(i-1,cables_acquisitions) == 0)
    start_sol = diffSol(:,ii);
  end
  [sol,fval] = fsolve(@(v)CalcDirectKinematicsLengthSwivel(cdpr_parameters,...
    record,l_current,swivel_ang_current,v),start_sol,utilities.fsolve_options);
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

vPose = [diffSol(:,ii);vPose];
initial_guess1 = [diffs0(:,ii);diffl0(:,ii);vPose];
tic
[sol2(:,ii),resnorm2(ii,1),residual2,exitflag2,output2] = lsqnonlin(@(v)HomingOptimizationFunction6...
  (cdpr_parameters,record,vLength,vSwivelAngle,v),initial_guess1 ,[],[],utilities.lsqnonlin_options_grad);
realSol2(:,ii) = sol2(1:2*cdpr_parameters.n_cables+6,ii);
errSol2(ii,1) = norm(realSol2(:,ii)-[home_swivel_ang;home_l;home_pose]);
t2(ii) = toc;

end
% HomingProcedure
%  clc
%  tic
%    [sol1,resnorm1(jj,1),residual1,exitflag1,output1] = lsqnonlin(@(v)HomingOptimizationFunction6...
%    (cdpr_parameters,record,vLength,vSwivelAngle,v),initial_guess1 ,[],[],utilities.lsqnonlin_options);
%  realSol1(:,jj) = sol1(1:2*cdpr_parameters.n_cables+6,1);
% errSol1(jj,1) = norm(realSol1(:,jj)-[home_swivel_ang;home_l;home_pose]);
%  t1(jj) = toc;
% clc
tic
[sol2,resnorm2(jj,1),residual2,exitflag2,output2] = lsqnonlin(@(v)HomingOptimizationFunction6...
  (cdpr_parameters,record,vLength,vSwivelAngle,v),initial_guess1 ,[],[],utilities.lsqnonlin_options_grad);
realSol2(:,jj) = sol2(1:2*cdpr_parameters.n_cables+6,1);
errSol2(jj,1) = norm(realSol2(:,jj)-[home_swivel_ang;home_l;home_pose]);
t2(jj) = toc;

m(jj) = meas_stage;


