function start = SimulateGoToStartProcedureErroneous(cdpr_p, cdpr_v, ws_p,rec,ut,T)

reference_position = ws_p.workspace_center;
ang = LookForCloseSolution(cdpr_p,ws_p,reference_position); 
pose = [reference_position;ang];
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars...
            (cdpr_p.underactuated_platform,0,pose);
         
[cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),fval] =...
  fsolve(@(v) FunGs(cdpr_p,cdpr_v.underactuated_platform.pose_P(1:cdpr_p.n_cables),v,rec),...
    cdpr_v.underactuated_platform.pose_P(cdpr_p.n_cables+1:end),ut.fsolve_options_grad);

initial_guess = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

[start.pose,fval] = fsolve(@(v) FunDkStat(cdpr_p,T(1).*0.8.*ones(cdpr_p.n_cables,1),v,rec),...
    initial_guess,ut.fsolve_options_grad);
cdpr_v = UpdateIKZeroOrd(start.pose(1:3),...
  start.pose(4:end),cdpr_p,cdpr_v);

start.l = zeros(cdpr_p.n_cables,1);
start.sw = zeros(cdpr_p.n_cables,1);
for i=1:cdpr_p.n_cables
  start.l(i,1) = cdpr_v.cable(i).complete_length;
  start.sw(i,1) = cdpr_v.cable(i).swivel_ang;
end


end