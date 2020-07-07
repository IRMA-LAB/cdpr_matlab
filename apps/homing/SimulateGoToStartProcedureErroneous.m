function start = SimulateGoToStartProcedureErroneous(cdpr_p, cdpr_v, ws_p,rec,ut,T)

reference_position = ws_p.workspace_center;
initial_guess = LookForCloseSolution(cdpr_p,ws_p,reference_position); 
reference_angle = fsolve(@(v) FunGs(cdpr_p,reference_position,v,rec),...
    initial_guess,ut.fsolve_options_grad);

initial_guess = [reference_position;reference_angle];

start.pose = fsolve(@(v) FunDkStat(cdpr_p,T(1).*0.8.*ones(cdpr_p.n_cables,1),v,rec),...
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