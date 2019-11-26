function data = GetDestinations(data,ws_par,cdpr_p,record,ut)

    data.pNumber = 4;
    
    data.p(1:3,1)  = ws_par.workspace_center-[0.3;0;0.5];
    initial_guess = LookForCloseSolution(cdpr_p,ws_par,data.p(1:3,1));
    data.p(4:6,1)  = fsolve(@(v) FunGs(cdpr_p,data.p(1:3,1),v,record),...
    initial_guess,ut.fsolve_options_grad);
    
    data.dt(1) = 2.5; 
    
    data.p(1:3,2)  = ws_par.workspace_center+[0.5;0;-0.5];
    initial_guess = LookForCloseSolution(cdpr_p,ws_par,data.p(1:3,2));
    data.p(4:6,2)  = fsolve(@(v) FunGs(cdpr_p,data.p(1:3,2),v,record),...
    initial_guess,ut.fsolve_options_grad);
    
    data.dt(2) = 2; 
    
    data.p(1:3,3)  = ws_par.workspace_center+[0.1;0;0.1];
    initial_guess = LookForCloseSolution(cdpr_p,ws_par,data.p(1:3,3));
    data.p(4:6,3)  = fsolve(@(v) FunGs(cdpr_p,data.p(1:3,3),v,record),...
    initial_guess,ut.fsolve_options_grad);

    data.dt(3) = 2; 
    
    data.p(1:3,4)  = data.p(1:3,1);
    data.p(4:6,4)  = data.p(4:6,1);
  
end