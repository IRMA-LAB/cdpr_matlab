function [vector,dummy] = FindOptimalCoefficientsAndTime(k,cdpr_p,cdpr_v,ut,sim_data,ind,geom_fun,record)

    sim_data.dt(ind) = k(end);
    K = [k(1:2) k(3:4) k(5:6)];
    sim_data.cMat=Poly656Coef(sim_data.lims,K);
    sol_guess = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,ind,time,orientation,geom_fun),[0 sim_data.dt(ind)],...
        [sim_data.p(4:6,ind);0;0;0],ut.ode45_options);
    solinit.x = sol_guess.x; solinit.y = sol_guess.y; solinit.parameters = k(1:end-1);
    sol = bvp5c(@(time,orientation,kk) IntegrableInverseDynamicsMod(cdpr_p,cdpr_v,...
            sim_data,ind,time,orientation,kk,geom_fun),@(ya,yb,kk) bcfun(ya,yb,kk,sim_data,ind),solinit,ut.bvp_options);

        K = [sol.parameters(1:2) sol.parameters(3:4) sol.parameters(5:6)];
        sim_data.cMat=Poly656Coef(sim_data.lims,K);
        sol_updated = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
            sim_data,ind,time,orientation,geom_fun),[0 sim_data.dt(ind)],...
            [sim_data.p(4:6,ind);0;0;0],ut.ode45_options);

    vector = [];
    t = linspace(0,sol_updated.x(end),100);
    [y yp] = deval(sol_updated,t);
    vector = [];
for i=1:length(t)
    
    pStart = sim_data.p(1:cdpr_p.n_cables,ind);
    pEnd = sim_data.p(1:cdpr_p.n_cables,ind+1);
    normalizedTime = t(i)/sim_data.dt(ind);
    dofs = sim_data.motion_law_function(sim_data,normalizedTime,...
    geom_fun,pStart,pEnd,sim_data.dt(ind));
    cdpr_v = UpdateIKZeroOrd(dofs(1,:)',y(1:3,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(dofs(2,:)',y(4:6,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(dofs(3,:)',yp(4:6,i),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesDynamicTension(cdpr_v);
    vector = [vector;-cdpr_v.tension_vector];
 
end 

 vector = [vector-sim_data.tension_lims(1);-vector-sim_data.tension_lims(2)];
 dummy = [];

    
end