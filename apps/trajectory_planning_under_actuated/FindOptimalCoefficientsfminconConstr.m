function [vector,dummy] = FindOptimalCoefficientsfminconConstr(k,cdpr_p,cdpr_v,ut,sim_data,ind,geom_fun,record)

    K = [k(1:2) k(3:4) k(5:6)];
    sim_data.cMat=Poly656Coef(sim_data.lims,K);
    sol = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,ind,time,orientation,geom_fun),[0 sim_data.dt(ind)],...
        [sim_data.p(4:6,ind);0;0;0],ut.ode45_options);
    t = linspace(0,sol.x(end),100);
    [y yp] = deval(sol,t);
    vector = [];
for i=1:length(t)
    
    pStart = sim_data.p(1:cdpr_p.n_cables,ind);
    pEnd = sim_data.p(1:cdpr_p.n_cables,ind+1);
    normalizedTime = t(i)/sim_data.dt(ind);
    dofs = sim_data.motion_law_function(sim_data,normalizedTime,...
    geom_fun,pStart,pEnd,sim_data.dt(ind));

%     normalizedTime = NormalizedTime(sim_data.coeff(:,ind),timeSpan(i),sim_data.dt(ind));
%     dofs = ActuatedDofs(sim_data,normalizedTime,...
%         geom_fun,pStart,pEnd);
    
    cdpr_v = UpdateIKZeroOrd(dofs(1,:)',y(1:3,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(dofs(2,:)',y(4:6,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(dofs(3,:)',yp(4:6,i),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesDynamicTension(cdpr_v);
    vector = [vector;-cdpr_v.tension_vector];
 
end 

 dummy = [y(1:cdpr_p.n_cables,end)-sim_data.p(4:6,ind+1);
               y(cdpr_p.n_cables+1:2*cdpr_p.n_cables,end)];

    
end