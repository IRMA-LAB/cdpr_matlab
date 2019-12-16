function val = FindOptimalCoefficientsfmincon(k,cdpr_p,cdpr_v,ut,sim_data,i,geom_fun,record)

    K = [k(1:2) k(3:4) k(5:6)];
    sim_data.cMat=Poly656Coef(sim_data.lims,K);
    sol = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,i,time,orientation,sim_data.dt(i),k,geom_fun),[0 sim_data.dt(i)],...
        [sim_data.p(4:6,i);0;0;0],ut.ode45_options);
[y yp] = deval(sol,linspace(0,sol.x(end),100));
%     vector = [y(1:cdpr_p.n_cables,end)-sim_data.p(4:6,i+1);
%               y(cdpr_p.n_cables+1:2*cdpr_p.n_cables,end);
%               yp(cdpr_p.n_cables+1:2*cdpr_p.n_cables,end)];
          %val = max(max(abs(yp)));
          val = 0;
    
end