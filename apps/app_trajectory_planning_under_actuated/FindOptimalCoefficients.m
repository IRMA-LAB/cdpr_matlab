function vector = FindOptimalCoefficients(k,cdpr_p,cdpr_v,ut,sim_data,i,geom_fun,record)

    sol = ode45(@(time,orientation) IntegrableInverseDynamics33(cdpr_p,cdpr_v,...
        sim_data,i,time,orientation,sim_data.dt(i),k,geom_fun),[0 sim_data.dt(i)],...
        [sim_data.p(4:6,i);0;0;0],ut.ode45_options);
[y yp] = deval(sol,sol.x);
    vector = [y(1:cdpr_p.n_cables,end)-sim_data.p(4:6,i+1);
              y(cdpr_p.n_cables+1:2*cdpr_p.n_cables,end);
              yp(cdpr_p.n_cables+1:2*cdpr_p.n_cables,end)];

%     sol = RKSolver(@(time,orientation) IntegrableInverseDynamics33(cdpr_p,cdpr_v,...
%         sim_data,i,time,orientation,sim_data.dt(i),k,geom_fun),...
%         0:ut.t_interval*10:sim_data.dt(i),[sim_data.p(4:6,i);0;0;0]);  
%     vector = [sol.y(1:6-cdpr_p.n_cables,end)-sim_data.p(4:6,i+1);
%                 sol.y(7-cdpr_p.n_cables:2*(6-cdpr_p.n_cables),end);
%                 sol.f(:,end)];
              
    
end