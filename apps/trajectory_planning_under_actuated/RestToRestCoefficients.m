function [output] = RestToRestCoefficients(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut,record)
 
for i = 1:sim_data.pNumber-1

  coefficientGuess  = [zeros(2*(cdpr_p.pose_dim-cdpr_p.n_cables),1)];  
  sim_data.dt(i) = sim_data.dt(i)+sim_data.shaper(2,end,i);
  sim_data.T = sim_data.dt(i);
  sim_data.motion_law_function = @ActuatedDofsMod;
  tic
  [sim_data.coeff(:,i),fval,exitFlag1,Foutput]  = fmincon(@(k)0,coefficientGuess,[],[],[],[],[],[],@(k)FindOptimalCoefficientsfminconConstr(k,cdpr_p,cdpr_v,...
    ut,sim_data,i,geom_fun,record),ut.fmincon_options);
    toc
  K = [coefficientGuess(1:2) coefficientGuess(3:4) coefficientGuess(5:6)];
    sim_data.cMat=Poly656Coef(sim_data.lims,K);
    tic
    sol = ode45(@(time,orientation) IntegrableInverseDynamics(cdpr_p,cdpr_v,...
        sim_data,i,time,orientation,geom_fun),[0 sim_data.dt(i)],...
        [sim_data.p(4:6,i);0;0;0],ut.ode45_options);
    t = linspace(0,sol.x(end),100);
    [y yp] = deval(sol,t);  
    solinit.x = t; solinit.y = y; solinit.parameters = coefficientGuess;
  sol = bvp5c(@(time,orientation,k) IntegrableInverseDynamicsMod(cdpr_p,cdpr_v,...
        sim_data,i,time,orientation,k,geom_fun),@(ya,yb,k) bcfun(ya,yb,k,sim_data,i),solinit);
    toc
  sim_data.coeffbvp5c(:,i) = sol.parameters;  
    sim_data.cMat=Poly656Coef(sim_data.lims,[sim_data.coeff(1:2,i) sim_data.coeff(3:4,i) sim_data.coeff(5:6,i)]);
%     cdpr_p.platform.mass = 1.05*cdpr_p.platform.mass;
%     cdpr_p.platform.inertia_mat_G_loc = 1.05*cdpr_p.platform.inertia_mat_G_loc;
%     cdpr_p.platform.pos_PG_loc = 1.05* cdpr_p.platform.pos_PG_loc;
 out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(i).fval = fval;
 output(i).t = out.t;
 output(i).platform = out.platform;
 output(i).cables(:) = out.cable(:);
 output(i).tension_vector = out.tension_vector;
 output(i).coefficients = sim_data.coeff(:,i);
 output(i).coefficientsbvp5c = sim_data.coeffbvp5c(:,i);
 
end

end