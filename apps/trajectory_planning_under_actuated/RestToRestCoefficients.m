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
    sim_data.cMat=Poly656Coef(sim_data.lims,[sim_data.coeff(1:2,i) sim_data.coeff(3:4,i) sim_data.coeff(5:6,i)]);
 out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(i).fval = fval;
 output(i).t = out.t;
 output(i).platform = out.platform;
 output(i).cables(:) = out.cable(:);
 output(i).tension_vector = out.tension_vector;
 output(i).coefficients = sim_data.coeff(:,i);
 
end

end