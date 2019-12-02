function [output] = RestToRestCoefficients(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut,record)

for i = 1:sim_data.pNumber-1
  coefficientGuess  = zeros(2*(cdpr_p.pose_dim-cdpr_p.n_cables),1);  
  [sim_data.coeff(:,i),fval,exitFlag1,Foutput] = fsolve(@(k) FindOptimalCoefficients(k,cdpr_p,cdpr_v,...
    ut,sim_data,i,geom_fun,record),coefficientGuess,ut.fsolve_options);
 out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(i).fval = fval;
 output(i).t = out.t;
 output(i).platform = out.platform;
 output(i).cables(:) = out.cable(:);
 output(i).coefficients = sim_data.coeff(:,i);
 
end

end