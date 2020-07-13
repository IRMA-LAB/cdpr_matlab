function [output] = StandardInverseSimulator(cdpr_p,cdpr_v,...
     sim_data,ut)

for ind = 1:length(sim_data)
 
 sim_data(ind).T = sim_data(ind).dt;
 sim_data(ind).motion_law_function = @ActuatedDofsMod;
 sim_data(ind).cMat=TrapzCoef(sim_data(ind).lim);
 out = GenerateOutputUnderActuated(cdpr_p,cdpr_v,sim_data(ind),ut);
 output(ind).t = out.t;
 output(ind).platform = out.platform;
 output(ind).cables(:) = out.cable(:);
 output(ind).tension_vector = out.tension_vector;

end

end