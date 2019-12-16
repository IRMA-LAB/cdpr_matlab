function [output] = StandardInverseSimulator(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut)

for i = 1:sim_data.pNumber-1
    
 sim_data.dt(i) = sim_data.dt(i)+sim_data.shaper(2,end,i);
 sim_data.T = sim_data.dt(i);
 sim_data.cMat=TrapzModCoef(sim_data.lims);
 sim_data.motion_law_function = @ActuatedDofsMod;
 out = GenerateOutputUnderActuated(i,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(i).t = out.t;
 output(i).platform = out.platform;
 output(i).cables(:) = out.cable(:);
 output(i).tension_vector = out.tension_vector;

end

end