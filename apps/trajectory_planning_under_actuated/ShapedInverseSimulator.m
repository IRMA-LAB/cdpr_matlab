function [output] = ShapedInverseSimulator(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut)

for ind = 1:sim_data.pNumber-1
 
 sim_data.T = sim_data.dt(ind)+sim_data.shaper(2,end,ind);  
 sim_data.motion_law_function = @ShapedActuatedDofs;
 sim_data.cMat=TrapzModCoef(sim_data.lims);
 out = GenerateOutputUnderActuated(ind,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
 output(ind).t = out.t;
 output(ind).platform = out.platform;
 output(ind).tension_vector = out.tension_vector;
 output(ind).cables(:) = out.cable(:);
       
end

end