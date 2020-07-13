function [output] = ShapedInverseSimulator(cdpr_p,cdpr_v,...
     sim_data,ut)

for ind = 1:length(sim_data)
 sim_data(ind).T = sim_data(ind).dt+sim_data(ind).shaper(2,end);  
 sim_data(ind).motion_law_function = @ShapedActuatedDofs;
 sim_data(ind).cMat=TrapzCoef(sim_data(ind).lim);
 out = GenerateOutputUnderActuated(cdpr_p,cdpr_v,sim_data(ind),ut);
 output(ind).t = out.t;
 output(ind).platform = out.platform;
 output(ind).tension_vector = out.tension_vector;
 output(ind).cables(:) = out.cable(:);
%  
%  sim_data.dt(ind) = t1*0.6;
%  sim_data.lims(:) = [0.3;0.3];
%  sim_data.T = sim_data.dt(ind)+sim_data.shaper(2,end,ind);  
%  sim_data.motion_law_function = @ShapedActuatedDofs;
%  sim_data.cMat=TrapzCoef(sim_data.lims(:,ind));
%  out = GenerateOutputUnderActuated(ind,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
%  output(ind).t1 = out.t;
%  output(ind).platform1 = out.platform;
%  output(ind).tension_vector1 = out.tension_vector;
%  output(ind).cables1(:) = out.cable(:);
%  
%   sim_data.dt(ind) = t1*1.2;
%   sim_data.lims(:) = [0.3;0.3];
%  sim_data.T = sim_data.dt(ind)+sim_data.shaper(2,end,ind);  
%  sim_data.motion_law_function = @ShapedActuatedDofs;
%  sim_data.cMat=TrapzCoef(sim_data.lims(:,ind));
%  out = GenerateOutputUnderActuated(ind,cdpr_p,cdpr_v,sim_data,geom_fun,ut);
%  output(ind).t2 = out.t;
%  output(ind).platform2 = out.platform;
%  output(ind).tension_vector2 = out.tension_vector;
%  output(ind).cables2(:) = out.cable(:);
       
end

end