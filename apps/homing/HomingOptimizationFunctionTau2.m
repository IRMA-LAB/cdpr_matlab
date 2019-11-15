function vector = HomingOptimizationFunctionTau2(cdpr_parameters,record,vLength,vSwivelAngle,initial_guess1,opt,v)
for i=1:length(vLength)/cdpr_parameters.n_cables
  vLengthN((i-1)*cdpr_parameters.n_cables+1:i*cdpr_parameters.n_cables,1) = vLength((i-1)*cdpr_parameters.n_cables+1:i*cdpr_parameters.n_cables).*v(1:cdpr_parameters.n_cables);  
end
cdpr_p = cdpr_parameters;
R = RotX(v(cdpr_parameters.n_cables+1))*RotY(v(cdpr_parameters.n_cables+2));
cdpr_p.platform.gravity_acceleration = R*cdpr_parameters.platform.gravity_acceleration;
[sol,resnorm,residual,exitflag,output] = lsqnonlin(@(v)HomingOptimizationFunction6...
   (cdpr_p,record,vLengthN,vSwivelAngle,v),initial_guess1,[],[],opt);
vector = residual;
end