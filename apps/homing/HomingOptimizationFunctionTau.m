function vector = HomingOptimizationFunctionTau(cdpr_parameters,record,vLength,vSwivelAngle,initial_guess1,opt,v)
for i=1:length(vLength)/cdpr_parameters.n_cables
  vLengthN((i-1)*cdpr_parameters.n_cables+1:i*cdpr_parameters.n_cables,1) = vLength((i-1)*cdpr_parameters.n_cables+1:i*cdpr_parameters.n_cables).*v(1:cdpr_parameters.n_cables);  
end
[sol,resnorm,residual,exitflag,output] = lsqnonlin(@(v)HomingOptimizationFunction6...
   (cdpr_parameters,record,vLengthN,vSwivelAngle,v),initial_guess1,[],[],opt);
vector = residual;
end