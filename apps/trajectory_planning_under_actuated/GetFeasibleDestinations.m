function data = GetFeasibleDestinations(data,ws_par,cdpr_p,cdpr_v,rec,ut)

for i=1:length(data)
info_ps(i) = ExtractWsInfo(cdpr_p,ws_par,data(i).p_s);
info_pf(i) = ExtractWsInfo(cdpr_p,ws_par,data(i).p_f);
%if ()
if(strcmp(data(i).tr_tech,'IS')) % IS
  f = ExtractNaturalFrequenciesOnPath(info_ps(i).pose,info_pf(i).pose,data(i).geometricFunction,cdpr_p,cdpr_v,ut);
  [data(i).shaper(:,:),data(i).shaperL] = ComputeMultiModeShaper(f,1,data(i).IS_order,ut);
  [data(i).lim,data(i).dt] = ComputeTrajectoryLimsAndT(data,f);
  data(i).p_s = info_ps(i).pose;
  data(i).p_f = info_pf(i).pose;
else % RTR
  
end
end
  
end