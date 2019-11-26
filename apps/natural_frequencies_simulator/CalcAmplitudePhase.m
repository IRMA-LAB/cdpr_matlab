function data = CalcAmplitudePhase(data)

sol_pos = linsolve(data.normal_modes,data.start_conf_un_act-data.ang_par);
sol_vel = linsolve(data.normal_modes.*(2.*pi.*data.nat_freq'),data.start_conf_un_act_der); 
for i=1:length(sol_pos)
   data.phase(i,1) = atan2(sol_vel(i),sol_pos(i));
   data.amplitude(i,1) = (sol_vel(i)+sol_pos(i))./(cos(data.phase(i,1))+sin(data.phase(i,1)));
end

end