function [output] = RestToRestCoefficients33(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut,record)

for i = 1:sim_data.pNumber-1
    
    if (i==1) 
        coefficientGuess = [0.417395784122571;-1.312572357399364;1.663918178998519;-1.135582010024466;0.435360135386057;-0.074830280788257];
    elseif (i==2)
        coefficientGuess = [1.189158832087217;-5.347598977898814;9.433310967413712;-8.757508245067660;4.079112150357192;-0.730564410099139];
    else
        coefficientGuess = [0.539409330300287;-2.67868474884556;6.77480393429024;-7.66217879952899;4.08448744007572;-0.854193211343938];
    end
  %coefficientGuess = zeros(6,1);
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