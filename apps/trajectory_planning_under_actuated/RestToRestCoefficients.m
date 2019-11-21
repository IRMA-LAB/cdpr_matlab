function [output] = RestToRestCoefficients(cdpr_p,cdpr_v,...
     sim_data,geom_fun,ut,record)

for i = 1:sim_data.pNumber-1
    if i==1
        coefficientGuess  = [0.408342931723560;-1.187730121374331;1.699178913462222;-1.013733090161724;0.271144177414540;-0.024023035317960];
    elseif i==2
        coefficientGuess  = [1.10493640953106;-5.12295800499690;10.2958654505579;-11.0432673147581;6.00524619123169;-1.28335619937834];
    elseif i ==3
        coefficientGuess = [0.508084420529509;-2.48342052237961;6.12068968752132;-6.51844045706564;3.30451923792074;-0.668003796248589];
    end
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