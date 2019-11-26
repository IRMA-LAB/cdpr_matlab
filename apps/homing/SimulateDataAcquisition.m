function [delta_l,delta_sw] = SimulateDataAcquisition(cdpr_p, cdpr_v, steps,home,rec,ut,T)

deltaT = (T(2)-T(1))/steps;
index = 0;
delta_l = zeros(cdpr_p.n_cables*2*steps,cdpr_p.n_cables);
delta_sw = delta_l;
for i = 1:cdpr_p.n_cables
    for procedure_phase = 1:2
        for j=1:steps
            index = index+1;
            tension_vector = T(1).*ones(cdpr_p.n_cables,1);
            if procedure_phase==1
                tension_vector(i) = T(1)+deltaT.*j;
            else
                tension_vector(i) = T(2)-deltaT.*j;
            end
            if (index==1)
                in_guess = home.pose;
            else
                in_guess = solution_calc;
            end
            solution_calc = fsolve(@(v)FunDkStat(cdpr_p,tension_vector,v,rec),...
                in_guess,ut.fsolve_options_grad);
            cdpr_v = UpdateIKZeroOrd(solution_calc(1:3),...
                solution_calc(4:end),cdpr_p,cdpr_v);
            for k=1:cdpr_p.n_cables
                delta_l(index,k) = cdpr_v.cable(k).complete_length-home.l(k);
                delta_sw(index,k) = cdpr_v.cable(k).swivel_ang-home.sw(k);
            end
        end
    end
end

end