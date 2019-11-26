function init = GenerateInitialGuess(cdpr_p,cdpr_v,delta_l,home,rec,ut)

n = length(delta_l);
init = zeros(cdpr_p.pose_dim*n,1);
for i = 1:n
    l = home.l+delta_l(i,:)';
    if (i==1)
        in_guess = home.pose;
    else
        in_guess = solution_calc;
    end
    solution_calc = fsolve(@(v)FunDkGsL(cdpr_p,l,v,rec),...
        in_guess,ut.fsolve_options_grad);
    cdpr_v = UpdateIKZeroOrd(solution_calc(1:3),...
        solution_calc(4:end),cdpr_p,cdpr_v);
    init(cdpr_p.pose_dim*(i-1)+1:i*cdpr_p.pose_dim,1) = cdpr_v.platform.pose;
end

end