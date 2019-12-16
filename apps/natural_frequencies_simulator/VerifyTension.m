function [data,K] = VerifyTension(cdpr_v,cdpr_p,p,dp,ddp,l)

for i=1:length(p)
    cdpr_v = UpdateIKZeroOrd(p(1:3,i),p(4:end,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(dp(1:3,i),dp(4:end,i),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKSecondOrd(ddp(1:3,i),ddp(4:end,i),cdpr_p,cdpr_v);
    
    [~,K.zero(:,i)] = CalcKinZeroOrdConstr(p(1:3,i),p(4:end,i),l,cdpr_p,cdpr_v);
    [~,K.one(:,i)] = CalcKinFirstOrdConstr(dp(1:3,i),dp(4:end,i),zeros(cdpr_p.n_cables),cdpr_p,cdpr_v);
    [~,K.two(:,i)] = CalcKinSecondOrdConstr(ddp(1:3,i),ddp(4:end,i),zeros(cdpr_p.n_cables),cdpr_p,cdpr_v);
    
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateDynamicsStateSpace(cdpr_p.underactuated_platform,...
        cdpr_v.analitic_jacobian,cdpr_v.platform.mass_matrix_global_ss,cdpr_v.platform.total_load_ss);
    cdpr_v = CalcCablesDynamicTensionStateSpace(cdpr_v);

    data(:,i) = cdpr_v.tension_vector+cdpr_v.underactuated_platform.analitic_parallel'*...
    (cdpr_v.platform.mass_matrix_global_ss*cdpr_v.underactuated_platform.analitic_orthogonal*...
    ddp(4:end,i)-cdpr_v.platform.total_load_ss);
end

end