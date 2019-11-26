function vect = IntegrableDirectDynamics(cdpr_p,cdpr_v,ut,...
        spline_s,time,state)
  
    for i=1:cdpr_p.n_cables
      l(i,1) = ppval(spline_s.l(i),time);
      l_d(i,1) = ppval(spline_s.l_d(i),time);
      l_d2(i,1) = ppval(spline_s.l_d2(i),time);
    end
    [cdpr_v,K0] = CalcKinZeroOrdConstr(state(1:3),state(4:6),l,cdpr_p,cdpr_v);
    [cdpr_v,K1] = CalcKinFirstOrdConstr(state(7:9),state(10:12),l_d,cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    
    Mat = [cdpr_v.platform.mass_matrix_global_ss cdpr_v.analitic_jacobian';
           cdpr_v.analitic_jacobian zeros(cdpr_p.n_cables)];
    f = [cdpr_v.platform.total_load_ss;
        l_d2-cdpr_v.analitic_jacobian_d*state(7:12)-2*ut.baum_zita.*ut.baum_omega.*K1-(ut.baum_omega.^2).*K0];
    v = linsolve(Mat,f);
    
    vect(1:6,1) = state(7:12);
    vect(7:12,1) = v(1:6);

end