function vect = IntegrableDirectDynamicsSimplified(cdpr_p,cdpr_v,...
        spline_s,time,state)
  
    for i=1:cdpr_p.n_cables
      l_d2(i,1) = ppval(spline_s.l_d2(i),time);
    end
    cdpr_v = UpdateIKZeroOrd(state(1:3),state(4:cdpr_p.pose_dim),cdpr_p,cdpr_v);
    cdpr_v = UpdateIKFirstOrd(state(cdpr_p.pose_dim+1:cdpr_p.pose_dim+3),state(cdpr_p.pose_dim+4:end),cdpr_p,cdpr_v);
    cdpr_v.platform = cdpr_v.platform.UpdateMassMatrixStateSpace(cdpr_p);
    cdpr_v = CalcTotalLoadsStateSpace(cdpr_v,cdpr_p);
    
    Mat = [cdpr_v.platform.mass_matrix_global_ss cdpr_v.analitic_jacobian';
           cdpr_v.analitic_jacobian zeros(cdpr_p.n_cables)];
    f = [cdpr_v.platform.total_load_ss;
        l_d2-cdpr_v.analitic_jacobian_d*state(7:12)];
    v = linsolve(Mat,f);
    
    vect(1:cdpr_p.pose_dim,1) = state(cdpr_p.pose_dim+1:2*cdpr_p.pose_dim);
    vect(cdpr_p.pose_dim+1:2*cdpr_p.pose_dim,1) = v(1:cdpr_p.pose_dim);

end