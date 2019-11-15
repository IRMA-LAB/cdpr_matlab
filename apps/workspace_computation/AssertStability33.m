function stab = AssertStability33(cdpr_p,pose)

  cdpr_v = CdprVar(cdpr_p.n_cables);
  cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
  cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
  cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
      cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob,eye(3));
  T = CalcCablesTension(cdpr_v);
  g = cdpr_v.platform.pos_OG_glob;
  for i=1:cdpr_p.n_cables
    r(:,i) = cdpr_v.cable(i).pos_OA_glob-g;
    a(:,i) = cdpr_v.cable(i).pos_OA_glob-cdpr_v.cable(i).pos_BA_glob;
    
  end
  H = zeros(6);
  for i=1:cdpr_p.n_cables
    rt = Anti(r(:,i));
    gt = Anti(g);
    at = Anti(a(:,i));
    J(i,:) = [cdpr_v.cable(i).pos_BA_glob' (rt*cdpr_v.cable(i).pos_BA_glob)'];
    H=H+T(i)./norm(cdpr_v.cable(i).pos_BA_glob).*[eye(3) -rt;
                                                  rt 0.5.*(rt*(gt-at)+(gt-at)*rt)];
  end
  N = null(J);
  Hr = N'*H*N;
  eigH = eig(Hr);
  stab2 = 0;
  for i = 1:rank(eigH)
    if eigH(i) <= 0 
    stab2 = 1;
    end
  end
  [~,stab] = chol(Hr);

end