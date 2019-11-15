function vector = HomingOptimizationFunctionNoSwiv(cdpr_p,record,dl,v)
 
[dim,~] = size(dl); dim = dim/cdpr_p.n_cables;
cdpr_v = CdprVar(cdpr_p.n_cables);
equationsXstage = 6;
vector = zeros(dim*equationsXstage,1);
for j = 1:dim    
  pose = v(cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:cdpr_p.n_cables+j*cdpr_p.pose_dim,1);
  cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
  cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
  cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
    cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob,eye(3));
  Ja(:,1:cdpr_p.n_cables) = cdpr_v.analitic_jacobian(:,1:3);
  Wa(1:cdpr_p.n_cables,1) = cdpr_v.ext_load(1:3,1);
  Ju(:,1:cdpr_p.pose_dim-cdpr_p.n_cables) = cdpr_v.analitic_jacobian(:,4:cdpr_p.pose_dim);
  Wu(1:6-cdpr_p.n_cables,1) = cdpr_v.ext_load(4:end,1);
  cdpr_v.tension_vector = linsolve(Ja',Wa);
  for (i=1:cdpr_p.n_cables)
    vector(equationsXstage*(j-1)+i,1) = cdpr_v.cable(i).length-(v(i)+dl((j-1)*cdpr_p.n_cables+i));
  end
  vector(equationsXstage*(j-1)+cdpr_p.n_cables+1:equationsXstage*j,1) = Ju'*cdpr_v.tension_vector -Wu;
  if (j==1)
    record.SetFrame(cdpr_v,cdpr_p);
  end
end
   
end