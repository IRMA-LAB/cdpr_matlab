function [vector,matrix] = HomingOptimizationFunction66(cdpr_p,record,dl,ds,v)
 
[dim,~] = size(dl); dim = dim/cdpr_p.n_cables;
cdpr_v = CdprVar(cdpr_p.n_cables);
equationsXstage = cdpr_p.n_cables+6;
vector = zeros(dim*equationsXstage,1);
matrix = zeros(dim*equationsXstage,length(v));
for j = 1:dim    
  pose = v(2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim,1);
  cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
  cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);
  cdpr_v.ext_load = CalcExternalLoads2(cdpr_p.platform,cdpr_v.platform.rot_mat,...
      cdpr_v.platform.pos_PG_glob,eye(3));
  Ja(:,1:cdpr_p.n_cables) = cdpr_v.geometric_jacobian(:,1:3);
  Wa(1:cdpr_p.n_cables,1) = cdpr_v.ext_load(1:3,1);
  Ju(:,1:cdpr_p.pose_dim-cdpr_p.n_cables) = cdpr_v.geometric_jacobian(:,4:cdpr_p.pose_dim);
  Wu(1:6-cdpr_p.n_cables,1) = cdpr_v.ext_load(4:end,1);
  cdpr_v.tension_vector = linsolve(Ja',Wa);
  [J_sl,J_q] = CalcOptimizationJacobians(cdpr_v,Ja,Ju,cdpr_p.platform.mass.*cdpr_p.platform.gravity_acceleration);
   for i=1:cdpr_p.n_cables
    vector(equationsXstage*(j-1)+i,1) = 10.*(cdpr_v.cable(i).swivel_ang-(v(i)+ds((j-1)*cdpr_p.n_cables+i))); 
    vector(equationsXstage*(j-1)+cdpr_p.n_cables+i,1) = 100.*(cdpr_v.cable(i).length-(v(cdpr_p.n_cables+i)+dl((j-1)*cdpr_p.n_cables+i)));
  end
  vector(equationsXstage*(j-1)+2*cdpr_p.n_cables+1:equationsXstage*j,1) = (Ju'*cdpr_v.tension_vector -Wu);
  matrix(equationsXstage*(j-1)+1:equationsXstage*(j-1)+cdpr_p.n_cables,1:cdpr_p.n_cables) = -10.*eye(cdpr_p.n_cables);
  matrix(equationsXstage*(j-1)+1+cdpr_p.n_cables:equationsXstage*(j-1)+2*cdpr_p.n_cables,1+cdpr_p.n_cables:2*cdpr_p.n_cables) = -100.*eye(cdpr_p.n_cables);
  matrix(equationsXstage*(j-1)+1:equationsXstage*(j-1)+2*cdpr_p.n_cables,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim) = J_sl;
  matrix(equationsXstage*(j-1)+1:equationsXstage*(j-1)+cdpr_p.n_cables,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim) = 10.*matrix(equationsXstage*(j-1)+1:equationsXstage*(j-1)+cdpr_p.n_cables,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim);
  matrix(equationsXstage*(j-1)+1+cdpr_p.n_cables:equationsXstage*(j-1)+2.*cdpr_p.n_cables,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim) = 100.*matrix(equationsXstage*(j-1)+1+cdpr_p.n_cables:equationsXstage*(j-1)+2.*cdpr_p.n_cables,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim);
  matrix(equationsXstage*(j-1)+2*cdpr_p.n_cables+1:equationsXstage*j,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim) = J_q;
%   if (j==1)
%     record.SetFrame(cdpr_v,cdpr_p);
%     %vector(1:equationsXstage,1) = 100.*vector(1:equationsXstage,1);
%     %matrix(1:equationsXstage,:) = 100.*matrix(1:equationsXstage,:);
%   end
end
   
end