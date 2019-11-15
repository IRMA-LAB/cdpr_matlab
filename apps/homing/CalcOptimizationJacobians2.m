function [J_sl,J_q] = CalcOptimizationJacobians2(cdpr_v,Ja,Ju,mg)

n = length(Ja);
m = length([Ja Ju]);
J_sl = zeros(2*n,m);
for i=1:n
  dsdq(i,:) = cdpr_v.cable(i).vers_w'*[eye(3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat]./...
    dot(cdpr_v.cable(i).pos_DA_glob,cdpr_v.cable(i).vers_u);
  dphidq(i,:) = cdpr_v.cable(i).vers_n'*[eye(3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat]./...
    norm(cdpr_v.cable(i).pos_BA_glob);
  dldq(i,:) = cdpr_v.cable(i).vers_rho'*[eye(3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat];
  jj = sin(cdpr_v.cable(i).tan_ang).*cdpr_v.cable(i).vers_w*dsdq(i,:);
  dJTp(:,i,:) = jj;
  dadq = [zeros(3,3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat];
  dJTo(:,i,:) = Anti(cdpr_v.cable(i).pos_PA_glob)*jj-Anti(cdpr_v.cable(i).vers_rho)*dadq;
end

J_sl(1:n,1:m) = dsdq;
J_sl(n+1:end,1:m) = dldq;
dJdq = [dJTp;dJTo];
dJa(:,:,:) = dJdq(1:n,:,:);
dJu(:,:,:) = dJdq(n+1:end,:,:);
dfdq = [zeros(3,6);zeros(3,3) -Anti(mg)*Anti(cdpr_v.platform.pos_PG_glob)*cdpr_v.platform.H_mat];
dWa(:,:) = dfdq(1:n,:);
dWu(:,:) = dfdq(n+1:end,:);

iJ = inv(Ja');
j0u = zeros(m-n,m);
j0a = zeros(n,m);
for i=1:m
  j0u(:,i) = dJu(:,:,i)*cdpr_v.tension_vector;
  j0a(:,i) = dJa(:,:,i)*cdpr_v.tension_vector;
end

J_q = j0u +Ju'*iJ*(-j0a+dWa)-dWu;

end