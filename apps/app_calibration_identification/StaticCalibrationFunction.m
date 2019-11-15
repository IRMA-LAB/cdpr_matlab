function f_vect = StaticCalibrationFunction(cdpr_p,p,d_counts,x)

l_0 = zeros(3,1);
for i=1:cdpr_p.n_cables
    l_0(i,1) = x(1+11*(i-1));
    cdpr_p.cable(i).motor_cable_tau = x(2+11*(i-1));
    cdpr_p.cable(i).swivel_pulley_r = x(3+11*(i-1));
    R = RotX(x(4+11*(i-1)))*RotZ(x(5+11*(i-1)));
    cdpr_p.cable(i).vers_i = R*cdpr_p.cable(i).vers_i;
    cdpr_p.cable(i).vers_j = R*cdpr_p.cable(i).vers_j;
    cdpr_p.cable(i).vers_k = R*cdpr_p.cable(i).vers_k;
    cdpr_p.cable(i).pos_D_glob = x(6+11*(i-1):8+11*(i-1));
    cdpr_p.cable(i).pos_A_loc = x(9+11*(i-1):11+11*(i-1));
end
n = length(p);
f_vect = zeros(3*n,1);
for i = 1:n
  cdpr_v = CdprVar(cdpr_p.n_cables);
  cdpr_v = UpdateIKZeroOrd(p(1:3,i),p(4:end,i),cdpr_p,cdpr_v);
  for j=1:cdpr_p.n_cables
    f_vect(j+(i-1)*cdpr_p.n_cables) = cdpr_v.cable(j).length-...
      (l_0(j)-cdpr_p.cable(j).motor_cable_tau*d_counts(j,i));
  end
end

end