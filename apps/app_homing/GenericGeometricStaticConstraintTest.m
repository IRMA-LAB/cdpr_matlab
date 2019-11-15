function [c,ceq] = GenericGeometricStaticConstraintTest(cdpr_p,parameters,variables,mask)

parameters_index = 1;
variables_index = 1;

pose = zeros(cdpr_p.pose_dim,1);
for i=1:cdpr_p.pose_dim
  if (mask(i,1) == 1)
    pose(i,1) = parameters(parameters_index,1);
    parameters_index = parameters_index + 1;
  else
    pose(i,1) = variables(variables_index,1);
    variables_index = variables_index + 1;
  end
end
cdpr_v = CdprVar(cdpr_p.n_cables);
cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
cdpr_v = UpdateIKFirstOrd(zeros(3,1),zeros(3,1),cdpr_p,cdpr_v);

cdpr_v.ext_load = CalcExternalLoads(cdpr_p.platform,cdpr_v.platform.rot_mat,...
  cdpr_v.platform.H_mat,cdpr_v.platform.pos_PG_glob);

Ja = zeros(cdpr_p.n_cables,cdpr_p.n_cables);
Ju = zeros(cdpr_p.n_cables,cdpr_p.pose_dim-cdpr_p.n_cables);
Wa = zeros(cdpr_p.n_cables,1);
Wu = zeros(cdpr_p.pose_dim-cdpr_p.n_cables,1);
actuated_index = 1;
not_actuated_index = 1;

for i=1:cdpr_p.pose_dim
  if (mask(i,1) == 1)
    Ja(:,actuated_index) = cdpr_v.analitic_jacobian(:,i);
    Wa(actuated_index,1) = cdpr_v.ext_load(i,1);
    actuated_index = actuated_index + 1;
  else
    Ju(:,not_actuated_index) = cdpr_v.analitic_jacobian(:,i);
    Wu(not_actuated_index,1) = cdpr_v.ext_load(i,1);
    not_actuated_index = not_actuated_index + 1;
  end
end

cdpr_v.tension_vector = linsolve(Ja',Wa);
vector = Ju'*cdpr_v.tension_vector -Wu;

x_axis = [1;0;0];
y_axis = [0;1;0];
z_axis = [0;0;1];
cx = dot(cdpr_v.platform.rot_mat(:,1),x_axis);
sx = norm(cross(cdpr_v.platform.rot_mat(:,1),x_axis));
x_angle = atan2(sx,cx);
cy = dot(cdpr_v.platform.rot_mat(:,2),y_axis);
sy = norm(cross(cdpr_v.platform.rot_mat(:,2),y_axis));
y_angle = atan2(sy,cy);
cz = dot(cdpr_v.platform.rot_mat(:,3),z_axis);
sz = norm(cross(cdpr_v.platform.rot_mat(:,3),z_axis));
z_angle = atan2(sz,cz);

c = -cdpr_v.tension_vector;
c = [c;
  x_angle-pi/2;
  -x_angle-pi/2;
  y_angle-pi/2;
  -y_angle-pi/2;
  z_angle-pi/2;
  -z_angle-pi/2];
ceq = vector;

end