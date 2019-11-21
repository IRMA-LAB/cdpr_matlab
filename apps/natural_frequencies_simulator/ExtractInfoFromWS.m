function data = ExtractInfoFromWS(cdpr_v,cdpr_p,ws_data,ut,rec,i,c,dc)

data.pos = ws_data.position(:,i);
data.rot_mat = ws_data.rot_mat(:,:,i);
data.ang_par = cdpr_p.RotToPar(data.rot_mat);

cdpr_v = UpdateIKZeroOrd(data.pos,data.ang_par,cdpr_p,cdpr_v);

data.nat_freq = ws_data.nat_freq(:,i);
data.normal_modes = ws_data.normal_modes(:,:,i);
data.l = cdpr_v.cable_vector;

sign_p = 2*rand(3,1)-1;
sign_p = sign_p./abs(sign_p);
data.start_conf_un_act = data.ang_par+c.*(pi/180).*sign_p;
start_conf_act = fsolve(@(v) FunDkUnActL(cdpr_p,data.l,data.start_conf_un_act,v,rec),...
    data.pos,ut.fsolve_options_grad);
data.p0 = [start_conf_act;data.start_conf_un_act];
cdpr_v = UpdateIKZeroOrd(start_conf_act,data.start_conf_un_act,cdpr_p,cdpr_v);
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateAnaliticJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.analitic_jacobian);
sign_v = rand(3,1);
sign_v = sign_v./abs(sign_v);
data.start_conf_un_act_der = dc.*(pi/180).*sign_v;
data.v0 = cdpr_v.underactuated_platform.analitic_orthogonal*data.start_conf_un_act_der;

data = CalcAmplitudePhase(data);

end