function f_vect = StaticComGravCalibrationFunction(cdpr_p,data,x)

cdpr_p.platform.pos_G_loc = x(1:3,1);
R = RotX(x(4))*RotY(x(5));
n = length(data.dl);
f_vect = [];
for i = 1:n
    name = strcat('pose',int2str(i-1));
    pose = data.(name); 
    pose_mean = mean(pose,2);
    cdpr_v = CdprVar(cdpr_p.n_cables);
    cdpr_v = UpdateIKZeroOrd(pose_mean(1:3,1),pose_mean(4:end,1),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoadsStateSpace(cdpr_v,cdpr_p,R);
    Ja = cdpr_v.analitic_jacobian(:,1:3);
    Ju = cdpr_v.analitic_jacobian(:,4:6);
    Wa(:,1) = cdpr_v.ext_load_ss(1:3,1);
    Wu(:,1) = cdpr_v.ext_load_ss(4:6,1);

    cdpr_v.tension_vector = linsolve(Ja',Wa);
    f_vect = [f_vect; Ju'*cdpr_v.tension_vector-Wu];  
    %record.SetFrame(cdpr_v,cdpr_p);
       
end

end