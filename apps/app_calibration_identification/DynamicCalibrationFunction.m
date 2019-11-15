function f_vect = DynamicCalibrationFunction(cdpr_p,data,x)

for i=1:cdpr_p.n_cables
    cdpr_p.cable(i).motor_cable_tau = x(1+10*(i-1));
    cdpr_p.cable(i).swivel_pulley_r = x(2+10*(i-1));
    if (i==1||i==3)
        R = RotY(x(3+10*(i-1)))*RotZ(x(4+10*(i-1)));
    else
        R = RotX(x(3+10*(i-1)))*RotZ(x(4+10*(i-1)));
    end
    cdpr_p.cable(i).vers_i = R*cdpr_p.cable(i).vers_i;
    cdpr_p.cable(i).vers_j = R*cdpr_p.cable(i).vers_j;
    cdpr_p.cable(i).vers_k = R*cdpr_p.cable(i).vers_k;
    cdpr_p.cable(i).pos_D_glob = x(5+10*(i-1):7+10*(i-1));
    cdpr_p.cable(i).pos_A_loc = x(8+10*(i-1):10+10*(i-1));
end
n = length(data.dl);
f_vect = [];
f_vect2 = [];
f_vect3 = [];
l_mean_0= [];
for i = 1:n
    name = strcat('pose',int2str(i-1));
    name2 = strcat('pose_d',int2str(i-1));
    name3 = strcat('pose_dd',int2str(i-1));
    pose = data.(name);
    pose_d = data.(name2);
    pose_dd = data.(name3);
    l_now = [];
    dl_now = [];
    ddl_now = [];
    f_vect_loc = [];
    f_vect_loc2 = [];
    f_vect_loc3 = [];
    for j = 1:length(pose)
        cdpr_v = CdprVar(cdpr_p.n_cables);
        cdpr_v = UpdateIKZeroOrd(pose(1:3,j),pose(4:end,j),cdpr_p,cdpr_v);
        cdpr_v = UpdateIKFirstOrd(pose_d(1:3,j),pose_d(4:end,j),cdpr_p,cdpr_v);
        cdpr_v = UpdateIKSecondOrd(pose_dd(1:3,j),pose_dd(4:end,j),cdpr_p,cdpr_v);
        for k=1:cdpr_p.n_cables
            l(k,1) = cdpr_v.cable(k).length;
            dl(k,1) = cdpr_v.cable(k).speed;
            ddl(k,1) = cdpr_v.cable(k).acceleration;
        end
        l_now = [l_now l];
        dl_now = [dl_now dl];
        ddl_now = [ddl_now ddl];
    end
    l_now_mean = mean(l_now,2);
    if (i==1)
        l_mean_0 = l_now_mean;
    end
    tau = [cdpr_p.cable(1).motor_cable_tau;cdpr_p.cable(2).motor_cable_tau;cdpr_p.cable(3).motor_cable_tau];
    for j = 1:length(pose)
        f_vect_loc = [f_vect_loc;l_now(:,j)-(l_mean_0+tau.*data.dl(:,i))];
        f_vect_loc2 = [f_vect_loc2;dl_now(:,j)];
        f_vect_loc3 = [f_vect_loc3;ddl_now(:,j)];
    end
    f_vect = [f_vect;f_vect_loc];
    f_vect2 = [f_vect2;f_vect_loc2];
    f_vect3 = [f_vect3;f_vect_loc3];
end

f_vect = f_vect+f_vect2;

end