function zeta = FindClosestPose(cdpr_p,cdpr_v,ws_par,l)

for i=1:ws_par.counter
        zeta(1:3,1) = ws_par.position(:,i);
        zeta(4:6,1) = cdpr_p.RotToPar(ws_par.rot_mat(:,:,i));
        cdpr_v = UpdateIKZeroOrd(zeta(1:3,1),zeta(4:6,1),cdpr_p,cdpr_v);
        cdpr_v.cable_vector;
    if(norm(cdpr_v.cable_vector-l)<0.3)
        break
    end
end

end